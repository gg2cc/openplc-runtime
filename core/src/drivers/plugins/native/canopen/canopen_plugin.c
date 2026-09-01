/**
 * @file canopen_plugin.c
 * @brief Native CANopen plugin adapter for OpenPLC runtime.
 *
 * This adapter converts editor-generated CANopen JSON into a runtime object
 * dictionary and PDO configuration and binds it to the CANopenNode stack.
 * The plugin keeps the lifecycle contract from the runtime while creating one
 * CO_t instance per configured bus so the runtime exposes a real CANopen node
 * implementation instead of a passive config parser.
 */

#include "canopen_plugin.h"
#include "../can/can_netlink.h"
#include "../can/can_socket.h"
#include "../cjson/cJSON.h"
#include "canopen_config.h"
#ifndef CO_CONFIG_SDO_CLI
#define CO_CONFIG_SDO_CLI                                                                          \
    (CO_CONFIG_SDO_CLI_ENABLE | CO_CONFIG_SDO_CLI_SEGMENTED | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif
#ifndef CO_CONFIG_FIFO
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif

#include "OD.h"
#include "libs/CANopenLinux/CO_epoll_interface.h"
#include "libs/CANopenNode/301/CO_ODinterface.h"
#include "libs/CANopenNode/CANopen.h"
#include "plugin_logger.h"
#include <linux/can.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#define CANOPEN_NMT_CONTROL                                                                        \
    (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR |              \
     CO_ERR_REG_COMMUNICATION)
#define CANOPEN_LOCAL_RPDO_MAX 4
#define CANOPEN_LOCAL_RPDO_MAX_MAPPINGS 8
#define CANOPEN_LOCAL_TPDO_MAX 4
#define CANOPEN_LOCAL_TPDO_MAX_MAPPINGS 8

static inline int canopen_min_int(int a, int b)
{
    return (a < b) ? a : b;
}

typedef struct
{
    bool active;
    bool read;
    uint8_t node_id;
    uint16_t index;
    uint8_t sub_index;
    uint8_t payload[64];
    size_t payload_len;
    size_t expected_len;
    uint32_t elapsed_ms;
    uint32_t timeout_ms;
    uint8_t retry_count;
    uint8_t max_retries;
    CO_SDO_abortCode_t abort_code;
    char last_error[64];
} canopen_sdo_transaction_t;

typedef struct canopen_runtime_bus_s canopen_runtime_bus_t;

typedef enum
{
    CANOPEN_DATA_TYPE_INVALID = 0,
    CANOPEN_DATA_TYPE_BOOL,
    CANOPEN_DATA_TYPE_I8,
    CANOPEN_DATA_TYPE_U8,
    CANOPEN_DATA_TYPE_I16,
    CANOPEN_DATA_TYPE_U16,
    CANOPEN_DATA_TYPE_I32,
    CANOPEN_DATA_TYPE_U32,
    CANOPEN_DATA_TYPE_I64,
    CANOPEN_DATA_TYPE_U64,
    CANOPEN_DATA_TYPE_F32,
    CANOPEN_DATA_TYPE_F64
} canopen_data_type_t;

static canopen_data_type_t canopen_parse_data_type(const char *data_type)
{
    if (!data_type || data_type[0] == '\0' || strcasecmp(data_type, "bool") == 0)
        return CANOPEN_DATA_TYPE_BOOL;
    if (strcasecmp(data_type, "i8") == 0)
        return CANOPEN_DATA_TYPE_I8;
    if (strcasecmp(data_type, "u8") == 0)
        return CANOPEN_DATA_TYPE_U8;
    if (strcasecmp(data_type, "i16") == 0)
        return CANOPEN_DATA_TYPE_I16;
    if (strcasecmp(data_type, "u16") == 0)
        return CANOPEN_DATA_TYPE_U16;
    if (strcasecmp(data_type, "i32") == 0)
        return CANOPEN_DATA_TYPE_I32;
    if (strcasecmp(data_type, "u32") == 0)
        return CANOPEN_DATA_TYPE_U32;
    if (strcasecmp(data_type, "i64") == 0)
        return CANOPEN_DATA_TYPE_I64;
    if (strcasecmp(data_type, "u64") == 0)
        return CANOPEN_DATA_TYPE_U64;
    if (strcasecmp(data_type, "f32") == 0)
        return CANOPEN_DATA_TYPE_F32;
    if (strcasecmp(data_type, "f64") == 0)
        return CANOPEN_DATA_TYPE_F64;
    return CANOPEN_DATA_TYPE_INVALID;
}

static uint16_t canopen_data_type_bit_length(canopen_data_type_t data_type)
{
    switch (data_type)
    {
    case CANOPEN_DATA_TYPE_BOOL:
        return 1U;
    case CANOPEN_DATA_TYPE_I8:
    case CANOPEN_DATA_TYPE_U8:
        return 8U;
    case CANOPEN_DATA_TYPE_I16:
    case CANOPEN_DATA_TYPE_U16:
        return 16U;
    case CANOPEN_DATA_TYPE_I32:
    case CANOPEN_DATA_TYPE_U32:
    case CANOPEN_DATA_TYPE_F32:
        return 32U;
    case CANOPEN_DATA_TYPE_I64:
    case CANOPEN_DATA_TYPE_U64:
    case CANOPEN_DATA_TYPE_F64:
        return 64U;
    default:
        return 0U;
    }
}

static uint64_t canopen_normalize_value(uint64_t value, canopen_data_type_t data_type)
{
    uint16_t bit_length = canopen_data_type_bit_length(data_type);
    bool is_signed      = data_type == CANOPEN_DATA_TYPE_I8 || data_type == CANOPEN_DATA_TYPE_I16 ||
                          data_type == CANOPEN_DATA_TYPE_I32 || data_type == CANOPEN_DATA_TYPE_I64;

    if (is_signed && bit_length > 0U && bit_length < 64U &&
        (value & ((uint64_t)1U << (bit_length - 1U))) != 0U)
    {
        value |= ~(((uint64_t)1U << bit_length) - 1U);
    }
    return value;
}

typedef struct
{
    bool valid;
    uint16_t bit_offset;
    uint16_t bit_length;
    uint16_t plc_index;
    uint8_t plc_bit;
    uint8_t plc_type;
    canopen_data_type_t data_type;
} canopen_input_binding_t;

typedef struct
{
    bool valid;
    uint16_t bit_offset;
    uint16_t bit_length;
    uint16_t plc_index;
    uint8_t plc_bit;
    uint8_t plc_type;
    canopen_data_type_t data_type;
} canopen_output_field_t;

typedef struct
{
    bool valid;
    uint32_t cob_id;
    uint8_t dlc;
    uint8_t field_count;
    canopen_output_field_t fields[CANOPEN_LOCAL_TPDO_MAX_MAPPINGS];
} canopen_output_tpdo_t;

typedef struct
{
    canopen_runtime_bus_t *runtime;
    uint8_t rpdo_slot;
} canopen_rpdo_callback_context_t;

struct canopen_runtime_bus_s
{
    CO_t *co;
    bool initialized;
    bool epoll_ready;
    bool startup_confirmed;
    bool communication_fault;
    bool reconnect_required;
    bool bootup_seen;
    bool startup_sdo_sent;
    bool startup_reset_sent;
    uint8_t node_id;
    uint32_t bitrate;
    uint32_t last_start_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_bootup_ms;
    uint8_t retry_count;
    int fd;
    CO_epoll_t epoll;
    pthread_mutex_t stack_mutex;
    canopen_sdo_transaction_t sdo_transaction;
    canopen_input_binding_t input_bindings[CANOPEN_LOCAL_RPDO_MAX][CANOPEN_LOCAL_RPDO_MAX_MAPPINGS];
    uint8_t input_binding_count[CANOPEN_LOCAL_RPDO_MAX];
    uint8_t input_rpdo_node_id[CANOPEN_LOCAL_RPDO_MAX];
    canopen_rpdo_callback_context_t rpdo_callback_context[CANOPEN_LOCAL_RPDO_MAX];
    canopen_output_tpdo_t output_tpdos[CANOPEN_LOCAL_TPDO_MAX];
    uint8_t output_tpdo_count;
};

static int canopen_parse_iec_address(const char *address, char *prefix, size_t prefix_len,
                                     int *index, int *bit);
static void canopen_send_configured_sdos(const canopen_bus_config_t *bus,
                                         canopen_runtime_bus_t *runtime);
static bool canopen_has_node_guarding_slaves(const canopen_bus_config_t *bus);
static bool canopen_configure_node_guarding(const canopen_bus_config_t *bus, CO_t *co);
static void canopen_rpdo_signal_pre(void *object);
static void canopen_configure_local_rpdos(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime);
static void canopen_configure_local_tpdos(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime);
static void sync_plc_image_to_canopen_bus(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime);
static int canopen_parse_status_address(const char *address, uint16_t *index);
static void canopen_update_status_outputs(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime);

static plugin_runtime_args_t g_args;
static plugin_logger_t g_logger;
static canopen_config_t g_config;
static canopen_runtime_bus_t g_runtime_buses[MAX_CANOPEN_BUSES];
static int g_runtime_bus_count = 0;
static pthread_t g_canopen_rx_thread;
static volatile bool g_canopen_rx_running = false;

void log_printf(int priority, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsyslog(priority, format, args);
    va_end(args);
}

static void reset_runtime_state(void)
{
    memset(g_runtime_buses, 0, sizeof(g_runtime_buses));
    g_runtime_bus_count  = 0;
    g_canopen_rx_running = false;
}

static uint32_t canopen_now_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        return 0U;
    }
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL);
}

static void canopen_apply_heartbeat_defaults(const canopen_bus_config_t *bus, CO_t *co)
{
    if (bus == NULL || co == NULL)
    {
        return;
    }

    uint16_t heartbeat_ms = (bus->heartbeat_ms > 0U) ? (uint16_t)bus->heartbeat_ms : 1000U;
    OD_PERSIST_COMM.x1017_producerHeartbeatTime = heartbeat_ms;

    uint8_t enabled_count = 0U;
    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        if (enabled_count < OD_CNT_ARR_1016)
        {
            uint16_t slave_heartbeat_ms = 0U;
            if (strcmp(slave->protection_mode, "heartbeat_producer") == 0)
            {
                slave_heartbeat_ms = (uint16_t)slave->heartbeat_producer_time_ms;
            }
            else
            {
                for (int i = 0; i < slave->sdo_count; i++)
                {
                    if (slave->sdo[i].index == 0x1017U && slave->sdo[i].sub_index == 0U)
                    {
                        slave_heartbeat_ms = (uint16_t)slave->sdo[i].default_value;
                        break;
                    }
                }
            }
            if (slave_heartbeat_ms == 0U)
            {
                plugin_logger_warn(&g_logger,
                                   "CANopen heartbeat: missing slave 0x1017 value for node_id=%u",
                                   slave->node_id);
                continue;
            }

            /* Each 0x1016 entry stores the node ID in bits 31..16 and the heartbeat timeout
             * in milliseconds in bits 15..0. */
            uint16_t consumer_timeout_ms = (uint16_t)(slave_heartbeat_ms * 2U);
            OD_PERSIST_COMM.x1016_consumerHeartbeatTime[enabled_count] =
                (((uint32_t)slave->node_id << 16U) | (uint32_t)consumer_timeout_ms);
            plugin_logger_info(
                &g_logger,
                "CANopen heartbeat: consumer_timeout_ms=%u slave_producer_ms=%u node_id=%u "
                "bus=%s",
                consumer_timeout_ms, slave_heartbeat_ms, slave->node_id, bus->name);
            enabled_count++;
        }
    }
    OD_PERSIST_COMM.x1016_consumerHeartbeatTime_sub0 = enabled_count;

    plugin_logger_info(&g_logger,
                       "CANopen heartbeat: producer_heartbeat_ms=%u monitored_slaves=%u bus=%s",
                       heartbeat_ms, enabled_count, bus->name);

    if (co->NMT != NULL)
    {
        co->NMT->HBproducerTime_us = (uint32_t)heartbeat_ms * 1000U;
    }
}

static bool canopen_has_node_guarding_slaves(const canopen_bus_config_t *bus)
{
    if (bus == NULL)
    {
        return false;
    }

    for (int i = 0; i < bus->slave_count; i++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[i];
        if (slave->enabled && strcmp(slave->protection_mode, "heartbeat_producer") != 0)
        {
            return true;
        }
    }
    return false;
}

static bool canopen_configure_node_guarding(const canopen_bus_config_t *bus, CO_t *co)
{
    if (!canopen_has_node_guarding_slaves(bus))
    {
        return true;
    }

    if (co == NULL || co->NGmaster == NULL)
    {
        plugin_logger_error(&g_logger, "CANopen node guarding master unavailable for bus=%s",
                            bus->name);
        return false;
    }

    uint8_t index = 0U;
    for (int i = 0; i < bus->slave_count; i++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[i];
        if (!slave->enabled || strcmp(slave->protection_mode, "heartbeat_producer") == 0 ||
            slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        if (index >= CO_CONFIG_NODE_GUARDING_MASTER_COUNT)
        {
            plugin_logger_error(&g_logger, "Too many node guarding slaves configured for bus=%s",
                                bus->name);
            return false;
        }

        CO_ReturnError_t err = CO_nodeGuardingMaster_initNode(co->NGmaster, index, slave->node_id,
                                                              (uint16_t)slave->node_guard_time_ms);
        if (err != CO_ERROR_NO)
        {
            plugin_logger_error(&g_logger, "Node guarding init failed for bus=%s node_id=%u err=%d",
                                bus->name, slave->node_id, err);
            return false;
        }
        plugin_logger_info(&g_logger,
                           "Node guarding master configured: bus=%s node_id=%u guard_time_ms=%u",
                           bus->name, slave->node_id, slave->node_guard_time_ms);
        index++;
    }
    return true;
}

static void canopen_hb_state_changed(uint8_t nodeId, uint8_t idx, CO_NMT_internalState_t state,
                                     void *object)
{
    (void)idx;
    canopen_runtime_bus_t *runtime = (canopen_runtime_bus_t *)object;
    if (runtime == NULL)
    {
        return;
    }

    runtime->last_heartbeat_ms = canopen_now_ms();

    if (state == CO_NMT_INITIALIZING)
    {
        runtime->bootup_seen        = true;
        runtime->reconnect_required = true;
        runtime->startup_confirmed  = false;
        runtime->startup_sdo_sent   = false;
        runtime->startup_reset_sent = false;
        runtime->last_bootup_ms     = runtime->last_heartbeat_ms;
        plugin_logger_warn(
            &g_logger,
            "CANopen lifecycle: boot-up detected from node %u, restarting start sequence", nodeId);
        return;
    }

    if (state == CO_NMT_OPERATIONAL)
    {
        runtime->startup_confirmed  = true;
        runtime->reconnect_required = false;
        runtime->startup_sdo_sent   = false;
        runtime->retry_count        = 0U;
        plugin_logger_info(
            &g_logger, "CANopen lifecycle: node %u reached Operational; startup confirmed", nodeId);
        return;
    }

    if (state == CO_NMT_STOPPED || state == CO_NMT_PRE_OPERATIONAL)
    {
        runtime->startup_confirmed  = false;
        runtime->reconnect_required = true;
        runtime->startup_sdo_sent   = false;
        runtime->startup_reset_sent = false;
        plugin_logger_warn(&g_logger, "CANopen lifecycle: node %u not operational yet (state=%d)",
                           nodeId, state);
    }
}

static void canopen_runtime_lifecycle_tick(const canopen_bus_config_t *bus,
                                           canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL || runtime->co == NULL)
    {
        return;
    }

#if ((CO_CONFIG_NODE_GUARDING) & CO_CONFIG_NODE_GUARDING_MASTER_ENABLE) != 0
    if (canopen_has_node_guarding_slaves(bus) && runtime->co->NGmaster != NULL)
    {
        if (runtime->co->NGmaster->allMonitoredOperational)
        {
            runtime->startup_confirmed  = true;
            runtime->reconnect_required = false;
            runtime->retry_count        = 0U;
        }
        else if (runtime->startup_confirmed)
        {
            runtime->startup_confirmed  = false;
            runtime->reconnect_required = true;
            runtime->startup_sdo_sent   = false;
            runtime->startup_reset_sent = false;
            plugin_logger_warn(&g_logger,
                               "CANopen lifecycle: node guarding lost operational state on bus=%s; "
                               "scheduling reconnect...",
                               bus->name);
        }
        else if (!runtime->reconnect_required)
        {
            uint32_t now_ms = canopen_now_ms();
            if (runtime->last_start_ms > 0U && (now_ms - runtime->last_start_ms) > 5000U)
            {
                runtime->reconnect_required = true;
                runtime->startup_sdo_sent   = false;
                runtime->startup_reset_sent = false;
                plugin_logger_warn(&g_logger,
                                   "CANopen lifecycle: node guarding startup timeout on bus=%s; "
                                   "retrying start",
                                   bus->name);
            }
        }
        return;
    }
#endif

    if (runtime->co->HBcons == NULL)
    {
        return;
    }

    if (runtime->co->HBcons->numberOfMonitoredNodes == 0U)
    {
        runtime->startup_confirmed  = true;
        runtime->reconnect_required = false;
        return;
    }

    bool all_operational = true;
    bool any_bootup      = false;
    bool any_timeout     = false;
    uint32_t now_ms      = canopen_now_ms();

    for (uint8_t i = 0; i < runtime->co->HBcons->numberOfMonitoredNodes; i++)
    {
        CO_HBconsNode_t *node = &runtime->co->HBconsMonitoredNodes[i];
        if (node == NULL || node->nodeId == 0U)
        {
            continue;
        }

        if (node->HBstate == CO_HBconsumer_TIMEOUT)
        {
            any_timeout     = true;
            all_operational = false;
        }
        if (node->NMTstate == CO_NMT_INITIALIZING)
        {
            any_bootup      = true;
            all_operational = false;
        }
        if (node->NMTstate != CO_NMT_OPERATIONAL)
        {
            all_operational = false;
        }
    }

    if (any_bootup)
    {
        if (runtime->startup_confirmed || !runtime->reconnect_required)
        {
            plugin_logger_warn(
                &g_logger,
                "CANopen lifecycle: boot-up detected on bus=%s (slave in Pre-Operational), "
                "restarting start sequence...",
                bus->name);
        }
        runtime->bootup_seen        = true;
        runtime->reconnect_required = true;
        runtime->startup_confirmed  = false;
        runtime->startup_sdo_sent   = false;
        runtime->startup_reset_sent = false;
        runtime->last_bootup_ms     = now_ms;
    }
    if (any_timeout)
    {
        if (runtime->startup_confirmed)
        {
            runtime->startup_confirmed  = false;
            runtime->reconnect_required = true;
            runtime->startup_sdo_sent   = false;
            runtime->startup_reset_sent = false;
            runtime->retry_count++;
            plugin_logger_warn(
                &g_logger,
                "CANopen lifecycle: heartbeat timeout on bus=%s; scheduling reconnect...",
                bus->name);
        }
        else if (!runtime->reconnect_required)
        {
            if (runtime->last_start_ms > 0U && (now_ms - runtime->last_start_ms) > 3000U)
            {
                runtime->reconnect_required = true;
                runtime->startup_sdo_sent   = false;
                runtime->startup_reset_sent = false;
                plugin_logger_warn(
                    &g_logger,
                    "CANopen lifecycle: heartbeat timeout recovery failed on bus=%s; retrying...",
                    bus->name);
            }
        }
    }
    if (all_operational && !any_timeout && !any_bootup)
    {
        if (!runtime->startup_confirmed)
        {
            plugin_logger_info(
                &g_logger, "CANopen lifecycle: all monitored nodes confirmed Operational on bus=%s",
                bus->name);
        }
        runtime->startup_confirmed  = true;
        runtime->reconnect_required = false;
        runtime->retry_count        = 0U;
    }
}

static void canopen_start_configured_slaves(const canopen_bus_config_t *bus, CO_t *co,
                                            canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || co == NULL || co->NMT == NULL || runtime == NULL)
    {
        return;
    }

    uint32_t now_ms = canopen_now_ms();
    if (runtime != NULL && runtime->startup_confirmed && !runtime->reconnect_required)
    {
        return;
    }
    if (runtime != NULL && runtime->last_start_ms != 0U && !runtime->reconnect_required)
    {
        return;
    }
    if (runtime != NULL && runtime->last_start_ms != 0U &&
        (now_ms - runtime->last_start_ms) < 1000U)
    {
        return;
    }

    plugin_logger_info(&g_logger,
                       "CANopen standard start sequence initiating for bus=%s (recovery_count=%u)",
                       bus->name, runtime->retry_count + 1);

    /* Step 0: Reset the slave first to ensure a clean power-on or restart state (NMT 0x81). */
    if (!runtime->startup_reset_sent)
    {
        bool reset_sent = false;
        for (int s = 0; s < bus->slave_count; s++)
        {
            const canopen_slave_config_t *slave = &bus->slaves[s];
            if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
            {
                continue;
            }

            pthread_mutex_lock(&runtime->stack_mutex);
            CO_ReturnError_t err = CO_NMT_sendCommand(co->NMT, CO_NMT_RESET_NODE, slave->node_id);
            pthread_mutex_unlock(&runtime->stack_mutex);
            if (err == CO_ERROR_NO)
            {
                reset_sent = true;
                runtime->communication_fault = false;
                plugin_logger_info(&g_logger,
                                   "NMT Reset Node sent: bus=%s slave=%s node_id=%u command=0x81",
                                   bus->name, slave->name, slave->node_id);
            }
            else
            {
                runtime->communication_fault = true;
            }
        }

        runtime->startup_reset_sent = true;
        if (reset_sent)
        {
            usleep(50000U);
        }
    }

    /* Step 1: Standard CANopen sequence: Ensure slave is in Pre-Operational state (NMT 0x80) */
    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        pthread_mutex_lock(&runtime->stack_mutex);
        CO_ReturnError_t err =
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_PRE_OPERATIONAL, slave->node_id);
        pthread_mutex_unlock(&runtime->stack_mutex);
        if (err == CO_ERROR_NO)
        {
            runtime->communication_fault = false;
            plugin_logger_info(
                &g_logger,
                "NMT Enter Pre-Operational sent: bus=%s slave=%s node_id=%u command=0x80",
                bus->name, slave->name, slave->node_id);
        }
        else
        {
            runtime->communication_fault = true;
        }
    }

    /* Small pause to allow slave to process transition to Pre-Operational before sending SDOs */
    usleep(30000U);

    /* Step 2: Configure SDOs while slave is in Pre-Operational state */
    if (!runtime->startup_sdo_sent)
    {
        canopen_send_configured_sdos(bus, runtime);
        runtime->startup_sdo_sent = true;
        plugin_logger_info(
            &g_logger, "Configured SDOs sent in Pre-Operational state: bus=%s recovery_count=%u",
            bus->name, runtime->retry_count + 1);
    }

    /* Step 3: Send NMT Start Remote Node (0x01) to enter Operational state */
    uint8_t started = 0U;
    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        pthread_mutex_lock(&runtime->stack_mutex);
        CO_ReturnError_t err =
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, slave->node_id);
        pthread_mutex_unlock(&runtime->stack_mutex);
        if (err != CO_ERROR_NO)
        {
            runtime->communication_fault = true;
            plugin_logger_warn(&g_logger, "NMT Start failed for bus=%s slave=%s node_id=%u err=%d",
                               bus->name, slave->name, slave->node_id, err);
            continue;
        }

        started++;
        runtime->communication_fault = false;
        plugin_logger_info(&g_logger, "NMT Start sent: bus=%s slave=%s node_id=%u command=0x01",
                           bus->name, slave->name, slave->node_id);
    }

    runtime->last_start_ms      = now_ms;
    runtime->reconnect_required = false;
    runtime->retry_count++;
    runtime->startup_reset_sent = true;

    if (started == 0U)
    {
        plugin_logger_warn(&g_logger, "No configured CANopen slave nodes were started for bus=%s",
                           bus->name);
    }
}

static void *canopen_linux_runtime_worker_proc(void *arg)
{
    (void)arg;
    plugin_logger_info(&g_logger, "CANopen Linux runtime thread started");

    while (g_canopen_rx_running)
    {
        bool any_bus_ready = false;
        for (int i = 0; i < g_runtime_bus_count; i++)
        {
            canopen_runtime_bus_t *runtime = &g_runtime_buses[i];
            if (!runtime->initialized || !runtime->epoll_ready || runtime->co == NULL)
            {
                continue;
            }

            any_bus_ready = true;

            CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
            pthread_mutex_lock(&runtime->stack_mutex);
            CO_epoll_wait(&runtime->epoll);
            CO_epoll_processMain(&runtime->epoll, runtime->co, true, &reset);
            CO_epoll_processRT(&runtime->epoll, runtime->co, false);
            CO_epoll_processLast(&runtime->epoll);

            canopen_runtime_lifecycle_tick(&g_config.buses[i], runtime);
            pthread_mutex_unlock(&runtime->stack_mutex);

            if (runtime->reconnect_required || !runtime->startup_confirmed)
            {
                canopen_start_configured_slaves(&g_config.buses[i], runtime->co, runtime);
            }
        }

        if (!any_bus_ready)
        {
            usleep(100000U);
        }
    }

    plugin_logger_info(&g_logger, "CANopen Linux runtime thread stopped");
    return NULL;
}

static uint16_t get_bus_bitrate_kbps(const canopen_bus_config_t *bus)
{
    if (bus == NULL || bus->bitrate == 0U)
    {
        return 125U;
    }
    return (uint16_t)(bus->bitrate / 1000U);
}

static bool canopen_prepare_input_binding(const canopen_pdo_mapping_t *mapping, int buffer_size,
                                          canopen_input_binding_t *binding)
{
    if (mapping == NULL || binding == NULL || !mapping->bound || mapping->plc_address[0] == '\0' ||
        mapping->direction[0] == '\0' || strcasecmp(mapping->direction, "input") != 0)
    {
        return false;
    }

    char prefix[4] = {0};
    int iec_index  = 0;
    int iec_bit    = 0;
    if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                   &iec_bit) ||
        iec_index < 0 || iec_index >= buffer_size || iec_bit < 0)
    {
        return false;
    }

    char address[64] = {0};
    if (sscanf(mapping->plc_address, "%63s", address) != 1 ||
        (strncasecmp(address, "%IX", 3) != 0 && strncasecmp(address, "%IB", 3) != 0 &&
         strncasecmp(address, "%IW", 3) != 0 && strncasecmp(address, "%ID", 3) != 0 &&
         strncasecmp(address, "%IL", 3) != 0))
    {
        return false;
    }

    uint16_t default_length = 0U;
    uint8_t plc_type        = 0U;
    if (strncasecmp(address, "%IX", 3) == 0)
    {
        if (iec_bit >= 8)
        {
            return false;
        }
        default_length = 1U;
        plc_type       = 0U;
    }
    else if (strncasecmp(address, "%IB", 3) == 0)
    {
        default_length = 8U;
        plc_type       = 3U;
    }
    else if (strncasecmp(address, "%IW", 3) == 0)
    {
        default_length = 16U;
        plc_type       = 5U;
    }
    else if (strncasecmp(address, "%ID", 3) == 0)
    {
        default_length = 32U;
        plc_type       = 8U;
    }
    else
    {
        default_length = 64U;
        plc_type       = 11U;
    }

    canopen_data_type_t data_type = canopen_parse_data_type(mapping->data_type);
    uint16_t bit_length           = canopen_data_type_bit_length(data_type);
    if (bit_length == 0U || bit_length > default_length ||
        (bit_length > 1U && (bit_length & 0x07U) != 0U))
    {
        return false;
    }

    binding->valid      = true;
    binding->bit_offset = 0U;
    binding->bit_length = bit_length;
    binding->plc_index  = (uint16_t)iec_index;
    binding->plc_bit    = (uint8_t)iec_bit;
    binding->plc_type   = plc_type;
    binding->data_type  = data_type;
    return true;
}

static uint64_t canopen_read_rpdo_bits(const uint8_t *payload, uint16_t bit_offset,
                                       uint16_t bit_length)
{
    if (payload == NULL || bit_length == 0U || bit_length > 64U)
    {
        return 0U;
    }

    uint64_t value = 0U;
    for (uint16_t bit = 0U; bit < bit_length; bit++)
    {
        uint16_t source_bit = (uint16_t)(bit_offset + bit);
        if ((payload[source_bit >> 3U] & (uint8_t)(1U << (source_bit & 0x07U))) != 0U)
        {
            value |= (uint64_t)1U << bit;
        }
    }
    return value;
}

static void canopen_rpdo_signal_pre(void *object)
{
    const canopen_rpdo_callback_context_t *context =
        (const canopen_rpdo_callback_context_t *)object;
    if (context == NULL || context->runtime == NULL || context->runtime->co == NULL ||
        context->rpdo_slot >= CANOPEN_LOCAL_RPDO_MAX ||
        context->runtime->input_binding_count[context->rpdo_slot] == 0U)
    {
        return;
    }

    CO_RPDO_t *rpdo        = &context->runtime->co->RPDO[context->rpdo_slot];
    const uint8_t *payload = rpdo->CANrxData[0];
    uint8_t binding_count  = context->runtime->input_binding_count[context->rpdo_slot];
    for (uint8_t i = 0U; i < binding_count; i++)
    {
        const canopen_input_binding_t *binding =
            &context->runtime->input_bindings[context->rpdo_slot][i];
        if (!binding->valid)
        {
            continue;
        }

        uint64_t value = canopen_normalize_value(
            canopen_read_rpdo_bits(payload, binding->bit_offset, binding->bit_length),
            binding->data_type);
        switch (binding->plc_type)
        {
        case 0U:
            if (g_args.journal_write_bool != NULL)
            {
                g_args.journal_write_bool(0, binding->plc_index, binding->plc_bit,
                                          value != 0U ? 1 : 0);
            }
            break;
        case 3U:
            if (g_args.journal_write_byte != NULL)
            {
                g_args.journal_write_byte(3, binding->plc_index, (int)(uint8_t)value);
            }
            break;
        case 5U:
            if (g_args.journal_write_int != NULL)
            {
                g_args.journal_write_int(5, binding->plc_index, (int)(uint16_t)value);
            }
            break;
        case 8U:
            if (g_args.journal_write_dint != NULL)
            {
                g_args.journal_write_dint(8, binding->plc_index, (unsigned int)(uint32_t)value);
            }
            break;
        case 11U:
            if (g_args.journal_write_lint != NULL)
            {
                g_args.journal_write_lint(11, binding->plc_index, (unsigned long long)value);
            }
            break;
        default:
            break;
        }
    }
}

static void canopen_configure_local_rpdos(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL || OD == NULL)
    {
        return;
    }

    memset(runtime->input_bindings, 0, sizeof(runtime->input_bindings));
    memset(runtime->input_binding_count, 0, sizeof(runtime->input_binding_count));
    memset(runtime->input_rpdo_node_id, 0, sizeof(runtime->input_rpdo_node_id));

    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        for (int p = 0; p < slave->rpdo_count; p++)
        {
            const canopen_pdo_t *pdo = &slave->rpdo[p];
            if (pdo->index < 0x1800U || pdo->index >= (0x1800U + CANOPEN_LOCAL_RPDO_MAX) ||
                pdo->mapping_count <= 0)
            {
                continue;
            }

            uint8_t slot = (uint8_t)(pdo->index - 0x1800U);
            if (runtime->input_binding_count[slot] != 0U)
            {
                plugin_logger_warn(&g_logger,
                                   "CANopen local RPDO slot collision: slot=%u old_node=%u "
                                   "new_node=%u; replacing binding",
                                   slot, runtime->input_rpdo_node_id[slot], slave->node_id);
            }

            runtime->input_binding_count[slot] = 0U;
            runtime->input_rpdo_node_id[slot]  = (uint8_t)slave->node_id;
            uint16_t bit_offset                = 0U;
            uint8_t map_count =
                (uint8_t)canopen_min_int(pdo->mapping_count, CANOPEN_LOCAL_RPDO_MAX_MAPPINGS);
            for (uint8_t m = 0U; m < map_count; m++)
            {
                const canopen_pdo_mapping_t *mapping = &pdo->mapping[m];
                if (mapping->bit_length == 0U || mapping->bit_length > 64U ||
                    (mapping->bit_length & 0x07U) != 0U ||
                    (uint32_t)bit_offset + mapping->bit_length > 8U * 8U)
                {
                    plugin_logger_warn(&g_logger,
                                       "Skipping invalid local RPDO mapping: bus=%s slave=%s "
                                       "pdo=%s map=%s bits=%u",
                                       bus->name, slave->name, pdo->name, mapping->name,
                                       mapping->bit_length);
                    continue;
                }

                uint32_t local_map = (uint32_t)mapping->bit_length;
                (void)OD_set_u32(OD_find(OD, (uint16_t)(0x1600U + slot)), (uint8_t)(m + 1U),
                                 local_map, true);

                canopen_input_binding_t *binding =
                    &runtime->input_bindings[slot][runtime->input_binding_count[slot]];
                if (canopen_prepare_input_binding(mapping, g_args.buffer_size, binding))
                {
                    binding->bit_offset = bit_offset;
                    runtime->input_binding_count[slot]++;
                }
                bit_offset = (uint16_t)(bit_offset + mapping->bit_length);
            }

            (void)OD_set_u8(OD_find(OD, (uint16_t)(0x1600U + slot)), 0U, map_count, true);
            uint32_t cob_id = (uint32_t)(0x180U + ((uint32_t)slot * 0x100U) + slave->node_id);
            (void)OD_set_u32(OD_find(OD, (uint16_t)(0x1400U + slot)), 1U, cob_id, true);
            (void)OD_set_u8(OD_find(OD, (uint16_t)(0x1400U + slot)), 2U, 0xFEU, true);
            (void)OD_set_u16(OD_find(OD, (uint16_t)(0x1400U + slot)), 5U, 0U, true);

            plugin_logger_info(&g_logger,
                               "CANopen local RPDO configured: bus=%s slave=%s slot=%u "
                               "source_tpdo=0x%04X cob_id=0x%03X mappings=%u inputs=%u",
                               bus->name, slave->name, slot, pdo->index, (unsigned)cob_id,
                               map_count, runtime->input_binding_count[slot]);
        }
    }
}

static bool canopen_prepare_output_binding(const canopen_pdo_mapping_t *mapping, int buffer_size,
                                           canopen_output_field_t *binding)
{
    if (mapping == NULL || binding == NULL || !mapping->bound || mapping->plc_address[0] == '\0' ||
        mapping->direction[0] == '\0' || strcasecmp(mapping->direction, "output") != 0)
    {
        return false;
    }

    char prefix[4] = {0};
    int iec_index  = 0;
    int iec_bit    = 0;
    if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                   &iec_bit) ||
        iec_index < 0 || iec_index >= buffer_size || iec_bit < 0)
    {
        return false;
    }

    char address[64] = {0};
    if (sscanf(mapping->plc_address, "%63s", address) != 1 ||
        (strncasecmp(address, "%QX", 3) != 0 && strncasecmp(address, "%QB", 3) != 0 &&
         strncasecmp(address, "%QW", 3) != 0 && strncasecmp(address, "%QD", 3) != 0 &&
         strncasecmp(address, "%QL", 3) != 0))
    {
        return false;
    }

    uint16_t default_length = 0U;
    uint8_t plc_type        = 0U;
    if (strncasecmp(address, "%QX", 3) == 0)
    {
        if (iec_bit >= 8)
        {
            return false;
        }
        default_length = 1U;
        plc_type       = 0U;
    }
    else if (strncasecmp(address, "%QB", 3) == 0)
    {
        default_length = 8U;
        plc_type       = 3U;
    }
    else if (strncasecmp(address, "%QW", 3) == 0)
    {
        default_length = 16U;
        plc_type       = 5U;
    }
    else if (strncasecmp(address, "%QD", 3) == 0)
    {
        default_length = 32U;
        plc_type       = 8U;
    }
    else
    {
        default_length = 64U;
        plc_type       = 11U;
    }

    canopen_data_type_t data_type = canopen_parse_data_type(mapping->data_type);
    uint16_t bit_length           = canopen_data_type_bit_length(data_type);
    if (bit_length == 0U || bit_length > default_length ||
        (bit_length > 1U && (bit_length & 0x07U) != 0U))
    {
        return false;
    }

    binding->valid      = true;
    binding->bit_offset = 0U;
    binding->bit_length = bit_length;
    binding->plc_index  = (uint16_t)iec_index;
    binding->plc_bit    = (uint8_t)iec_bit;
    binding->plc_type   = plc_type;
    binding->data_type  = data_type;
    return true;
}

static void canopen_write_tpdo_bits(uint8_t *payload, uint16_t bit_offset, uint16_t bit_length,
                                    uint64_t value)
{
    if (payload == NULL || bit_length == 0U || bit_length > 64U)
    {
        return;
    }

    for (uint16_t bit = 0U; bit < bit_length; bit++)
    {
        uint16_t target_bit = (uint16_t)(bit_offset + bit);
        if ((value & ((uint64_t)1U << bit)) != 0U)
        {
            payload[target_bit >> 3U] |= (uint8_t)(1U << (target_bit & 0x07U));
        }
        else
        {
            payload[target_bit >> 3U] &= (uint8_t)~(1U << (target_bit & 0x07U));
        }
    }
}

static void canopen_configure_local_tpdos(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL)
    {
        return;
    }

    memset(runtime->output_tpdos, 0, sizeof(runtime->output_tpdos));
    runtime->output_tpdo_count = 0U;

    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        for (int p = 0; p < slave->tpdo_count; p++)
        {
            const canopen_pdo_t *pdo = &slave->tpdo[p];
            if (pdo->mapping_count <= 0 || runtime->output_tpdo_count >= CANOPEN_LOCAL_TPDO_MAX)
            {
                continue;
            }

            uint16_t comm_index = pdo->index;
            uint32_t cob_id =
                (uint32_t)(0x200U + ((uint32_t)(comm_index - 0x1400U) * 0x100U) + slave->node_id);

            canopen_output_tpdo_t *tpdo = &runtime->output_tpdos[runtime->output_tpdo_count];
            tpdo->valid                 = true;
            tpdo->cob_id                = cob_id;
            tpdo->field_count           = 0U;
            uint16_t bit_offset         = 0U;

            uint8_t map_count =
                (uint8_t)canopen_min_int(pdo->mapping_count, CANOPEN_LOCAL_TPDO_MAX_MAPPINGS);
            for (uint8_t m = 0U; m < map_count; m++)
            {
                const canopen_pdo_mapping_t *mapping = &pdo->mapping[m];
                canopen_output_field_t *field        = &tpdo->fields[tpdo->field_count];
                if (canopen_prepare_output_binding(mapping, g_args.buffer_size, field))
                {
                    field->bit_offset = bit_offset;
                    tpdo->field_count++;
                }
                bit_offset = (uint16_t)(bit_offset + mapping->bit_length);
            }

            tpdo->dlc = (uint8_t)canopen_min_int((bit_offset + 7U) / 8U, 8);
            if (tpdo->dlc == 0U)
            {
                tpdo->dlc = 8U;
            }

            plugin_logger_info(&g_logger,
                               "CANopen local TPDO output configured: bus=%s slave=%s pdo=%s "
                               "cob_id=0x%03X dlc=%u outputs=%u",
                               bus->name, slave->name, pdo->name, (unsigned)tpdo->cob_id, tpdo->dlc,
                               tpdo->field_count);

            runtime->output_tpdo_count++;
        }
    }
}

static void sync_plc_image_to_canopen_bus(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL || runtime->fd < 0 || runtime->output_tpdo_count == 0U ||
        g_args.buffer_size <= 0)
    {
        return;
    }

    uint8_t payloads[CANOPEN_LOCAL_TPDO_MAX][8];
    memset(payloads, 0, sizeof(payloads));

    if (g_args.image_lock)
    {
        g_args.image_lock();
    }

    for (uint8_t p = 0U; p < runtime->output_tpdo_count; p++)
    {
        const canopen_output_tpdo_t *tpdo = &runtime->output_tpdos[p];
        if (!tpdo->valid || tpdo->field_count == 0U)
        {
            continue;
        }

        for (uint8_t f = 0U; f < tpdo->field_count; f++)
        {
            const canopen_output_field_t *field = &tpdo->fields[f];
            if (!field->valid)
            {
                continue;
            }

            uint64_t value = 0U;
            switch (field->plc_type)
            {
            case 0U: // BOOL
                if (g_args.bool_output != NULL && g_args.bool_output[field->plc_index] != NULL &&
                    g_args.bool_output[field->plc_index][field->plc_bit] != NULL)
                {
                    value = *g_args.bool_output[field->plc_index][field->plc_bit] ? 1U : 0U;
                }
                break;
            case 3U: // BYTE
                if (g_args.byte_output != NULL && g_args.byte_output[field->plc_index] != NULL)
                {
                    value = (uint64_t)*g_args.byte_output[field->plc_index];
                }
                break;
            case 5U: // INT/UINT
                if (g_args.int_output != NULL && g_args.int_output[field->plc_index] != NULL)
                {
                    value = (uint64_t)*g_args.int_output[field->plc_index];
                }
                break;
            case 8U: // DINT/UDINT
                if (g_args.dint_output != NULL && g_args.dint_output[field->plc_index] != NULL)
                {
                    value = (uint64_t)*g_args.dint_output[field->plc_index];
                }
                break;
            case 11U: // LINT/ULINT
                if (g_args.lint_output != NULL && g_args.lint_output[field->plc_index] != NULL)
                {
                    value = (uint64_t)*g_args.lint_output[field->plc_index];
                }
                break;
            default:
                break;
            }

            canopen_write_tpdo_bits(payloads[p], field->bit_offset, field->bit_length, value);
        }
    }

    if (g_args.image_unlock)
    {
        g_args.image_unlock();
    }

    for (uint8_t p = 0U; p < runtime->output_tpdo_count; p++)
    {
        const canopen_output_tpdo_t *tpdo = &runtime->output_tpdos[p];
        if (!tpdo->valid || tpdo->field_count == 0U)
        {
            continue;
        }

        can_socket_write(runtime->fd, tpdo->cob_id, false, false, tpdo->dlc, payloads[p]);
    }
}

static void apply_od_pdo_defaults(const canopen_bus_config_t *bus, canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL)
    {
        return;
    }

    plugin_logger_info(
        &g_logger, "apply_od_pdo_defaults: bus=%s slave_count=%d bus_tpdo=%d bus_rpdo=%d",
        bus->name, bus->slave_count, bus->tpdo_count, bus->rpdo_count);

    const uint16_t local_node_id = bus->local_node_id > 0U ? bus->local_node_id : 0x7FU;

    /* Runtime uses the generated OD.c defaults as the source of truth. */
    OD_PERSIST_COMM.x1018_identity.vendor_ID      = (uint32_t)local_node_id;
    OD_PERSIST_COMM.x1018_identity.productCode    = (uint32_t)bus->bitrate;
    OD_PERSIST_COMM.x1018_identity.revisionNumber = bus->heartbeat_ms;
    OD_PERSIST_COMM.x1018_identity.serialNumber   = bus->sync_period_ms;

    canopen_configure_local_rpdos(bus, runtime);
    canopen_configure_local_tpdos(bus, runtime);
}

static int canopen_parse_iec_address(const char *address, char *prefix, size_t prefix_len,
                                     int *index, int *bit)
{
    if (address == NULL || prefix == NULL || index == NULL)
    {
        return 0;
    }

    char work[64];
    if (sscanf(address, "%63s", work) != 1)
    {
        return 0;
    }

    if (strncasecmp(work, "%QX", 3) == 0 || strncasecmp(work, "%IX", 3) == 0 ||
        strncasecmp(work, "%QB", 3) == 0 || strncasecmp(work, "%IB", 3) == 0 ||
        strncasecmp(work, "%QW", 3) == 0 || strncasecmp(work, "%IW", 3) == 0 ||
        strncasecmp(work, "%QD", 3) == 0 || strncasecmp(work, "%ID", 3) == 0 ||
        strncasecmp(work, "%QL", 3) == 0 || strncasecmp(work, "%IL", 3) == 0)
    {
        snprintf(prefix, prefix_len, "%c%c", work[0], work[1]);
        char *suffix = work + 3;

        if (strncasecmp(work, "%QX", 3) == 0 || strncasecmp(work, "%IX", 3) == 0)
        {
            char *dot = strchr(suffix, '.');
            if (dot == NULL)
            {
                return 0;
            }
            *dot   = '\0';
            *index = atoi(suffix);
            *bit   = atoi(dot + 1);
            return 1;
        }

        if (strncasecmp(work, "%QB", 3) == 0 || strncasecmp(work, "%IB", 3) == 0)
        {
            *index = atoi(suffix);
            *bit   = 0;
            return 1;
        }

        *index = atoi(suffix);
        *bit   = 0;
        return 1;
    }

    return 0;
}

static int canopen_parse_status_address(const char *address, uint16_t *index)
{
    if (address == NULL || index == NULL)
    {
        return 0;
    }

    unsigned int parsed_index = 0U;
    char trailing             = '\0';
    if (sscanf(address, " %%%*[Ii]%*[Bb]%u %c", &parsed_index, &trailing) != 1 ||
        parsed_index > UINT16_MAX)
    {
        return 0;
    }

    *index = (uint16_t)parsed_index;
    return 1;
}

static void canopen_write_status(const char *address, bool ok)
{
    uint16_t index = 0U;
    if (!canopen_parse_status_address(address, &index) || g_args.journal_write_byte == NULL)
    {
        return;
    }

    g_args.journal_write_byte(3, index, ok ? 0 : 1);
}

static bool canopen_slave_status(const canopen_slave_config_t *slave,
                                 const canopen_runtime_bus_t *runtime)
{
    if (slave == NULL || runtime == NULL || runtime->co == NULL || !runtime->epoll_ready ||
        !slave->enabled)
    {
        return false;
    }

    if (strcmp(slave->protection_mode, "heartbeat_producer") == 0)
    {
        if (runtime->co->HBcons == NULL)
        {
            return false;
        }

        for (uint8_t i = 0U; i < runtime->co->HBcons->numberOfMonitoredNodes; i++)
        {
            const CO_HBconsNode_t *node = &runtime->co->HBconsMonitoredNodes[i];
            if (node->nodeId == slave->node_id)
            {
                return node->HBstate == CO_HBconsumer_ACTIVE &&
                       node->NMTstate == CO_NMT_OPERATIONAL;
            }
        }
        return false;
    }

#if ((CO_CONFIG_NODE_GUARDING) & CO_CONFIG_NODE_GUARDING_MASTER_ENABLE) != 0
    if (runtime->co->NGmaster != NULL)
    {
        for (uint8_t i = 0U; i < CO_CONFIG_NODE_GUARDING_MASTER_COUNT; i++)
        {
            const CO_nodeGuardingMasterNode_t *node = &runtime->co->NGmaster->nodes[i];
            if ((node->ident & 0x7FU) == slave->node_id)
            {
                return node->monitoringActive && node->NMTstate == CO_NMT_OPERATIONAL;
            }
        }
    }
#endif
    return false;
}

static void canopen_update_status_outputs(const canopen_bus_config_t *bus,
                                          canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL)
    {
        return;
    }

    const bool bus_ok = runtime->fd >= 0 && runtime->epoll_ready && !runtime->communication_fault;
    const bool master_ok = runtime->initialized && runtime->co != NULL && runtime->epoll_ready &&
                           !runtime->communication_fault;
    canopen_write_status(bus->bus_status_plc_address, bus_ok);
    canopen_write_status(bus->master_status_plc_address, master_ok);

    for (int i = 0; i < bus->slave_count; i++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[i];
        canopen_write_status(slave->status_plc_address, canopen_slave_status(slave, runtime));
    }
}

static int canopen_bus_plc_to_od(const canopen_bus_config_t *bus,
                                 const canopen_pdo_mapping_t *mapping)
{
    if (bus == NULL || mapping == NULL || !OD)
    {
        return 0;
    }

    if (!mapping->bound || mapping->plc_address[0] == '\0')
    {
        return 0;
    }

    if (mapping->direction[0] != '\0' && strcasecmp(mapping->direction, "output") != 0)
    {
        return 0;
    }

    OD_entry_t *od_entry = OD_find(OD, mapping->index);
    if (od_entry == NULL)
    {
        return 0;
    }

    const char *type = mapping->data_type[0] ? mapping->data_type : "u32";
    char prefix[4];
    int iec_index = 0;
    int iec_bit   = 0;

    if (strcasecmp(type, "bool") == 0)
    {
        if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                       &iec_bit))
        {
            return 0;
        }
        if (g_args.bool_output == NULL || g_args.bool_output[iec_index] == NULL ||
            g_args.bool_output[iec_index][iec_bit] == NULL)
        {
            return 0;
        }
        IEC_BOOL val = *g_args.bool_output[iec_index][iec_bit];
        OD_set_u8(od_entry, mapping->sub_index, val ? 1U : 0U, false);
        return 1;
    }
    if (strcasecmp(type, "u8") == 0 || strcasecmp(type, "byte") == 0)
    {
        if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                       &iec_bit))
        {
            return 0;
        }
        if (g_args.byte_output == NULL || g_args.byte_output[iec_index] == NULL)
        {
            return 0;
        }
        IEC_BYTE val = *g_args.byte_output[iec_index];
        OD_set_u8(od_entry, mapping->sub_index, (uint8_t)val, false);
        return 1;
    }
    if (strcasecmp(type, "i16") == 0 || strcasecmp(type, "u16") == 0)
    {
        if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                       &iec_bit))
        {
            return 0;
        }
        if (g_args.int_output == NULL || g_args.int_output[iec_index] == NULL)
        {
            return 0;
        }
        IEC_UINT val = *g_args.int_output[iec_index];
        if (strcasecmp(type, "i16") == 0)
        {
            OD_set_i16(od_entry, mapping->sub_index, (int16_t)val, false);
        }
        else
        {
            OD_set_u16(od_entry, mapping->sub_index, (uint16_t)val, false);
        }
        return 1;
    }
    if (strcasecmp(type, "i32") == 0 || strcasecmp(type, "u32") == 0)
    {
        if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                       &iec_bit))
        {
            return 0;
        }
        if (g_args.dint_output == NULL || g_args.dint_output[iec_index] == NULL)
        {
            return 0;
        }
        IEC_UDINT val = *g_args.dint_output[iec_index];
        if (strcasecmp(type, "i32") == 0)
        {
            OD_set_i32(od_entry, mapping->sub_index, (int32_t)val, false);
        }
        else
        {
            OD_set_u32(od_entry, mapping->sub_index, (uint32_t)val, false);
        }
        return 1;
    }
    if (strcasecmp(type, "i64") == 0 || strcasecmp(type, "u64") == 0)
    {
        if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                       &iec_bit))
        {
            return 0;
        }
        if (g_args.lint_output == NULL || g_args.lint_output[iec_index] == NULL)
        {
            return 0;
        }
        IEC_ULINT val = *g_args.lint_output[iec_index];
        if (strcasecmp(type, "i64") == 0)
        {
            OD_set_i64(od_entry, mapping->sub_index, (int64_t)val, false);
        }
        else
        {
            OD_set_u64(od_entry, mapping->sub_index, (uint64_t)val, false);
        }
        return 1;
    }
    if (strcasecmp(type, "f32") == 0)
    {
        if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                       &iec_bit))
        {
            return 0;
        }
        if (g_args.dint_output == NULL || g_args.dint_output[iec_index] == NULL)
        {
            return 0;
        }
        OD_set_f32(od_entry, mapping->sub_index, (float32_t)(float)(*g_args.dint_output[iec_index]),
                   false);
        return 1;
    }
    if (strcasecmp(type, "f64") == 0)
    {
        if (!canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                       &iec_bit))
        {
            return 0;
        }
        if (g_args.lint_output == NULL || g_args.lint_output[iec_index] == NULL)
        {
            return 0;
        }
        OD_set_f64(od_entry, mapping->sub_index,
                   (float64_t)(double)(*g_args.lint_output[iec_index]), false);
        return 1;
    }
    return 0;
}

static void sync_canopen_bus_to_plc_image(const canopen_bus_config_t *bus)
{
    if (bus == NULL || g_args.buffer_size <= 0)
    {
        return;
    }

    if (g_args.image_lock)
    {
        g_args.image_lock();
    }

    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled)
        {
            continue;
        }
        for (int i = 0; i < slave->tpdo_count; i++)
        {
            for (int j = 0; j < slave->tpdo[i].mapping_count; j++)
            {
                canopen_bus_plc_to_od(bus, &slave->tpdo[i].mapping[j]);
            }
        }
    }

    if (g_args.image_unlock)
    {
        g_args.image_unlock();
    }
}

static bool canopen_encode_sdo_payload(const canopen_sdo_entry_t *entry, uint8_t *payload,
                                       size_t payload_size, size_t *payload_len)
{
    if (entry == NULL || payload == NULL || payload_len == NULL)
    {
        return false;
    }

    const char *type = entry->data_type[0] != '\0' ? entry->data_type : "u32";
    int64_t raw      = (int64_t)entry->default_value;

    if (strcasecmp(type, "bool") == 0 || strcasecmp(type, "boolean") == 0)
    {
        if (payload_size < 1U)
        {
            return false;
        }
        payload[0]   = (uint8_t)(raw != 0 ? 1U : 0U);
        *payload_len = 1U;
        return true;
    }

    if (strcasecmp(type, "u8") == 0 || strcasecmp(type, "byte") == 0 ||
        strcasecmp(type, "uint8") == 0 || strcasecmp(type, "int8") == 0 ||
        strcasecmp(type, "sint8") == 0)
    {
        if (payload_size < 1U)
        {
            return false;
        }
        payload[0]   = (uint8_t)(raw & 0xFF);
        *payload_len = 1U;
        return true;
    }

    if (strcasecmp(type, "u16") == 0 || strcasecmp(type, "uint16") == 0 ||
        strcasecmp(type, "i16") == 0 || strcasecmp(type, "int16") == 0)
    {
        if (payload_size < 2U)
        {
            return false;
        }
        uint16_t value = (uint16_t)(raw & 0xFFFF);
        memcpy(payload, &value, sizeof(value));
        *payload_len = sizeof(value);
        return true;
    }

    if (strcasecmp(type, "u32") == 0 || strcasecmp(type, "uint32") == 0 ||
        strcasecmp(type, "i32") == 0 || strcasecmp(type, "int32") == 0)
    {
        if (payload_size < 4U)
        {
            return false;
        }
        uint32_t value = (uint32_t)raw;
        memcpy(payload, &value, sizeof(value));
        *payload_len = sizeof(value);
        return true;
    }

    if (strcasecmp(type, "u64") == 0 || strcasecmp(type, "uint64") == 0 ||
        strcasecmp(type, "i64") == 0 || strcasecmp(type, "int64") == 0)
    {
        if (payload_size < 8U)
        {
            return false;
        }
        uint64_t value = (uint64_t)raw;
        memcpy(payload, &value, sizeof(value));
        *payload_len = sizeof(value);
        return true;
    }

    return false;
}

static bool canopen_send_sdo_write(canopen_runtime_bus_t *runtime, uint8_t node_id,
                                   const canopen_sdo_entry_t *entry)
{
    if (runtime == NULL || runtime->co == NULL || runtime->co->SDOclient == NULL || entry == NULL)
    {
        return false;
    }

    CO_t *co = runtime->co;
    pthread_mutex_lock(&runtime->stack_mutex);
    uint8_t payload[8] = {0};
    size_t payload_len = 0U;
    if (!canopen_encode_sdo_payload(entry, payload, sizeof(payload), &payload_len))
    {
        pthread_mutex_unlock(&runtime->stack_mutex);
        plugin_logger_warn(&g_logger,
                           "Skipping unsupported SDO write: node=%u index=0x%04X sub=%u type=%s",
                           node_id, entry->index, entry->sub_index,
                           entry->data_type[0] != '\0' ? entry->data_type : "u32");
        return false;
    }

    CO_SDOclient_t *sdo_client = &co->SDOclient[0];
    CO_SDO_return_t ret = CO_SDOclient_setup(sdo_client, (uint32_t)CO_CAN_ID_SDO_CLI + node_id,
                                             (uint32_t)CO_CAN_ID_SDO_SRV + node_id, node_id);
    if (ret != CO_SDO_RT_ok_communicationEnd)
    {
        pthread_mutex_unlock(&runtime->stack_mutex);
        plugin_logger_warn(&g_logger, "SDO setup failed for node=%u index=0x%04X sub=%u ret=%d",
                           node_id, entry->index, entry->sub_index, ret);
        return false;
    }

    ret = CO_SDOclientDownloadInitiate(sdo_client, entry->index, entry->sub_index, payload_len,
                                       1000U, false);
    if (ret != CO_SDO_RT_ok_communicationEnd)
    {
        pthread_mutex_unlock(&runtime->stack_mutex);
        plugin_logger_warn(&g_logger, "SDO initiate failed for node=%u index=0x%04X sub=%u ret=%d",
                           node_id, entry->index, entry->sub_index, ret);
        return false;
    }

    size_t written                = CO_SDOclientDownloadBufWrite(sdo_client, payload, payload_len);
    bool partial                  = (written < payload_len);
    CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;

    do
    {
        CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
        if (runtime->epoll_ready)
        {
            CO_epoll_wait(&runtime->epoll);
            CO_epoll_processMain(&runtime->epoll, runtime->co, true, &reset);
            CO_epoll_processRT(&runtime->epoll, runtime->co, false);
            CO_epoll_processLast(&runtime->epoll);
        }

        uint32_t time_difference_us = runtime->epoll.timeDifference_us;
        if (time_difference_us == 0U)
        {
            time_difference_us = 10000U;
        }

        ret = CO_SDOclientDownload(sdo_client, time_difference_us, false, partial, &abort_code,
                                   NULL, NULL);
        if (ret < 0)
        {
            pthread_mutex_unlock(&runtime->stack_mutex);
            plugin_logger_warn(&g_logger,
                               "SDO download failed for node=%u index=0x%04X sub=%u abort=0x%08X",
                               node_id, entry->index, entry->sub_index, abort_code);
            return false;
        }
    } while (ret > 0);

    pthread_mutex_unlock(&runtime->stack_mutex);

    plugin_logger_info(&g_logger,
                       "SDO write sent once: bus local node=%u target_node=%u index=0x%04X sub=%u "
                       "len=%zu value=0x%08X",
                       co->NMT->nodeId, node_id, entry->index, entry->sub_index, payload_len,
                       entry->default_value);
    return true;
}

static void canopen_add_pdo_mapping_sdos(const canopen_slave_config_t *slave,
                                         canopen_sdo_entry_t *entries, int *entry_count)
{
    if (slave == NULL || entries == NULL || entry_count == NULL)
    {
        return;
    }

    int count = *entry_count;

    // Add RPDO entries
    for (int p = 0; p < slave->rpdo_count; p++)
    {
        const canopen_pdo_t *pdo = &slave->rpdo[p];
        if (pdo == NULL || pdo->index < 0x1800U ||
            pdo->index >= (0x1800U + CANOPEN_LOCAL_RPDO_MAX) || pdo->mapping_count <= 0)
        {
            continue;
        }

        uint16_t comm_index = pdo->index;                      // 0x1800 - 0x180*
        uint16_t map_index  = (uint16_t)(pdo->index + 0x200U); // 0x1A00 - 0x1A0*
        uint32_t cob_id =
            (uint32_t)(0x180U + ((uint32_t)(comm_index - 0x1800U) * 0x100U) + slave->node_id);

        snprintf(entries[count].name, sizeof(entries[count].name), "rpdo_%d_map_count", p + 1);
        entries[count].index        = map_index;
        entries[count].sub_index    = 0U;
        entries[count].data_type[0] = 'u';
        entries[count].data_type[1] = '8';
        entries[count].default_value =
            pdo->mapping_count; // the number of mapped application objects in the PDO
        count++;

        for (int m = 0; m < pdo->mapping_count; m++)
        {
            const canopen_pdo_mapping_t *mapping = &pdo->mapping[m];
            if (mapping == NULL)
            {
                continue;
            }

            snprintf(entries[count].name, sizeof(entries[count].name), "rpdo_%d_map_%d", p + 1,
                     m + 1);
            entries[count].index        = map_index;
            entries[count].sub_index    = (uint8_t)(m + 1U);
            entries[count].data_type[0] = 'u';
            entries[count].data_type[1] = '3';
            entries[count].data_type[2] = '2';
            entries[count].default_value =
                (int32_t)(((uint32_t)mapping->index << 16U) | ((uint32_t)mapping->sub_index << 8U) |
                          (uint32_t)mapping->bit_length); // the mapping value is a 32-bit value
                                                          // that encodes the index, sub-index, and
                                                          // bit length of the mapped object
            count++;
        }

        snprintf(entries[count].name, sizeof(entries[count].name), "rpdo_%d_cob_id", p + 1);
        entries[count].index         = comm_index;
        entries[count].sub_index     = 1U;
        entries[count].data_type[0]  = 'u';
        entries[count].data_type[1]  = '3';
        entries[count].data_type[2]  = '2';
        entries[count].default_value = (int32_t)cob_id; // the COB-ID used by the RPDO
        count++;

        snprintf(entries[count].name, sizeof(entries[count].name), "rpdo_%d_trans_type", p + 1);
        entries[count].index         = comm_index;
        entries[count].sub_index     = 2U;
        entries[count].data_type[0]  = 'u';
        entries[count].data_type[1]  = '8';
        entries[count].default_value = 0xFE; // the transmission type of the RPDO
        count++;

        snprintf(entries[count].name, sizeof(entries[count].name), "rpdo_%d_inhibit_time", p + 1);
        entries[count].index         = comm_index;
        entries[count].sub_index     = 3U;
        entries[count].data_type[0]  = 'u';
        entries[count].data_type[1]  = '1';
        entries[count].data_type[2]  = '6';
        entries[count].default_value = 500; // the inhibit time of the TPDO in milliseconds
        count++;

        snprintf(entries[count].name, sizeof(entries[count].name), "rpdo_%d_event_timer", p + 1);
        entries[count].index        = comm_index;
        entries[count].sub_index    = 5U;
        entries[count].data_type[0] = 'u';
        entries[count].data_type[1] = '1';
        entries[count].data_type[2] = '6';
        entries[count].default_value =
            0; // the event timer of the TPDO in milliseconds (0 means no event timer)
        count++;
    }

    for (int p = 0; p < slave->tpdo_count; p++)
    {
        const canopen_pdo_t *pdo = &slave->tpdo[p];
        if (pdo == NULL || pdo->index < 0x1400U ||
            pdo->index >= (0x1400U + CANOPEN_LOCAL_TPDO_MAX) || pdo->mapping_count <= 0)
        {
            continue;
        }

        uint16_t comm_index = pdo->index;                      // 0x1400 - 0x140*
        uint16_t map_index  = (uint16_t)(pdo->index + 0x200U); // 0x1600 - 0x160*
        uint32_t cob_id =
            (uint32_t)(0x200U + ((uint32_t)(comm_index - 0x1400U) * 0x100U) + slave->node_id);

        snprintf(entries[count].name, sizeof(entries[count].name), "tpdo_%d_map_count", p + 1);
        entries[count].index        = map_index;
        entries[count].sub_index    = 0U;
        entries[count].data_type[0] = 'u';
        entries[count].data_type[1] = '8';
        entries[count].default_value =
            pdo->mapping_count; // the number of mapped application objects in the PDO
        count++;

        for (int m = 0; m < pdo->mapping_count; m++)
        {
            const canopen_pdo_mapping_t *mapping = &pdo->mapping[m];
            if (mapping == NULL)
            {
                continue;
            }

            snprintf(entries[count].name, sizeof(entries[count].name), "tpdo_%d_map_%d", p + 1,
                     m + 1);
            entries[count].index        = map_index;
            entries[count].sub_index    = (uint8_t)(m + 1U);
            entries[count].data_type[0] = 'u';
            entries[count].data_type[1] = '3';
            entries[count].data_type[2] = '2';
            entries[count].default_value =
                (int32_t)(((uint32_t)mapping->index << 16U) | ((uint32_t)mapping->sub_index << 8U) |
                          (uint32_t)mapping->bit_length); // the mapping value is a 32-bit value
                                                          // that encodes the index, sub-index, and
                                                          // bit length of the mapped object
            count++;
        }

        snprintf(entries[count].name, sizeof(entries[count].name), "tpdo_%d_cob_id", p + 1);
        entries[count].index         = comm_index;
        entries[count].sub_index     = 1U;
        entries[count].data_type[0]  = 'u';
        entries[count].data_type[1]  = '3';
        entries[count].data_type[2]  = '2';
        entries[count].default_value = (int32_t)cob_id; // the COB-ID used by the TPDO
        count++;

        snprintf(entries[count].name, sizeof(entries[count].name), "tpdo_%d_trans_type", p + 1);
        entries[count].index         = comm_index;
        entries[count].sub_index     = 2U;
        entries[count].data_type[0]  = 'u';
        entries[count].data_type[1]  = '8';
        entries[count].default_value = 0xFE; // the transmission type of the TPDO
        count++;

        snprintf(entries[count].name, sizeof(entries[count].name), "tpdo_%d_inhibit_time", p + 1);
        entries[count].index         = comm_index;
        entries[count].sub_index     = 3U;
        entries[count].data_type[0]  = 'u';
        entries[count].data_type[1]  = '1';
        entries[count].data_type[2]  = '6';
        entries[count].default_value = 0; // the inhibit time of the TPDO in milliseconds
        count++;
    }

    *entry_count = count;
}

static void canopen_send_configured_sdos(const canopen_bus_config_t *bus,
                                         canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL || runtime->co == NULL)
    {
        return;
    }

    if (runtime->co->SDOclient == NULL)
    {
        plugin_logger_warn(&g_logger,
                           "SDO client unavailable on bus=%s; skipping configured SDO writes",
                           bus->name);
        return;
    }

    /* Give slaves a small pause after entering Pre-Operational state before issuing SDO writes */
    usleep(50000U);

    int sent_count = 0;
    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        canopen_sdo_entry_t protection_sdos[3];
        int protection_sdo_count = 0;
        memset(protection_sdos, 0, sizeof(protection_sdos));
        if (strcmp(slave->protection_mode, "heartbeat_producer") == 0)
        {
            snprintf(protection_sdos[0].name, sizeof(protection_sdos[0].name), "guard_time_off");
            protection_sdos[0].index         = 0x100CU;
            protection_sdos[0].data_type[0]  = 'u';
            protection_sdos[0].data_type[1]  = '1';
            protection_sdos[0].data_type[2]  = '6';
            protection_sdos[0].default_value = 0;

            snprintf(protection_sdos[1].name, sizeof(protection_sdos[1].name), "life_factor_off");
            protection_sdos[1].index         = 0x100DU;
            protection_sdos[1].data_type[0]  = 'u';
            protection_sdos[1].data_type[1]  = '8';
            protection_sdos[1].default_value = 0;

            snprintf(protection_sdos[2].name, sizeof(protection_sdos[2].name),
                     "heartbeat_producer_time");
            protection_sdos[2].index         = 0x1017U;
            protection_sdos[2].data_type[0]  = 'u';
            protection_sdos[2].data_type[1]  = '1';
            protection_sdos[2].data_type[2]  = '6';
            protection_sdos[2].default_value = (int32_t)slave->heartbeat_producer_time_ms;

            protection_sdo_count = 3;
        }
        else
        {
            snprintf(protection_sdos[0].name, sizeof(protection_sdos[0].name),
                     "heartbeat_producer_off");
            protection_sdos[0].index         = 0x1017U;
            protection_sdos[0].data_type[0]  = 'u';
            protection_sdos[0].data_type[1]  = '1';
            protection_sdos[0].data_type[2]  = '6';
            protection_sdos[0].default_value = 0;

            snprintf(protection_sdos[1].name, sizeof(protection_sdos[1].name), "guard_time");
            protection_sdos[1].index         = 0x100CU;
            protection_sdos[1].data_type[0]  = 'u';
            protection_sdos[1].data_type[1]  = '1';
            protection_sdos[1].data_type[2]  = '6';
            protection_sdos[1].default_value = (int32_t)slave->node_guard_time_ms;

            snprintf(protection_sdos[2].name, sizeof(protection_sdos[2].name), "life_time_factor");
            protection_sdos[2].index         = 0x100DU;
            protection_sdos[2].data_type[0]  = 'u';
            protection_sdos[2].data_type[1]  = '8';
            protection_sdos[2].default_value = slave->node_guard_life_factor;

            protection_sdo_count = 3;
        }

        for (int i = 0; i < protection_sdo_count; i++)
        {
            if (canopen_send_sdo_write(runtime, slave->node_id, &protection_sdos[i]))
            {
                sent_count++;
            }
        }

        canopen_sdo_entry_t generated_sdos[(CANOPEN_LOCAL_RPDO_MAX + CANOPEN_LOCAL_TPDO_MAX) *
                                           (CANOPEN_LOCAL_RPDO_MAX_MAPPINGS + 4U)];
        int generated_sdo_count = 0;
        memset(generated_sdos, 0, sizeof(generated_sdos));
        canopen_add_pdo_mapping_sdos(slave, generated_sdos, &generated_sdo_count);
        for (int i = 0; i < generated_sdo_count; i++)
        {
            if (canopen_send_sdo_write(runtime, slave->node_id, &generated_sdos[i]))
            {
                sent_count++;
            }
        }

        for (int i = 0; i < slave->sdo_count; i++)
        {
            const canopen_sdo_entry_t *entry = &slave->sdo[i];
            if (entry->index == 0U)
            {
                continue;
            }

            if ((strcmp(slave->protection_mode, "heartbeat_producer") == 0 &&
                 (entry->index == 0x100CU || entry->index == 0x100DU)) ||
                (strcmp(slave->protection_mode, "heartbeat_producer") != 0 &&
                 entry->index == 0x1017U))
            {
                continue;
            }

            if (canopen_send_sdo_write(runtime, slave->node_id, entry))
            {
                sent_count++;
            }
        }
    }

    plugin_logger_info(&g_logger, "Configured SDO writes completed for bus=%s sent=%d", bus->name,
                       sent_count);
}

static int init_runtime_bus(const canopen_bus_config_t *bus, int bus_index)
{
    CO_t *co = CO_new(NULL, NULL);
    if (co == NULL)
    {
        plugin_logger_error(&g_logger, "CO_new() failed for bus %d (%s)", bus_index, bus->name);
        return -1;
    }

    g_runtime_buses[bus_index].co                 = co;
    g_runtime_buses[bus_index].initialized        = true;
    g_runtime_buses[bus_index].epoll_ready        = false;
    g_runtime_buses[bus_index].startup_confirmed  = false;
    g_runtime_buses[bus_index].reconnect_required = false;
    g_runtime_buses[bus_index].bootup_seen        = false;
    g_runtime_buses[bus_index].startup_sdo_sent   = false;
    memset(&g_runtime_buses[bus_index].epoll, 0, sizeof(g_runtime_buses[bus_index].epoll));
    g_runtime_buses[bus_index].node_id =
        (uint8_t)((bus->local_node_id >= 1U && bus->local_node_id <= 127U) ? bus->local_node_id
                                                                           : 0xFFU);
    g_runtime_buses[bus_index].bitrate           = bus->bitrate;
    g_runtime_buses[bus_index].last_start_ms     = 0U;
    g_runtime_buses[bus_index].last_heartbeat_ms = 0U;
    g_runtime_buses[bus_index].last_bootup_ms    = 0U;
    g_runtime_buses[bus_index].retry_count       = 0U;
    g_runtime_buses[bus_index].fd                = -1;
    pthread_mutex_init(&g_runtime_buses[bus_index].stack_mutex, NULL);

    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        plugin_logger_info(
            &g_logger,
            "Bus[%d] slave[%d]: name=%s node_id=%u enabled=%d tpdo=%d rpdo=%d sdo=%d",
            bus_index, s, slave->name, slave->node_id, slave->enabled,
            slave->tpdo_count, slave->rpdo_count, slave->sdo_count);
        for (int i = 0; i < slave->tpdo_count; i++)
        {
            plugin_logger_info(&g_logger,
                               "Bus[%d] slave[%d] TPDO[%d]: name=%s index=0x%04X mappings=%d",
                               bus_index, s, i, slave->tpdo[i].name, slave->tpdo[i].index,
                               slave->tpdo[i].mapping_count);
        }
        for (int i = 0; i < slave->rpdo_count; i++)
        {
            plugin_logger_info(&g_logger,
                               "Bus[%d] slave[%d] RPDO[%d]: name=%s index=0x%04X mappings=%d",
                               bus_index, s, i, slave->rpdo[i].name, slave->rpdo[i].index,
                               slave->rpdo[i].mapping_count);
        }
    }

    apply_od_pdo_defaults(bus, &g_runtime_buses[bus_index]);
    canopen_apply_heartbeat_defaults(
        bus, co); // Apply heartbeat defaults to the CANopen stack based on the bus configuration

    plugin_logger_info(&g_logger,
                       "CANopen init sequence: step=interface_ready bus=%s if=%s bitrate=%u",
                       bus->name, bus->interface, bus->bitrate);
    if (bus->interface[0] != '\0')
    {
        can_hardware_config_t hw = {0};
        snprintf(hw.interface, sizeof(hw.interface), "%s", bus->interface);
        hw.bitrate         = bus->bitrate;
        hw.sjw             = bus->sjw;
        hw.sample_point    = bus->sample_point;
        hw.restart_ms      = bus->restart_ms;
        hw.triple_sampling = bus->triple_sampling;
        hw.auto_bringup    = bus->auto_bringup;

        if (can_netlink_configure_and_up(&hw, &g_logger) != 0)
        {
            plugin_logger_warn(&g_logger,
                               "CANopen init sequence: interface_ready failed for %s; continuing "
                               "but socket may remain down",
                               bus->interface);
        }
        else
        {
            plugin_logger_info(&g_logger,
                               "CANopen init sequence: interface_ready success bus=%s if=%s",
                               bus->name, bus->interface);
        }
    }

    uint32_t errInfo = 0U;
    uint16_t bitRate = get_bus_bitrate_kbps(bus);

    CO_CANptrSocketCan_t canptr = {0};
    canptr.can_ifindex          = (int)if_nametoindex(bus->interface);
    if (canptr.can_ifindex <= 0)
    {
        plugin_logger_error(&g_logger, "CANopen init sequence: if_nametoindex failed for %s",
                            bus->interface);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }

    plugin_logger_info(&g_logger, "CANopen init sequence: step=socket_open bus=%s if=%s ifindex=%d",
                       bus->name, bus->interface, canptr.can_ifindex);
    if (bus->interface[0] != '\0')
    {
        g_runtime_buses[bus_index].fd = can_socket_open(bus->interface, &g_logger);
        if (g_runtime_buses[bus_index].fd < 0)
        {
            plugin_logger_warn(
                &g_logger,
                "CANopen init sequence: socket_open failed for bus=%s on %s; keeping stack "
                "initialized but offline",
                bus->name, bus->interface);
        }
        else
        {
            plugin_logger_info(&g_logger,
                               "CANopen init sequence: socket_open success bus=%s if=%s fd=%d",
                               bus->name, bus->interface, g_runtime_buses[bus_index].fd);
        }
    }

    if (CO_epoll_create(&g_runtime_buses[bus_index].epoll, 1000U) != CO_ERROR_NO)
    {
        plugin_logger_error(&g_logger, "CANopen init sequence: CO_epoll_create() failed for %s",
                            bus->name);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }
    canptr.epoll_fd = g_runtime_buses[bus_index].epoll.epoll_fd;
    plugin_logger_info(&g_logger,
                       "CANopen init sequence: step=CO_CANinit bus=%s if=%s ifindex=%d epoll_fd=%d "
                       "bitrate_kbps=%u",
                       bus->name, bus->interface, canptr.can_ifindex, canptr.epoll_fd, bitRate);

    CO_ReturnError_t err = CO_CANinit(co, &canptr, bitRate);
    if (err != CO_ERROR_NO)
    {
        plugin_logger_error(
            &g_logger,
            "CANopen init sequence: CO_CANinit() failed for %s: %d (ifindex=%d epoll_fd=%d)",
            bus->name, err, canptr.can_ifindex, canptr.epoll_fd);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }
    plugin_logger_info(&g_logger,
                       "CANopen init sequence: step=CO_CANinit OK bus=%s if=%s local_node_id=%u",
                       bus->name, bus->interface, g_runtime_buses[bus_index].node_id);

    if (co->HBcons != NULL)
    {
        CO_HBconsumer_initCallbackNmtChanged(co->HBcons, 0U, &g_runtime_buses[bus_index],
                                             canopen_hb_state_changed);
    }

    plugin_logger_info(
        &g_logger,
        "CANopen init sequence: step=CO_CANopenInit bus=%s local_node_id=%u heartbeat_ms=%u",
        bus->name, g_runtime_buses[bus_index].node_id,
        (bus->heartbeat_ms > 0U) ? bus->heartbeat_ms : 500U);
    err = CO_CANopenInit(co, NULL, NULL, OD, NULL, CANOPEN_NMT_CONTROL,
                         (bus->heartbeat_ms > 0U) ? (uint16_t)bus->heartbeat_ms : 500U, 1000U, 500U,
                         false, g_runtime_buses[bus_index].node_id, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
    {
        plugin_logger_error(
            &g_logger, "CANopen init sequence: CO_CANopenInit() failed for %s: %d (errInfo=0x%X)",
            bus->name, err, errInfo);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }
    plugin_logger_info(&g_logger,
                       "CANopen init sequence: step=CO_CANopenInit OK bus=%s local_node_id=%u",
                       bus->name, g_runtime_buses[bus_index].node_id);

    plugin_logger_info(&g_logger,
                       "CANopen init sequence: step=PDO_init start bus=%s local_node_id=%u "
                       "OD_CNT_TPDO=%d OD_CNT_RPDO=%d slave_count=%d",
                       bus->name, g_runtime_buses[bus_index].node_id, OD_CNT_TPDO, OD_CNT_RPDO,
                       bus->slave_count);
    err = CO_CANopenInitPDO(co, co->em, OD, g_runtime_buses[bus_index].node_id, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
    {
        plugin_logger_error(
            &g_logger,
            "CANopen init sequence: CO_CANopenInitPDO() failed for %s: %d (errInfo=0x%X)",
            bus->name, err, errInfo);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }
    plugin_logger_info(
        &g_logger,
        "CANopen init sequence: step=PDO_init OK bus=%s node_id=%u tpdo_count=%d rpdo_count=%d",
        bus->name, g_runtime_buses[bus_index].node_id, bus->tpdo_count, bus->rpdo_count);

    for (uint8_t slot = 0U; slot < CANOPEN_LOCAL_RPDO_MAX; slot++)
    {
        if (g_runtime_buses[bus_index].input_binding_count[slot] == 0U)
        {
            continue;
        }
        g_runtime_buses[bus_index].rpdo_callback_context[slot].runtime =
            &g_runtime_buses[bus_index];
        g_runtime_buses[bus_index].rpdo_callback_context[slot].rpdo_slot = slot;
        CO_RPDO_initCallbackPre(&co->RPDO[slot],
                                &g_runtime_buses[bus_index].rpdo_callback_context[slot],
                                canopen_rpdo_signal_pre);
        plugin_logger_info(&g_logger,
                           "CANopen local RPDO input callback bound: bus=%s slot=%u cob_id=0x%03X "
                           "node_id=%u mappings=%u",
                           bus->name, slot,
                           (unsigned)(0x180U + ((uint32_t)slot * 0x100U) +
                                      g_runtime_buses[bus_index].input_rpdo_node_id[slot]),
                           g_runtime_buses[bus_index].input_rpdo_node_id[slot],
                           g_runtime_buses[bus_index].input_binding_count[slot]);
    }

    CO_CANsetNormalMode(co->CANmodule);
    plugin_logger_info(&g_logger, "CANopen init sequence: step=CAN_normal_mode bus=%s if=%s fd=%d",
                       bus->name, bus->interface, g_runtime_buses[bus_index].fd);

    if (!canopen_configure_node_guarding(bus, co))
    {
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }

    if (g_runtime_buses[bus_index].fd >= 0)
    {
        /* Use the CANopenLinux mainloop pattern: one epoll instance, initialized once,
         * attached to the stack before normal mode is enabled. This keeps the OpenPLC
         * plugin as a thin adapter rather than maintaining a second runtime loop. */
        CO_epoll_initCANopenMain(&g_runtime_buses[bus_index].epoll, co);
        g_runtime_buses[bus_index].epoll_ready = true;
        plugin_logger_info(&g_logger, "CANopen Linux epoll runtime enabled for bus=%s interface=%s",
                           bus->name, bus->interface);

        /* Start the slave state machine first; only then send the startup SDO writes.
         * Sending SDOs before the remote node is actually started may cause no response and
         * abort 0x05040000 (SDO protocol timed out). */
        canopen_start_configured_slaves(bus, co, &g_runtime_buses[bus_index]);
    }
    else
    {
        plugin_logger_warn(
            &g_logger, "CANopen Linux runtime not started for bus=%s because socket is not ready",
            bus->name);
    }

    plugin_logger_info(&g_logger,
                       "CANopen runtime bound: bus=%s interface=%s local_node_id=%u bitrate=%u "
                         "tpdo=%d rpdo=%d socket_fd=%d",
                       bus->name, bus->interface, bus->local_node_id, bus->bitrate,
                         bus->tpdo_count, bus->rpdo_count,
                       g_runtime_buses[bus_index].fd);
    return 0;
}

int init(void *args)
{
    if (!args)
        return -1;
    memcpy(&g_args, args, sizeof(plugin_runtime_args_t));

    if (!plugin_logger_init(&g_logger, "CANOPEN_PLUGIN", &g_args))
    {
        return -1;
    }

    reset_runtime_state();
    plugin_logger_info(&g_logger, "Initializing CANopen runtime adapter");

    const char *cfg_path = g_args.plugin_specific_config_file_path;
    if (!cfg_path || !cfg_path[0])
    {
        plugin_logger_info(&g_logger,
                           "No CANopen config file path configured; plugin remains idle");
        canopen_config_init_defaults(&g_config);
        return 0;
    }

    if (access(cfg_path, F_OK) != 0)
    {
        plugin_logger_warn(&g_logger, "CANopen config file not found: %s", cfg_path);
        canopen_config_init_defaults(&g_config);
        return 0;
    }

    int rc = canopen_config_parse(cfg_path, &g_config, &g_logger);
    if (rc != 0)
    {
        plugin_logger_error(&g_logger, "Failed to parse CANopen configuration");
        return -1;
    }

    plugin_logger_info(&g_logger, "CANopen runtime adapter initialized with %d bus(es)",
                       g_config.bus_count);
    return 0;
}

int start_loop(void)
{
    plugin_logger_info(&g_logger, "Starting CANopen runtime adapter");
    if (g_config.bus_count == 0)
    {
        plugin_logger_warn(&g_logger, "No active CANopen bus configured");
        return 0;
    }

    reset_runtime_state();
    g_runtime_bus_count = 0;
    for (int i = 0; i < g_config.bus_count; i++)
    {
        const canopen_bus_config_t *bus = &g_config.buses[i];
        if (bus->enabled && init_runtime_bus(bus, i) == 0)
        {
            g_runtime_bus_count++;
        }
        plugin_logger_info(&g_logger,
                           "Bus[%d]: name=%s interface=%s local_node_id=%u bitrate=%u "
                            "tpdo=%d rpdo=%d",
                           i, bus->name, bus->interface, bus->local_node_id, bus->bitrate,
                            bus->tpdo_count, bus->rpdo_count);
    }

    if (g_runtime_bus_count > 0)
    {
        g_canopen_rx_running = true;
        if (pthread_create(&g_canopen_rx_thread, NULL, canopen_linux_runtime_worker_proc, NULL) !=
            0)
        {
            plugin_logger_error(&g_logger, "Failed to create CANopen Linux runtime thread");
            g_canopen_rx_running = false;
        }
    }
    return 0;
}

void cycle_start(void) {}

void cycle_end(void)
{
    for (int i = 0; i < g_config.bus_count; i++)
    {
        const canopen_bus_config_t *bus = &g_config.buses[i];
        if (!bus->enabled || i >= MAX_CANOPEN_BUSES)
        {
            continue;
        }

        if (g_runtime_buses[i].initialized && g_runtime_buses[i].co != NULL)
        {
            /* Exchange PDO process image when slave node(s) are confirmed in Operational state */
            if (g_runtime_buses[i].startup_confirmed)
            {
                sync_plc_image_to_canopen_bus(bus, &g_runtime_buses[i]);
                sync_canopen_bus_to_plc_image(bus);
            }
        }

        canopen_update_status_outputs(bus, &g_runtime_buses[i]);
    }
}

void stop_loop(void)
{
    plugin_logger_info(&g_logger, "Stopping CANopen runtime adapter");
    g_canopen_rx_running = false;
    if (pthread_join(g_canopen_rx_thread, NULL) != 0 && g_runtime_bus_count > 0)
    {
        plugin_logger_warn(&g_logger, "CANopen Linux runtime thread not joined cleanly");
    }

    for (int i = 0; i < MAX_CANOPEN_BUSES; i++)
    {
        if (g_runtime_buses[i].epoll_ready)
        {
            CO_epoll_close(&g_runtime_buses[i].epoll);
            g_runtime_buses[i].epoll_ready = false;
        }
        if (g_runtime_buses[i].fd >= 0)
        {
            can_socket_close(g_runtime_buses[i].fd);
            g_runtime_buses[i].fd = -1;
        }
        if (g_runtime_buses[i].initialized)
        {
            pthread_mutex_destroy(&g_runtime_buses[i].stack_mutex);
        }
    }
}

void cleanup(void)
{
    plugin_logger_info(&g_logger, "Cleaning up CANopen runtime adapter");
    for (int i = 0; i < MAX_CANOPEN_BUSES; i++)
    {
        if (g_runtime_buses[i].co != NULL)
        {
            CO_delete(g_runtime_buses[i].co);
            g_runtime_buses[i].co          = NULL;
            g_runtime_buses[i].initialized = false;
        }
        if (g_runtime_buses[i].fd >= 0)
        {
            can_socket_close(g_runtime_buses[i].fd);
            g_runtime_buses[i].fd = -1;
        }
    }
    reset_runtime_state();
    canopen_config_free(&g_config);
}

int execute_command(const char *command_json, char *response, size_t response_size)
{
    if (!response || response_size == 0)
        return -1;
    response[0] = '\0';

    if (!command_json || !command_json[0])
    {
        snprintf(response, response_size, "{\"status\":\"empty_command\"}");
        return 0;
    }

    cJSON *root = cJSON_Parse(command_json);
    if (root == NULL)
    {
        snprintf(response, response_size, "{\"status\":\"invalid_json\"}");
        return 0;
    }

    const char *op =
        cJSON_GetObjectItem(root, "op") ? cJSON_GetObjectItem(root, "op")->valuestring : NULL;
    const char *bus_name =
        cJSON_GetObjectItem(root, "bus") ? cJSON_GetObjectItem(root, "bus")->valuestring : NULL;
    int bus_index = -1;
    for (int i = 0; i < g_runtime_bus_count; i++)
    {
        if (bus_name == NULL || strcmp(g_config.buses[i].name, bus_name) == 0)
        {
            bus_index = i;
            break;
        }
    }

    if (bus_index < 0)
    {
        bus_index = 0;
    }

    if (op != NULL && strcmp(op, "sdo_read") == 0)
    {
        uint16_t index    = 0U;
        uint8_t sub_index = 0U;
        uint8_t node_id   = 0U;

        cJSON *idx = cJSON_GetObjectItem(root, "index");
        if (idx != NULL && idx->valuestring)
        {
            index = (uint16_t)strtoul(idx->valuestring, NULL, 0);
        }
        else if (idx != NULL && cJSON_IsNumber(idx))
        {
            index = (uint16_t)idx->valueint;
        }

        cJSON *sub = cJSON_GetObjectItem(root, "sub_index");
        if (sub != NULL)
        {
            sub_index =
                (uint8_t)(cJSON_IsNumber(sub) ? sub->valueint
                                              : (uint8_t)strtoul(sub->valuestring, NULL, 0));
        }

        cJSON *node = cJSON_GetObjectItem(root, "node_id");
        if (node == NULL || (!cJSON_IsNumber(node) && !cJSON_IsString(node)))
        {
            snprintf(
                response, response_size,
                "{\"status\":\"sdo_read_failed\",\"reason\":\"missing_node_id\",\"bus\":\"%s\"}",
                g_config.buses[bus_index].name);
            cJSON_Delete(root);
            return 0;
        }
        node_id = (uint8_t)(cJSON_IsNumber(node) ? node->valueint
                                                 : (uint8_t)strtoul(node->valuestring, NULL, 0));

        snprintf(
            response, response_size,
            "{\"status\":\"accepted\",\"op\":\"sdo_read\",\"bus\":\"%s\",\"node_id\":%u,"
            "\"index\":%u,\"sub_index\":%u,\"note\":\"SDO is config-only in thin adapter mode\"}",
            g_config.buses[bus_index].name, node_id, index, sub_index);
        cJSON_Delete(root);
        return 0;
    }

    if (op != NULL && strcmp(op, "sdo_write") == 0)
    {
        uint16_t index    = 0U;
        uint8_t sub_index = 0U;
        uint8_t node_id   = 0U;
        uint8_t data[16];
        size_t data_len = 0U;

        cJSON *idx = cJSON_GetObjectItem(root, "index");
        if (idx != NULL && idx->valuestring)
        {
            index = (uint16_t)strtoul(idx->valuestring, NULL, 0);
        }
        else if (idx != NULL && cJSON_IsNumber(idx))
        {
            index = (uint16_t)idx->valueint;
        }

        cJSON *sub = cJSON_GetObjectItem(root, "sub_index");
        if (sub != NULL)
        {
            sub_index =
                (uint8_t)(cJSON_IsNumber(sub) ? sub->valueint
                                              : (uint8_t)strtoul(sub->valuestring, NULL, 0));
        }

        cJSON *node = cJSON_GetObjectItem(root, "node_id");
        if (node == NULL || (!cJSON_IsNumber(node) && !cJSON_IsString(node)))
        {
            cJSON_Delete(root);
            snprintf(
                response, response_size,
                "{\"status\":\"sdo_write_failed\",\"reason\":\"missing_node_id\",\"bus\":\"%s\"}",
                g_config.buses[bus_index].name);
            return 0;
        }
        node_id = (uint8_t)(cJSON_IsNumber(node) ? node->valueint
                                                 : (uint8_t)strtoul(node->valuestring, NULL, 0));

        cJSON *value = cJSON_GetObjectItem(root, "value");
        if (value != NULL)
        {
            if (cJSON_IsNumber(value))
            {
                uint32_t v = (uint32_t)value->valueint;
                memcpy(data, &v, sizeof(v));
                data_len = sizeof(v);
            }
        }

        if (data_len == 0U)
        {
            cJSON_Delete(root);
            snprintf(response, response_size,
                     "{\"status\":\"sdo_write_failed\",\"reason\":\"missing_value\"}");
            return 0;
        }

        cJSON_Delete(root);
        snprintf(
            response, response_size,
            "{\"status\":\"accepted\",\"op\":\"sdo_write\",\"bus\":\"%s\",\"node_id\":%u,"
            "\"index\":%u,\"sub_index\":%u,\"note\":\"SDO is config-only in thin adapter mode\"}",
            g_config.buses[bus_index].name, node_id, index, sub_index);
        return 0;
    }

    cJSON *status = cJSON_GetObjectItem(root, "status");
    if (status != NULL && strcmp(status->valuestring, "poll") == 0)
    {
        snprintf(
            response, response_size, "{\"status\":\"ok\",\"pending_sdo\":%d,\"active\":%s}",
            g_runtime_bus_count > 0 ? (g_runtime_buses[bus_index].sdo_transaction.active ? 1 : 0)
                                    : 0,
            g_runtime_bus_count > 0 && g_runtime_buses[bus_index].sdo_transaction.active ? "true"
                                                                                         : "false");
        cJSON_Delete(root);
        return 0;
    }

    cJSON_Delete(root);
    snprintf(response, response_size, "{\"status\":\"accepted\",\"bus_count\":%d,\"command\":%s}",
             g_runtime_bus_count, command_json);
    return 0;
}
