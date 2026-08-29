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

#include "libs/CANopenLinux/CO_epoll_interface.h"
#include "libs/CANopenNode/301/CO_ODinterface.h"
#include "libs/CANopenNode/CANopen.h"
#include "OD.h"
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

typedef struct
{
    CO_t *co;
    bool initialized;
    bool epoll_ready;
    bool startup_confirmed;
    bool reconnect_required;
    bool bootup_seen;
    uint8_t node_id;
    uint32_t bitrate;
    uint32_t last_start_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_bootup_ms;
    uint8_t retry_count;
    int fd;
    CO_epoll_t epoll;
    canopen_sdo_transaction_t sdo_transaction;
} canopen_runtime_bus_t;

static int canopen_parse_iec_address(const char *address, char *prefix, size_t prefix_len,
                                     int *index, int *bit);

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
            OD_PERSIST_COMM.x1016_consumerHeartbeatTime[enabled_count] = heartbeat_ms;
        }
        enabled_count++;
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
        runtime->retry_count        = 0U;
        plugin_logger_info(
            &g_logger, "CANopen lifecycle: node %u reached Operational; startup confirmed", nodeId);
        return;
    }

    if (state == CO_NMT_STOPPED || state == CO_NMT_PRE_OPERATIONAL)
    {
        runtime->startup_confirmed  = false;
        runtime->reconnect_required = true;
        plugin_logger_warn(&g_logger, "CANopen lifecycle: node %u not operational yet (state=%d)",
                           nodeId, state);
    }
}

static void canopen_runtime_lifecycle_tick(const canopen_bus_config_t *bus,
                                           canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL || runtime->co == NULL || runtime->co->HBcons == NULL)
    {
        return;
    }

    bool any_operational = false;
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
            any_timeout = true;
        }
        if (node->NMTstate == CO_NMT_INITIALIZING)
        {
            any_bootup = true;
        }
        if (node->NMTstate == CO_NMT_OPERATIONAL)
        {
            any_operational = true;
        }
    }

    if (any_bootup)
    {
        runtime->bootup_seen        = true;
        runtime->reconnect_required = true;
        runtime->startup_confirmed  = false;
        runtime->last_bootup_ms     = now_ms;
    }
    if (any_timeout)
    {
        runtime->reconnect_required = true;
        runtime->startup_confirmed  = false;
        runtime->retry_count++;
        plugin_logger_warn(&g_logger,
                           "CANopen lifecycle: heartbeat timeout on bus=%s; scheduling reconnect",
                           bus->name);
    }
    if (any_operational)
    {
        runtime->startup_confirmed  = true;
        runtime->reconnect_required = false;
        runtime->retry_count        = 0U;
    }
}

static void canopen_start_configured_slaves(const canopen_bus_config_t *bus, CO_t *co,
                                            canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || co == NULL || co->NMT == NULL)
    {
        return;
    }

    uint32_t now_ms = canopen_now_ms();
    if (runtime != NULL && runtime->startup_confirmed && !runtime->reconnect_required)
    {
        return;
    }
    if (runtime != NULL && runtime->last_start_ms != 0U &&
        (now_ms - runtime->last_start_ms) < 1000U)
    {
        return;
    }

    uint8_t started = 0U;
    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled || slave->node_id == 0U || slave->node_id > 127U)
        {
            continue;
        }

        CO_ReturnError_t err =
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, slave->node_id);
        if (err != CO_ERROR_NO)
        {
            plugin_logger_warn(&g_logger, "NMT start failed for bus=%s slave=%s node_id=%u err=%d",
                               bus->name, slave->name, slave->node_id, err);
            continue;
        }

        started++;
        runtime->last_start_ms      = now_ms;
        runtime->startup_confirmed  = false;
        runtime->reconnect_required = false;
        runtime->retry_count++;
        plugin_logger_info(&g_logger, "NMT start sent: bus=%s slave=%s node_id=%u command=0x%02X",
                           bus->name, slave->name, slave->node_id, CO_NMT_ENTER_OPERATIONAL);
    }

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
            CO_epoll_wait(&runtime->epoll);
            CO_epoll_processMain(&runtime->epoll, runtime->co, true, &reset);
            CO_epoll_processRT(&runtime->epoll, runtime->co, false);
            CO_epoll_processLast(&runtime->epoll);
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

static void apply_od_pdo_defaults(const canopen_bus_config_t *bus)
{
    if (bus == NULL)
    {
        return;
    }

    plugin_logger_info(
        &g_logger, "apply_od_pdo_defaults: bus=%s slave_count=%d bus_od=%d bus_tpdo=%d bus_rpdo=%d",
        bus->name, bus->slave_count, bus->od_entry_count, bus->tpdo_count, bus->rpdo_count);

    const uint16_t local_node_id = bus->local_node_id > 0U ? bus->local_node_id : 0x7FU;

    /* Runtime uses the generated OD.c defaults as the source of truth. JSON od_entries are
     * editor metadata only and are intentionally ignored here; slave PDOs are consumed from the
     * binding table instead of dynamically creating OD entries from JSON. */
    OD_PERSIST_COMM.x1018_identity.vendor_ID      = (uint32_t)local_node_id;
    OD_PERSIST_COMM.x1018_identity.productCode    = (uint32_t)bus->bitrate;
    OD_PERSIST_COMM.x1018_identity.revisionNumber = bus->heartbeat_ms;
    OD_PERSIST_COMM.x1018_identity.serialNumber   = bus->sync_period_ms;

    const canopen_pdo_t *rpdo_ref = NULL;
    const canopen_pdo_t *tpdo_ref = NULL;

    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        if (!slave->enabled)
        {
            continue;
        }
        if (rpdo_ref == NULL && slave->rpdo_count > 0 && slave->rpdo[0].mapping_count > 0)
        {
            rpdo_ref = &slave->rpdo[0];
        }
        if (tpdo_ref == NULL && slave->tpdo_count > 0 && slave->tpdo[0].mapping_count > 0)
        {
            tpdo_ref = &slave->tpdo[0];
        }
    }

    if (rpdo_ref != NULL)
    {
        uint8_t count = (uint8_t)canopen_min_int(rpdo_ref->mapping_count, 8);
        plugin_logger_info(&g_logger,
                           "apply_od_pdo_defaults: selected RPDO=%s index=0x%04X mappings=%d",
                           rpdo_ref->name, rpdo_ref->index, rpdo_ref->mapping_count);
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.highestSub_indexSupported = 0x05U;
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.COB_IDUsedByRPDO =
            (uint32_t)(0x200U + local_node_id);
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.transmissionType = 0xFEU;
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.eventTimer       = 0U;

        memset(&OD_PERSIST_COMM.x1600_RPDOMappingParameter, 0,
               sizeof(OD_PERSIST_COMM.x1600_RPDOMappingParameter));
        OD_PERSIST_COMM.x1600_RPDOMappingParameter.numberOfMappedApplicationObjectsInPDO = count;
        for (uint8_t i = 0; i < count; i++)
        {
            const canopen_pdo_mapping_t *mapping = &rpdo_ref->mapping[i];
            uint32_t map                         = ((uint32_t)mapping->index << 16U) |
                                                   ((uint32_t)mapping->sub_index << 8U) |
                                                   ((uint32_t)mapping->bit_length & 0xFFU);
            switch (i)
            {
            case 0:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject1 = map;
                break;
            case 1:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject2 = map;
                break;
            case 2:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject3 = map;
                break;
            case 3:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject4 = map;
                break;
            case 4:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject5 = map;
                break;
            case 5:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject6 = map;
                break;
            case 6:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject7 = map;
                break;
            case 7:
                OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject8 = map;
                break;
            default:
                break;
            }
        }
    }

    if (tpdo_ref != NULL)
    {
        uint8_t count = (uint8_t)canopen_min_int(tpdo_ref->mapping_count, 8);
        plugin_logger_info(&g_logger,
                           "apply_od_pdo_defaults: selected TPDO=%s index=0x%04X mappings=%d",
                           tpdo_ref->name, tpdo_ref->index, tpdo_ref->mapping_count);
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.highestSub_indexSupported = 0x06U;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.COB_IDUsedByTPDO =
            (uint32_t)(0x180U + local_node_id);
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.transmissionType = 0xFEU;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.inhibitTime      = 0U;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.eventTimer       = 0U;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.SYNCStartValue   = 0U;

        memset(&OD_PERSIST_COMM.x1A00_TPDOMappingParameter, 0,
               sizeof(OD_PERSIST_COMM.x1A00_TPDOMappingParameter));
        OD_PERSIST_COMM.x1A00_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO = count;
        for (uint8_t i = 0; i < count; i++)
        {
            const canopen_pdo_mapping_t *mapping = &tpdo_ref->mapping[i];
            uint32_t map                         = ((uint32_t)mapping->index << 16U) |
                                                   ((uint32_t)mapping->sub_index << 8U) |
                                                   ((uint32_t)mapping->bit_length & 0xFFU);
            switch (i)
            {
            case 0:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject1 = map;
                break;
            case 1:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject2 = map;
                break;
            case 2:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject3 = map;
                break;
            case 3:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject4 = map;
                break;
            case 4:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject5 = map;
                break;
            case 5:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject6 = map;
                break;
            case 6:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject7 = map;
                break;
            case 7:
                OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject8 = map;
                break;
            default:
                break;
            }
        }
    }
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
        char *suffix = work + 2;

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

static int canopen_od_to_plc_input(const canopen_bus_config_t *bus,
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

    if (mapping->direction[0] != '\0' && strcasecmp(mapping->direction, "input") != 0)
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
        uint8_t val = 0U;
        if (OD_get_u8(od_entry, mapping->sub_index, &val, false) == ODR_OK &&
            g_args.journal_write_bool &&
            canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                      &iec_bit))
        {
            g_args.journal_write_bool(0, iec_index, iec_bit, (int)val);
            return 1;
        }
        return 0;
    }
    if (strcasecmp(type, "u8") == 0 || strcasecmp(type, "byte") == 0)
    {
        uint8_t val = 0U;
        if (OD_get_u8(od_entry, mapping->sub_index, &val, false) == ODR_OK &&
            g_args.journal_write_byte &&
            canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                      &iec_bit))
        {
            g_args.journal_write_byte(3, iec_index, (int)val);
            return 1;
        }
        return 0;
    }
    if (strcasecmp(type, "i16") == 0 || strcasecmp(type, "u16") == 0)
    {
        uint16_t val = 0U;
        if (OD_get_u16(od_entry, mapping->sub_index, &val, false) == ODR_OK &&
            g_args.journal_write_int &&
            canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                      &iec_bit))
        {
            g_args.journal_write_int(5, iec_index, (int)val);
            return 1;
        }
        return 0;
    }
    if (strcasecmp(type, "i32") == 0 || strcasecmp(type, "u32") == 0)
    {
        uint32_t val = 0U;
        if (OD_get_u32(od_entry, mapping->sub_index, &val, false) == ODR_OK &&
            g_args.journal_write_dint &&
            canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                      &iec_bit))
        {
            g_args.journal_write_dint(8, iec_index, (unsigned int)val);
            return 1;
        }
        return 0;
    }
    if (strcasecmp(type, "i64") == 0 || strcasecmp(type, "u64") == 0)
    {
        uint64_t val = 0U;
        if (OD_get_u64(od_entry, mapping->sub_index, &val, false) == ODR_OK &&
            g_args.journal_write_lint &&
            canopen_parse_iec_address(mapping->plc_address, prefix, sizeof(prefix), &iec_index,
                                      &iec_bit))
        {
            g_args.journal_write_lint(11, iec_index, (unsigned long long)val);
            return 1;
        }
        return 0;
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
        for (int i = 0; i < slave->rpdo_count; i++)
        {
            for (int j = 0; j < slave->rpdo[i].mapping_count; j++)
            {
                canopen_bus_plc_to_od(bus, &slave->rpdo[i].mapping[j]);
            }
        }
    }

    if (g_args.image_unlock)
    {
        g_args.image_unlock();
    }
}

static void sync_plc_image_to_canopen(const canopen_bus_config_t *bus)
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
                canopen_od_to_plc_input(bus, &slave->tpdo[i].mapping[j]);
            }
        }
        for (int i = 0; i < slave->rpdo_count; i++)
        {
            for (int j = 0; j < slave->rpdo[i].mapping_count; j++)
            {
                canopen_od_to_plc_input(bus, &slave->rpdo[i].mapping[j]);
            }
        }
    }

    if (g_args.image_unlock)
    {
        g_args.image_unlock();
    }
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

    for (int s = 0; s < bus->slave_count; s++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[s];
        plugin_logger_info(
            &g_logger,
            "Bus[%d] slave[%d]: name=%s node_id=%u enabled=%d od_entries=%d tpdo=%d rpdo=%d sdo=%d",
            bus_index, s, slave->name, slave->node_id, slave->enabled, slave->od_entry_count,
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

    apply_od_pdo_defaults(bus);
    canopen_apply_heartbeat_defaults(bus, co);

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

    CO_CANsetNormalMode(co->CANmodule);
    plugin_logger_info(&g_logger, "CANopen init sequence: step=CAN_normal_mode bus=%s if=%s fd=%d",
                       bus->name, bus->interface, g_runtime_buses[bus_index].fd);

    if (g_runtime_buses[bus_index].fd >= 0)
    {
        /* Use the CANopenLinux mainloop pattern: one epoll instance, initialized once,
         * attached to the stack before normal mode is enabled. This keeps the OpenPLC
         * plugin as a thin adapter rather than maintaining a second runtime loop. */
        CO_epoll_initCANopenMain(&g_runtime_buses[bus_index].epoll, co);
        g_runtime_buses[bus_index].epoll_ready = true;
        plugin_logger_info(&g_logger, "CANopen Linux epoll runtime enabled for bus=%s interface=%s",
                           bus->name, bus->interface);
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
                       "od_entries=%d tpdo=%d rpdo=%d socket_fd=%d",
                       bus->name, bus->interface, bus->local_node_id, bus->bitrate,
                       bus->od_entry_count, bus->tpdo_count, bus->rpdo_count,
                       g_runtime_buses[bus_index].fd);
    return 0;
}

static void write_json_status(char *out, size_t out_size)
{
    if (!out || out_size == 0)
        return;

    int pending_sdo = 0;
    for (int i = 0; i < g_runtime_bus_count; i++)
    {
        if (g_runtime_buses[i].sdo_transaction.active)
        {
            pending_sdo++;
        }
    }

    snprintf(out, out_size,
             "{\n"
             "  \"bus_count\": %d,\n"
             "  \"enabled_buses\": %d,\n"
             "  \"pending_sdo\": %d\n"
             "}",
             g_config.bus_count, g_runtime_bus_count, pending_sdo);
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
                           "od_entries=%d tpdo=%d rpdo=%d",
                           i, bus->name, bus->interface, bus->local_node_id, bus->bitrate,
                           bus->od_entry_count, bus->tpdo_count, bus->rpdo_count);
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
            canopen_runtime_lifecycle_tick(bus, &g_runtime_buses[i]);
            if (g_runtime_buses[i].reconnect_required || !g_runtime_buses[i].startup_confirmed)
            {
                canopen_start_configured_slaves(bus, g_runtime_buses[i].co, &g_runtime_buses[i]);
            }

            /* Thin adapter phase: keep PLC↔PDO translation, but do not run a
             * custom SDO transaction loop. The Linux CANopen runtime handles the
             * protocol processing itself in the worker thread. */
            sync_plc_image_to_canopen(bus);
            sync_canopen_bus_to_plc_image(bus);
        }
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

int get_stats(char *out, size_t out_size)
{
    if (!out || out_size == 0)
        return -1;
    write_json_status(out, out_size);
    return 0;
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
