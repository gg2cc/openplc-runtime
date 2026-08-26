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
#include "libs/CANopenNode/CANopen.h"
#include "OD.h"
#include "plugin_logger.h"
#include <linux/can.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    uint8_t node_id;
    uint32_t bitrate;
    int fd;
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

static void reset_runtime_state(void)
{
    memset(g_runtime_buses, 0, sizeof(g_runtime_buses));
    g_runtime_bus_count  = 0;
    g_canopen_rx_running = false;
}

static void canopen_clear_sdo_transaction(canopen_runtime_bus_t *runtime)
{
    if (runtime == NULL)
    {
        return;
    }

    runtime->sdo_transaction.active        = false;
    runtime->sdo_transaction.read          = false;
    runtime->sdo_transaction.node_id       = 0U;
    runtime->sdo_transaction.index         = 0U;
    runtime->sdo_transaction.sub_index     = 0U;
    runtime->sdo_transaction.payload_len   = 0U;
    runtime->sdo_transaction.expected_len  = 0U;
    runtime->sdo_transaction.elapsed_ms    = 0U;
    runtime->sdo_transaction.timeout_ms    = 0U;
    runtime->sdo_transaction.retry_count   = 0U;
    runtime->sdo_transaction.max_retries   = 3U;
    runtime->sdo_transaction.abort_code    = CO_SDO_AB_NONE;
    runtime->sdo_transaction.last_error[0] = '\0';
    memset(runtime->sdo_transaction.payload, 0, sizeof(runtime->sdo_transaction.payload));

    if (runtime->co != NULL && runtime->co->SDOclient != NULL)
    {
        CO_SDOclientClose(&runtime->co->SDOclient[0]);
    }
}

static void canopen_fail_sdo_transaction(canopen_runtime_bus_t *runtime,
                                         CO_SDO_abortCode_t abort_code, const char *reason)
{
    if (runtime == NULL)
    {
        return;
    }

    runtime->sdo_transaction.abort_code = abort_code;
    if (reason != NULL && reason[0] != '\0')
    {
        snprintf(runtime->sdo_transaction.last_error, sizeof(runtime->sdo_transaction.last_error),
                 "%s", reason);
    }
    canopen_clear_sdo_transaction(runtime);
}

static int canopen_retry_sdo_transaction(canopen_runtime_bus_t *runtime)
{
    if (runtime == NULL || runtime->co == NULL || runtime->co->SDOclient == NULL)
    {
        return -1;
    }

    if (runtime->sdo_transaction.retry_count >= runtime->sdo_transaction.max_retries)
    {
        canopen_fail_sdo_transaction(runtime, CO_SDO_AB_DEVICE_INCOMPAT, "sdo retry exhausted");
        return -1;
    }

    runtime->sdo_transaction.retry_count++;
    runtime->sdo_transaction.elapsed_ms = 0U;
    runtime->sdo_transaction.timeout_ms = 1500U;
    snprintf(runtime->sdo_transaction.last_error, sizeof(runtime->sdo_transaction.last_error),
             "retry %u", (unsigned int)runtime->sdo_transaction.retry_count);
    return 0;
}

static int canopen_begin_sdo_request(canopen_runtime_bus_t *runtime, bool read, uint8_t node_id,
                                     uint16_t index, uint8_t sub_index, const uint8_t *data,
                                     size_t data_len)
{
    if (runtime == NULL || runtime->co == NULL || runtime->co->SDOclient == NULL)
    {
        return -1;
    }

    CO_SDOclient_t *client = &runtime->co->SDOclient[0];
    CO_SDO_return_t ret    = CO_SDOclient_setup(client, CO_CAN_ID_SDO_CLI + node_id,
                                                CO_CAN_ID_SDO_SRV + node_id, node_id);
    if (ret != CO_SDO_RT_ok_communicationEnd)
    {
        return -1;
    }

    canopen_clear_sdo_transaction(runtime);
    runtime->sdo_transaction.active       = true;
    runtime->sdo_transaction.read         = read;
    runtime->sdo_transaction.node_id      = node_id;
    runtime->sdo_transaction.index        = index;
    runtime->sdo_transaction.sub_index    = sub_index;
    runtime->sdo_transaction.timeout_ms   = 1500U;
    runtime->sdo_transaction.max_retries  = 3U;
    runtime->sdo_transaction.expected_len = data_len;
    runtime->sdo_transaction.payload_len  = 0U;
    memset(runtime->sdo_transaction.payload, 0, sizeof(runtime->sdo_transaction.payload));

    if (read)
    {
        ret = CO_SDOclientUploadInitiate(client, index, sub_index, 1000U, false);
        if (ret != CO_SDO_RT_ok_communicationEnd)
        {
            canopen_clear_sdo_transaction(runtime);
            return -1;
        }
        snprintf(runtime->sdo_transaction.last_error, sizeof(runtime->sdo_transaction.last_error),
                 "upload initiated");
        return 0;
    }

    if (data != NULL && data_len > 0U)
    {
        memcpy(runtime->sdo_transaction.payload, data,
               (data_len < sizeof(runtime->sdo_transaction.payload))
                   ? data_len
                   : sizeof(runtime->sdo_transaction.payload));
        runtime->sdo_transaction.payload_len = data_len < sizeof(runtime->sdo_transaction.payload)
                                                   ? data_len
                                                   : sizeof(runtime->sdo_transaction.payload);
    }

    ret = CO_SDOclientDownloadInitiate(client, index, sub_index, data_len, 1000U, false);
    if (ret != CO_SDO_RT_ok_communicationEnd)
    {
        canopen_clear_sdo_transaction(runtime);
        return -1;
    }

    size_t written = CO_SDOclientDownloadBufWrite(client, data, data_len);
    if (written < data_len)
    {
        canopen_clear_sdo_transaction(runtime);
        snprintf(runtime->sdo_transaction.last_error, sizeof(runtime->sdo_transaction.last_error),
                 "download buffer partial");
        return -1;
    }

    snprintf(runtime->sdo_transaction.last_error, sizeof(runtime->sdo_transaction.last_error),
             "download initiated");
    return 0;
}

static int canopen_iec_buffer_type_for_address(const char *address, int *type, int *index, int *bit)
{
    char prefix[4];
    int parsed_index = 0;
    int parsed_bit   = 0;
    if (!canopen_parse_iec_address(address, prefix, sizeof(prefix), &parsed_index, &parsed_bit))
    {
        return 0;
    }

    if (strcasecmp(prefix, "%IX") == 0 || strcasecmp(prefix, "%QX") == 0)
    {
        *type  = (prefix[1] == 'I') ? 0 : 1;
        *index = parsed_index;
        *bit   = parsed_bit;
        return 1;
    }
    if (strcasecmp(prefix, "%IB") == 0 || strcasecmp(prefix, "%QB") == 0)
    {
        *type  = (prefix[1] == 'I') ? 3 : 4;
        *index = parsed_index;
        *bit   = 0;
        return 1;
    }
    if (strcasecmp(prefix, "%IW") == 0 || strcasecmp(prefix, "%QW") == 0)
    {
        *type  = (prefix[1] == 'I') ? 5 : 6;
        *index = parsed_index;
        *bit   = 0;
        return 1;
    }
    if (strcasecmp(prefix, "%ID") == 0 || strcasecmp(prefix, "%QD") == 0)
    {
        *type  = (prefix[1] == 'I') ? 8 : 9;
        *index = parsed_index;
        *bit   = 0;
        return 1;
    }
    if (strcasecmp(prefix, "%IL") == 0 || strcasecmp(prefix, "%QL") == 0)
    {
        *type  = (prefix[1] == 'I') ? 11 : 12;
        *index = parsed_index;
        *bit   = 0;
        return 1;
    }

    return 0;
}

static int canopen_read_plc_value_for_sdo(const canopen_sdo_entry_t *entry, uint8_t *buf,
                                          size_t buf_size, size_t *written)
{
    if (entry == NULL || buf == NULL || written == NULL || buf_size == 0U ||
        entry->plc_address[0] == '\0')
    {
        return 0;
    }

    const char *type_name = entry->data_type[0] ? entry->data_type : "u32";
    int buffer_type       = 0;
    int plc_index         = 0;
    int bit_index         = 0;

    if (!canopen_iec_buffer_type_for_address(entry->plc_address, &buffer_type, &plc_index,
                                             &bit_index))
    {
        return 0;
    }

    if (strcasecmp(type_name, "bool") == 0)
    {
        if (buffer_type == 0 && g_args.bool_input != NULL && g_args.bool_input[plc_index] != NULL &&
            g_args.bool_input[plc_index][bit_index] != NULL)
        {
            buf[0] = (*g_args.bool_input[plc_index][bit_index] != 0) ? 1U : 0U;
        }
        else if (buffer_type == 1 && g_args.bool_output != NULL &&
                 g_args.bool_output[plc_index] != NULL &&
                 g_args.bool_output[plc_index][bit_index] != NULL)
        {
            buf[0] = (*g_args.bool_output[plc_index][bit_index] != 0) ? 1U : 0U;
        }
        else
        {
            return 0;
        }
        *written = 1U;
        return 1;
    }

    if (strcasecmp(type_name, "byte") == 0 || strcasecmp(type_name, "u8") == 0)
    {
        if (buffer_type == 3 && g_args.byte_input != NULL && g_args.byte_input[plc_index] != NULL)
        {
            buf[0] = *g_args.byte_input[plc_index];
        }
        else if (buffer_type == 4 && g_args.byte_output != NULL &&
                 g_args.byte_output[plc_index] != NULL)
        {
            buf[0] = *g_args.byte_output[plc_index];
        }
        else
        {
            return 0;
        }
        *written = 1U;
        return 1;
    }

    if (strcasecmp(type_name, "u16") == 0 || strcasecmp(type_name, "i16") == 0)
    {
        uint16_t val = 0U;
        if (buffer_type == 5 && g_args.int_input != NULL && g_args.int_input[plc_index] != NULL)
        {
            val = *g_args.int_input[plc_index];
        }
        else if (buffer_type == 6 && g_args.int_output != NULL &&
                 g_args.int_output[plc_index] != NULL)
        {
            val = *g_args.int_output[plc_index];
        }
        else
        {
            return 0;
        }
        memcpy(buf, &val, sizeof(val));
        *written = sizeof(val);
        return 1;
    }

    if (strcasecmp(type_name, "u32") == 0 || strcasecmp(type_name, "i32") == 0 ||
        strcasecmp(type_name, "f32") == 0)
    {
        uint32_t val = 0U;
        if (buffer_type == 8 && g_args.dint_input != NULL && g_args.dint_input[plc_index] != NULL)
        {
            val = *g_args.dint_input[plc_index];
        }
        else if (buffer_type == 9 && g_args.dint_output != NULL &&
                 g_args.dint_output[plc_index] != NULL)
        {
            val = *g_args.dint_output[plc_index];
        }
        else
        {
            return 0;
        }
        memcpy(buf, &val, sizeof(val));
        *written = sizeof(val);
        return 1;
    }

    if (strcasecmp(type_name, "u64") == 0 || strcasecmp(type_name, "i64") == 0 ||
        strcasecmp(type_name, "f64") == 0)
    {
        uint64_t val = 0U;
        if (buffer_type == 11 && g_args.lint_input != NULL && g_args.lint_input[plc_index] != NULL)
        {
            val = *g_args.lint_input[plc_index];
        }
        else if (buffer_type == 12 && g_args.lint_output != NULL &&
                 g_args.lint_output[plc_index] != NULL)
        {
            val = *g_args.lint_output[plc_index];
        }
        else
        {
            return 0;
        }
        memcpy(buf, &val, sizeof(val));
        *written = sizeof(val);
        return 1;
    }

    return 0;
}

static bool canopen_sdo_direction_matches(const canopen_sdo_entry_t *entry, bool read)
{
    if (entry == NULL)
    {
        return false;
    }

    if (entry->direction[0] == '\0')
    {
        return true;
    }

    if (read)
    {
        return (strcasecmp(entry->direction, "input") == 0 ||
                strcasecmp(entry->direction, "read") == 0);
    }

    return (strcasecmp(entry->direction, "output") == 0 ||
            strcasecmp(entry->direction, "write") == 0);
}

static int canopen_write_plc_value_from_sdo(const canopen_bus_config_t *bus,
                                            const canopen_sdo_entry_t *entry, const uint8_t *buf,
                                            size_t buf_len)
{
    (void)bus;
    if (entry == NULL || buf == NULL || entry->plc_address[0] == '\0')
    {
        return 0;
    }

    const char *type_name = entry->data_type[0] ? entry->data_type : "u32";
    int buffer_type       = 0;
    int plc_index         = 0;
    int bit_index         = 0;

    if (!canopen_iec_buffer_type_for_address(entry->plc_address, &buffer_type, &plc_index,
                                             &bit_index))
    {
        return 0;
    }

    if (strcasecmp(type_name, "bool") == 0 && buf_len > 0U)
    {
        int journal_type = 0;
        if (buffer_type == 0)
        {
            journal_type = 0;
        }
        else if (buffer_type == 1)
        {
            journal_type = 1;
        }
        else
        {
            return 0;
        }
        if (g_args.journal_write_bool == NULL)
        {
            return 0;
        }
        g_args.journal_write_bool(journal_type, (uint16_t)plc_index, (uint8_t)bit_index,
                                  (buf[0] != 0) ? 1 : 0);
        return 1;
    }

    if ((strcasecmp(type_name, "byte") == 0 || strcasecmp(type_name, "u8") == 0) && buf_len >= 1U)
    {
        int journal_type = (buffer_type == 3) ? 3 : ((buffer_type == 4) ? 4 : -1);
        if (journal_type < 0 || g_args.journal_write_byte == NULL)
        {
            return 0;
        }
        g_args.journal_write_byte(journal_type, (uint16_t)plc_index, buf[0]);
        return 1;
    }

    if ((strcasecmp(type_name, "u16") == 0 || strcasecmp(type_name, "i16") == 0) && buf_len >= 2U)
    {
        uint16_t value = 0U;
        memcpy(&value, buf, sizeof(value));
        int journal_type = (buffer_type == 5) ? 5 : ((buffer_type == 6) ? 6 : -1);
        if (journal_type < 0 || g_args.journal_write_int == NULL)
        {
            return 0;
        }
        g_args.journal_write_int(journal_type, (uint16_t)plc_index, (int)value);
        return 1;
    }

    if ((strcasecmp(type_name, "u32") == 0 || strcasecmp(type_name, "i32") == 0 ||
         strcasecmp(type_name, "f32") == 0) &&
        buf_len >= 4U)
    {
        uint32_t value = 0U;
        memcpy(&value, buf, sizeof(value));
        int journal_type = (buffer_type == 8) ? 8 : ((buffer_type == 9) ? 9 : -1);
        if (journal_type < 0 || g_args.journal_write_dint == NULL)
        {
            return 0;
        }
        g_args.journal_write_dint(journal_type, (uint16_t)plc_index, value);
        return 1;
    }

    if ((strcasecmp(type_name, "u64") == 0 || strcasecmp(type_name, "i64") == 0 ||
         strcasecmp(type_name, "f64") == 0) &&
        buf_len >= 8U)
    {
        uint64_t value = 0U;
        memcpy(&value, buf, sizeof(value));
        int journal_type = (buffer_type == 11) ? 11 : ((buffer_type == 12) ? 12 : -1);
        if (journal_type < 0 || g_args.journal_write_lint == NULL)
        {
            return 0;
        }
        g_args.journal_write_lint(journal_type, (uint16_t)plc_index, value);
        return 1;
    }

    return 0;
}

static void canopen_queue_plc_sdo_output(const canopen_bus_config_t *bus,
                                         canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL || runtime->co == NULL || runtime->co->SDOclient == NULL ||
        runtime->sdo_transaction.active)
    {
        return;
    }

    for (int i = 0; i < bus->sdo_count; i++)
    {
        const canopen_sdo_entry_t *entry = &bus->sdo[i];
        if (!entry->bound || entry->plc_address[0] == '\0')
        {
            continue;
        }
        if (!canopen_sdo_direction_matches(entry, false))
        {
            continue;
        }

        uint8_t payload[16] = {0};
        size_t payload_len  = 0U;
        if (!canopen_read_plc_value_for_sdo(entry, payload, sizeof(payload), &payload_len))
        {
            continue;
        }

        if (canopen_begin_sdo_request(runtime, false, runtime->node_id, entry->index,
                                      entry->sub_index, payload, payload_len) == 0)
        {
            return;
        }
    }
}

static void canopen_apply_sdo_upload_to_plc(const canopen_bus_config_t *bus,
                                            const canopen_runtime_bus_t *runtime)
{
    if (bus == NULL || runtime == NULL || runtime->sdo_transaction.payload_len == 0U)
    {
        return;
    }

    for (int i = 0; i < bus->sdo_count; i++)
    {
        const canopen_sdo_entry_t *entry = &bus->sdo[i];
        if (!entry->bound || entry->plc_address[0] == '\0')
        {
            continue;
        }
        if (!canopen_sdo_direction_matches(entry, true))
        {
            continue;
        }
        if (entry->index != runtime->sdo_transaction.index ||
            entry->sub_index != runtime->sdo_transaction.sub_index)
        {
            continue;
        }

        canopen_write_plc_value_from_sdo(bus, entry, runtime->sdo_transaction.payload,
                                         runtime->sdo_transaction.payload_len);
        break;
    }
}

static void canopen_process_sdo_transaction(canopen_runtime_bus_t *runtime,
                                            const canopen_bus_config_t *bus)
{
    if (runtime == NULL || runtime->co == NULL || runtime->co->SDOclient == NULL ||
        !runtime->sdo_transaction.active)
    {
        return;
    }

    runtime->sdo_transaction.elapsed_ms += 1000U;
    if (runtime->sdo_transaction.elapsed_ms >= runtime->sdo_transaction.timeout_ms)
    {
        if (runtime->sdo_transaction.retry_count < runtime->sdo_transaction.max_retries)
        {
            canopen_retry_sdo_transaction(runtime);
            return;
        }
        canopen_fail_sdo_transaction(runtime, CO_SDO_AB_DEVICE_INCOMPAT, "sdo timeout");
        return;
    }

    CO_SDOclient_t *client        = &runtime->co->SDOclient[0];
    CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;
    size_t size_indicated         = 0U;
    size_t size_transferred       = 0U;
    uint32_t timer_next_us        = 0U;
    CO_SDO_return_t ret           = 0;

    if (runtime->sdo_transaction.read)
    {
        ret = CO_SDOclientUpload(client, 1000U, false, &abort_code, &size_indicated,
                                 &size_transferred, &timer_next_us);
        if (ret < 0)
        {
            runtime->sdo_transaction.abort_code = abort_code;
            snprintf(runtime->sdo_transaction.last_error,
                     sizeof(runtime->sdo_transaction.last_error), "upload abort 0x%08X",
                     (unsigned int)abort_code);
            if (runtime->sdo_transaction.retry_count < runtime->sdo_transaction.max_retries)
            {
                canopen_retry_sdo_transaction(runtime);
                return;
            }
            canopen_clear_sdo_transaction(runtime);
            return;
        }

        if (ret == 0)
        {
            size_t n = CO_SDOclientUploadBufRead(client, runtime->sdo_transaction.payload,
                                                 sizeof(runtime->sdo_transaction.payload));
            runtime->sdo_transaction.payload_len  = n;
            runtime->sdo_transaction.expected_len = n;
            snprintf(runtime->sdo_transaction.last_error,
                     sizeof(runtime->sdo_transaction.last_error), "upload complete");
            canopen_apply_sdo_upload_to_plc(bus, runtime);
            canopen_clear_sdo_transaction(runtime);
            return;
        }

        return;
    }

    ret = CO_SDOclientDownload(client, 1000U, false, false, &abort_code, &size_transferred,
                               &timer_next_us);
    if (ret < 0)
    {
        runtime->sdo_transaction.abort_code = abort_code;
        snprintf(runtime->sdo_transaction.last_error, sizeof(runtime->sdo_transaction.last_error),
                 "download abort 0x%08X", (unsigned int)abort_code);
        if (runtime->sdo_transaction.retry_count < runtime->sdo_transaction.max_retries)
        {
            canopen_retry_sdo_transaction(runtime);
            return;
        }
        canopen_clear_sdo_transaction(runtime);
        return;
    }

    if (ret == 0)
    {
        runtime->sdo_transaction.payload_len = size_transferred;
        snprintf(runtime->sdo_transaction.last_error, sizeof(runtime->sdo_transaction.last_error),
                 "download complete");
        canopen_clear_sdo_transaction(runtime);
    }
}

static void canopen_dispatch_rx_message(CO_t *co, const struct can_frame *frame)
{
    if (co == NULL || co->CANmodule == NULL || co->CANmodule->rxArray == NULL || frame == NULL)
    {
        return;
    }

    uint32_t msg_ident = (uint32_t)(frame->can_id & CAN_EFF_MASK);
    uint16_t ident     = (uint16_t)msg_ident;
    bool rtr           = (frame->can_id & CAN_RTR_FLAG) != 0U;

    CO_CANrx_t *buffer = NULL;
    for (uint16_t idx = 0; idx < co->CANmodule->rxSize; idx++)
    {
        CO_CANrx_t *candidate = &co->CANmodule->rxArray[idx];
        uint32_t candidate_ident =
            ((uint32_t)candidate->ident & 0x07FFU) | ((uint32_t)(rtr ? 0x0800U : 0U));
        if (((msg_ident ^ candidate_ident) & candidate->mask) == 0U)
        {
            buffer = candidate;
            break;
        }
    }

    if (buffer == NULL || buffer->CANrx_callback == NULL)
    {
        return;
    }

    typedef struct
    {
        uint32_t ident;
        uint8_t DLC;
        uint8_t data[8];
    } canopen_rx_msg_t;

    canopen_rx_msg_t rx_msg;
    memset(&rx_msg, 0, sizeof(rx_msg));
    rx_msg.ident = ident | (rtr ? 0x800U : 0U);
    rx_msg.DLC   = frame->can_dlc > 8U ? 8U : frame->can_dlc;
    memcpy(rx_msg.data, frame->data, rx_msg.DLC);
    buffer->CANrx_callback(buffer->object, &rx_msg);
}

static void canopen_flush_tx_messages(canopen_runtime_bus_t *runtime)
{
    if (runtime == NULL || runtime->co == NULL || runtime->co->CANmodule == NULL || runtime->fd < 0)
    {
        return;
    }

    CO_CANmodule_t *can_mod = runtime->co->CANmodule;
    for (uint16_t idx = 0; idx < can_mod->txSize; idx++)
    {
        CO_CANtx_t *buffer = &can_mod->txArray[idx];
        if (!buffer->bufferFull)
        {
            continue;
        }

        uint32_t id = buffer->ident & 0x7FFU;
        bool rtr    = (buffer->ident & 0x8000U) != 0U;
        uint8_t dlc = buffer->DLC;
        if (dlc > 8U)
        {
            dlc = 8U;
        }

        if (can_socket_write(runtime->fd, id, false, rtr, dlc, buffer->data) == 0)
        {
            buffer->bufferFull = false;
            if (can_mod->CANtxCount > 0U)
            {
                can_mod->CANtxCount--;
            }
        }
    }
}

static void *canopen_rx_thread_proc(void *arg)
{
    (void)arg;
    plugin_logger_info(&g_logger, "CANopen RX thread started");

    while (g_canopen_rx_running)
    {
        bool any_bus_open = false;
        for (int i = 0; i < g_runtime_bus_count; i++)
        {
            canopen_runtime_bus_t *runtime = &g_runtime_buses[i];
            if (!runtime->initialized || runtime->fd < 0 || runtime->co == NULL)
            {
                continue;
            }

            any_bus_open       = true;
            uint32_t can_id    = 0U;
            bool eff           = false;
            bool rtr           = false;
            uint8_t dlc        = 0U;
            uint8_t payload[8] = {0};

            int res = can_socket_read(runtime->fd, &can_id, &eff, &rtr, &dlc, payload);
            if (res != 0)
            {
                continue;
            }

            struct can_frame frame;
            memset(&frame, 0, sizeof(frame));
            frame.can_dlc = dlc;
            frame.can_id  = can_id;
            if (eff)
            {
                frame.can_id |= CAN_EFF_FLAG;
            }
            if (rtr)
            {
                frame.can_id |= CAN_RTR_FLAG;
            }
            memcpy(frame.data, payload, dlc);
            canopen_dispatch_rx_message(runtime->co, &frame);
        }

        if (!any_bus_open)
        {
            usleep(100000U);
        }
    }

    plugin_logger_info(&g_logger, "CANopen RX thread stopped");
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

    OD_PERSIST_COMM.x1018_identity.vendor_ID      = bus->node_id > 0U ? (uint32_t)bus->node_id : 1U;
    OD_PERSIST_COMM.x1018_identity.productCode    = (uint32_t)bus->bitrate;
    OD_PERSIST_COMM.x1018_identity.revisionNumber = bus->heartbeat_ms;
    OD_PERSIST_COMM.x1018_identity.serialNumber   = bus->sync_period_ms;

    if (bus->rpdo_count > 0 && bus->rpdo[0].mapping_count > 0)
    {
        uint8_t count = (uint8_t)canopen_min_int(bus->rpdo[0].mapping_count, 8);
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.highestSub_indexSupported = 0x05U;
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.COB_IDUsedByRPDO =
            (bus->node_id > 0U) ? (uint32_t)(0x200U + bus->node_id) : 0x80000000U;
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.transmissionType = 0xFEU;
        OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.eventTimer       = 0U;

        memset(&OD_PERSIST_COMM.x1600_RPDOMappingParameter, 0,
               sizeof(OD_PERSIST_COMM.x1600_RPDOMappingParameter));
        OD_PERSIST_COMM.x1600_RPDOMappingParameter.numberOfMappedApplicationObjectsInPDO = count;
        for (uint8_t i = 0; i < count; i++)
        {
            const canopen_pdo_mapping_t *mapping = &bus->rpdo[0].mapping[i];
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

    if (bus->tpdo_count > 0 && bus->tpdo[0].mapping_count > 0)
    {
        uint8_t count = (uint8_t)canopen_min_int(bus->tpdo[0].mapping_count, 8);
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.highestSub_indexSupported = 0x06U;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.COB_IDUsedByTPDO =
            (bus->node_id > 0U) ? (uint32_t)(0x180U + bus->node_id) : 0x80000000U;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.transmissionType = 0xFEU;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.inhibitTime      = 0U;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.eventTimer       = 0U;
        OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.SYNCStartValue   = 0U;

        memset(&OD_PERSIST_COMM.x1A00_TPDOMappingParameter, 0,
               sizeof(OD_PERSIST_COMM.x1A00_TPDOMappingParameter));
        OD_PERSIST_COMM.x1A00_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO = count;
        for (uint8_t i = 0; i < count; i++)
        {
            const canopen_pdo_mapping_t *mapping = &bus->tpdo[0].mapping[i];
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

    for (int i = 0; i < bus->tpdo_count; i++)
    {
        for (int j = 0; j < bus->tpdo[i].mapping_count; j++)
        {
            canopen_bus_plc_to_od(bus, &bus->tpdo[i].mapping[j]);
        }
    }
    for (int i = 0; i < bus->rpdo_count; i++)
    {
        for (int j = 0; j < bus->rpdo[i].mapping_count; j++)
        {
            canopen_bus_plc_to_od(bus, &bus->rpdo[i].mapping[j]);
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

    for (int i = 0; i < bus->tpdo_count; i++)
    {
        for (int j = 0; j < bus->tpdo[i].mapping_count; j++)
        {
            canopen_od_to_plc_input(bus, &bus->tpdo[i].mapping[j]);
        }
    }
    for (int i = 0; i < bus->rpdo_count; i++)
    {
        for (int j = 0; j < bus->rpdo[i].mapping_count; j++)
        {
            canopen_od_to_plc_input(bus, &bus->rpdo[i].mapping[j]);
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

    g_runtime_buses[bus_index].co          = co;
    g_runtime_buses[bus_index].initialized = true;
    g_runtime_buses[bus_index].node_id =
        (uint8_t)((bus->node_id >= 1U && bus->node_id <= 127U) ? bus->node_id : 0xFFU);
    g_runtime_buses[bus_index].bitrate = bus->bitrate;
    g_runtime_buses[bus_index].fd      = -1;

    apply_od_pdo_defaults(bus);

    uint32_t errInfo     = 0U;
    uint16_t bitRate     = get_bus_bitrate_kbps(bus);
    CO_ReturnError_t err = CO_CANinit(co, NULL, bitRate);
    if (err != CO_ERROR_NO)
    {
        plugin_logger_error(&g_logger, "CO_CANinit() failed for %s: %d", bus->name, err);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }

    err = CO_CANopenInit(co, NULL, NULL, OD, NULL, CANOPEN_NMT_CONTROL,
                         (bus->heartbeat_ms > 0U) ? (uint16_t)bus->heartbeat_ms : 500U, 1000U, 500U,
                         false, g_runtime_buses[bus_index].node_id, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
    {
        plugin_logger_error(&g_logger, "CO_CANopenInit() failed for %s: %d (errInfo=0x%X)",
                            bus->name, err, errInfo);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }

    err = CO_CANopenInitPDO(co, co->em, OD, g_runtime_buses[bus_index].node_id, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
    {
        plugin_logger_error(&g_logger, "CO_CANopenInitPDO() failed for %s: %d (errInfo=0x%X)",
                            bus->name, err, errInfo);
        CO_delete(co);
        g_runtime_buses[bus_index].co          = NULL;
        g_runtime_buses[bus_index].initialized = false;
        return -1;
    }

    if (bus->interface[0] != '\0')
    {
        g_runtime_buses[bus_index].fd = can_socket_open(bus->interface, &g_logger);
        if (g_runtime_buses[bus_index].fd < 0)
        {
            plugin_logger_warn(&g_logger,
                               "SocketCAN open failed for CANopen bus %s on %s; keeping stack "
                               "initialized but offline",
                               bus->name, bus->interface);
        }
    }

    CO_CANsetNormalMode(co->CANmodule);
    plugin_logger_info(&g_logger,
                       "CANopen runtime bound: bus=%s interface=%s node_id=%u bitrate=%u "
                       "od_entries=%d tpdo=%d rpdo=%d socket_fd=%d",
                       bus->name, bus->interface, bus->node_id, bus->bitrate, bus->od_entry_count,
                       bus->tpdo_count, bus->rpdo_count, g_runtime_buses[bus_index].fd);
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
        plugin_logger_info(
            &g_logger,
            "Bus[%d]: name=%s interface=%s node_id=%u bitrate=%u od_entries=%d tpdo=%d rpdo=%d", i,
            bus->name, bus->interface, bus->node_id, bus->bitrate, bus->od_entry_count,
            bus->tpdo_count, bus->rpdo_count);
    }

    if (g_runtime_bus_count > 0)
    {
        g_canopen_rx_running = true;
        if (pthread_create(&g_canopen_rx_thread, NULL, canopen_rx_thread_proc, NULL) != 0)
        {
            plugin_logger_error(&g_logger, "Failed to create CANopen RX thread");
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
            canopen_process_sdo_transaction(&g_runtime_buses[i], bus);
            sync_canopen_bus_to_plc_image(bus);
            sync_plc_image_to_canopen(bus);
            if (!g_runtime_buses[i].sdo_transaction.active)
            {
                canopen_queue_plc_sdo_output(bus, &g_runtime_buses[i]);
            }
            canopen_flush_tx_messages(&g_runtime_buses[i]);
            CO_process(g_runtime_buses[i].co, false, 1000U, NULL);
        }
    }
}

void stop_loop(void)
{
    plugin_logger_info(&g_logger, "Stopping CANopen runtime adapter");
    g_canopen_rx_running = false;
    if (pthread_join(g_canopen_rx_thread, NULL) != 0 && g_runtime_bus_count > 0)
    {
        plugin_logger_warn(&g_logger, "CANopen RX thread not joined cleanly");
    }

    for (int i = 0; i < MAX_CANOPEN_BUSES; i++)
    {
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
        if (node != NULL)
        {
            node_id =
                (uint8_t)(cJSON_IsNumber(node) ? node->valueint
                                               : (uint8_t)strtoul(node->valuestring, NULL, 0));
        }
        else if (g_runtime_bus_count > 0)
        {
            node_id = g_runtime_buses[bus_index].node_id;
        }

        if (g_runtime_buses[bus_index].co != NULL &&
            g_runtime_buses[bus_index].co->SDOclient != NULL)
        {
            int rc = canopen_begin_sdo_request(&g_runtime_buses[bus_index], true, node_id, index,
                                               sub_index, NULL, 0U);
            if (rc == 0)
            {
                snprintf(response, response_size,
                         "{\"status\":\"queued\",\"op\":\"sdo_read\",\"bus\":\"%s\",\"node_id\":%u,"
                         "\"index\":%u,\"sub_index\":%u}",
                         g_config.buses[bus_index].name, node_id, index, sub_index);
                cJSON_Delete(root);
                return 0;
            }
        }

        snprintf(response, response_size, "{\"status\":\"sdo_read_failed\",\"bus\":\"%s\"}",
                 g_config.buses[bus_index].name);
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
        if (node != NULL)
        {
            node_id =
                (uint8_t)(cJSON_IsNumber(node) ? node->valueint
                                               : (uint8_t)strtoul(node->valuestring, NULL, 0));
        }
        else if (g_runtime_bus_count > 0)
        {
            node_id = g_runtime_buses[bus_index].node_id;
        }

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

        if (g_runtime_buses[bus_index].co != NULL &&
            g_runtime_buses[bus_index].co->SDOclient != NULL)
        {
            int rc = canopen_begin_sdo_request(&g_runtime_buses[bus_index], false, node_id, index,
                                               sub_index, data, data_len);
            if (rc == 0)
            {
                cJSON_Delete(root);
                snprintf(response, response_size,
                         "{\"status\":\"queued\",\"op\":\"sdo_write\",\"bus\":\"%s\",\"node_id\":%"
                         "u,\"index\":%u,\"sub_index\":%u}",
                         g_config.buses[bus_index].name, node_id, index, sub_index);
                return 0;
            }
        }

        cJSON_Delete(root);
        snprintf(response, response_size, "{\"status\":\"sdo_write_failed\",\"bus\":\"%s\"}",
                 g_config.buses[bus_index].name);
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
