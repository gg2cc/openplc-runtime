/**
 * @file can_plugin.c
 * @brief OpenPLC Native SocketCAN Plugin Implementation
 */

#include "can_plugin.h"
#include "can_config.h"
#include "can_netlink.h"
#include "can_socket.h"
#include "plugin_logger.h"
#include "plugin_types.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

static plugin_runtime_args_t g_args;
static plugin_logger_t g_logger;
static can_config_t g_config;
static int g_can_fds[MAX_CAN_INTERFACES];
static int g_can_fd_count                        = 0;
static uint64_t g_last_rx_ms[MAX_CAN_INTERFACES] = {0};
static uint64_t g_last_tx_ms[MAX_CAN_INTERFACES] = {0};

static pthread_t g_rx_thread;
static volatile bool g_rx_running = false;

static int mapping_width(can_data_type_t type)
{
    return type == CAN_DATA_BOOL || type == CAN_DATA_U8 || type == CAN_DATA_I8    ? 1
           : type == CAN_DATA_U16 || type == CAN_DATA_I16                         ? 2
           : type == CAN_DATA_U32 || type == CAN_DATA_I32 || type == CAN_DATA_F32 ? 4
                                                                                  : 8;
}

/* Performance counters for get_stats */
static uint64_t g_rx_count                               = 0;
static uint64_t g_tx_count                               = 0;
static uint64_t g_rx_errors                              = 0;
static uint64_t g_tx_errors                              = 0;
static uint64_t g_rx_count_by_iface[MAX_CAN_INTERFACES]  = {0};
static uint64_t g_tx_count_by_iface[MAX_CAN_INTERFACES]  = {0};
static uint64_t g_rx_errors_by_iface[MAX_CAN_INTERFACES] = {0};
static uint64_t g_tx_errors_by_iface[MAX_CAN_INTERFACES] = {0};

static uint64_t get_time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000 + (uint64_t)(tv.tv_usec / 1000);
}

static bool parse_status_address(const char *address, int *index)
{
    if (!address || !index || strncasecmp(address, "%IB", 3) != 0 || address[3] == '\0')
    {
        return false;
    }

    char *end   = NULL;
    long parsed = strtol(address + 3, &end, 10);
    if (end == address + 3 || *end != '\0' || parsed < 0 || parsed >= g_args.buffer_size)
    {
        return false;
    }

    *index = (int)parsed;
    return true;
}

static void write_status_address(const char *address, bool ok)
{
    int index = 0;
    if (g_args.journal_write_byte && parse_status_address(address, &index))
    {
        g_args.journal_write_byte(3, index, ok ? 0 : 1);
    }
}

static bool interface_data_ok(const can_interface_config_t *iface, int iface_idx, uint64_t now_ms)
{
    if (!iface || iface_idx < 0 || iface_idx >= MAX_CAN_INTERFACES)
        return false;
    uint32_t timeout_ms = iface->data_status_timeout_ms > 0 ? iface->data_status_timeout_ms : 3000U;
    uint64_t last_rx_ms = g_last_rx_ms[iface_idx];
    uint64_t last_tx_ms = g_last_tx_ms[iface_idx];
    bool rx_ok          = last_rx_ms > 0 && now_ms - last_rx_ms <= timeout_ms;
    bool tx_ok          = last_tx_ms > 0 && now_ms - last_tx_ms <= timeout_ms;
    return rx_ok || tx_ok;
}

static void update_status_outputs(uint64_t now_ms)
{
    for (int iface_idx = 0; iface_idx < g_config.interface_count && iface_idx < MAX_CAN_INTERFACES;
         iface_idx++)
    {
        const can_interface_config_t *iface = &g_config.interfaces[iface_idx];
        bool port_ok = g_can_fds[iface_idx] >= 0 && can_netlink_is_up(iface->hardware.interface);
        write_status_address(iface->port_status_plc_address, port_ok);
        write_status_address(iface->data_status_plc_address,
                             port_ok && interface_data_ok(iface, iface_idx, now_ms));
    }
}

static void *can_rx_thread_proc(void *arg)
{
    (void)arg;
    plugin_logger_info(&g_logger, "CAN RX thread started");

    uint32_t can_id = 0;
    bool eff        = false;
    bool rtr        = false;
    uint8_t dlc     = 0;
    uint8_t payload[8];

    while (g_rx_running)
    {
        bool any_fd_open = false;
        for (int iface_idx = 0; iface_idx < g_config.interface_count; iface_idx++)
        {
            int fd = g_can_fds[iface_idx];
            if (fd < 0)
            {
                continue;
            }
            any_fd_open = true;

            int res = can_socket_read(fd, &can_id, &eff, &rtr, &dlc, payload);
            if (res < 0)
            {
                g_rx_errors++;
                g_rx_errors_by_iface[iface_idx]++;
                continue;
            }

            g_rx_count++;
            g_rx_count_by_iface[iface_idx]++;
            g_last_rx_ms[iface_idx] = get_time_ms();

            can_interface_config_t *iface = &g_config.interfaces[iface_idx];
            for (int i = 0; i < iface->rx_frame_count; i++)
            {
                can_rx_frame_config_t *frame = &iface->rx_frames[i];
                if (frame->can_id == can_id && frame->eff == eff)
                {
                    for (int j = 0; j < frame->mapping_count; j++)
                    {
                        can_mapping_t *m = &frame->mappings[j];
                        if (m->byte_offset + mapping_width(m->data_type) > dlc)
                            continue;

                        switch (m->data_type)
                        {
                        case CAN_DATA_BOOL:
                        {
                            int bit_val = (payload[m->byte_offset] >> m->plc_bit) & 0x01;
                            if (g_args.journal_write_bool)
                                g_args.journal_write_bool(m->journal_type, m->plc_index, m->plc_bit,
                                                          bit_val);
                            break;
                        }
                        case CAN_DATA_U8:
                        case CAN_DATA_I8:
                        {
                            uint8_t val = payload[m->byte_offset];
                            if (g_args.journal_write_byte)
                                g_args.journal_write_byte(m->journal_type, m->plc_index, val);
                            break;
                        }
                        case CAN_DATA_U16:
                        case CAN_DATA_I16:
                        {
                            uint16_t val = (uint16_t)payload[m->byte_offset] |
                                           ((uint16_t)payload[m->byte_offset + 1] << 8);
                            if (g_args.journal_write_int)
                                g_args.journal_write_int(m->journal_type, m->plc_index, (int)val);
                            break;
                        }
                        case CAN_DATA_U32:
                        case CAN_DATA_I32:
                        case CAN_DATA_F32:
                        {
                            uint32_t val = (uint32_t)payload[m->byte_offset] |
                                           ((uint32_t)payload[m->byte_offset + 1] << 8) |
                                           ((uint32_t)payload[m->byte_offset + 2] << 16) |
                                           ((uint32_t)payload[m->byte_offset + 3] << 24);
                            if (g_args.journal_write_dint)
                                g_args.journal_write_dint(m->journal_type, m->plc_index, val);
                            break;
                        }
                        case CAN_DATA_U64:
                        case CAN_DATA_I64:
                        case CAN_DATA_F64:
                        {
                            uint64_t val = 0;
                            for (int byte = 0; byte < 8; byte++)
                                val |= (uint64_t)payload[m->byte_offset + byte] << (byte * 8);
                            if (g_args.journal_write_lint)
                                g_args.journal_write_lint(m->journal_type, m->plc_index, val);
                            break;
                        }
                        default:
                            break;
                        }
                    }
                    break;
                }
            }
        }

        if (!any_fd_open)
        {
            usleep(100000);
        }
    }

    plugin_logger_info(&g_logger, "CAN RX thread stopped");
    return NULL;
}

int init(void *args)
{
    if (!args)
        return -1;

    memset(g_can_fds, -1, sizeof(g_can_fds));

    /* Copy runtime args by value as required */
    memcpy(&g_args, args, sizeof(plugin_runtime_args_t));

    /* Initialize logger */
    if (!plugin_logger_init(&g_logger, "CAN_PLUGIN", &g_args))
    {
        return -1;
    }

    plugin_logger_info(&g_logger, "Initializing CAN Plugin...");

    /* Parse configuration. If no editor-side CAN config is provided,
     * do not auto-init the interface; the plugin remains inactive. */
    const char *cfg_path = g_args.plugin_specific_config_file_path;
    if (!cfg_path || !cfg_path[0])
    {
        plugin_logger_info(&g_logger, "No CAN config provided; skipping CAN initialization");
        return 0;
    }

    if (access(cfg_path, F_OK) != 0)
    {
        plugin_logger_info(&g_logger, "CAN config file %s not found; skipping CAN initialization",
                           cfg_path);
        return 0;
    }

    if (can_config_parse(cfg_path, &g_config, &g_logger) != 0)
    {
        plugin_logger_error(&g_logger, "Failed to parse CAN configuration file");
        return -1;
    }

    plugin_logger_info(&g_logger, "CAN Plugin initialized successfully");
    return 0;
}

int start_loop(void)
{
    if (g_config.interface_count <= 0)
    {
        plugin_logger_info(&g_logger, "No CAN interface configured; skipping CAN start");
        return 0;
    }

    plugin_logger_info(&g_logger, "Starting CAN Plugin for %d interfaces...",
                       g_config.interface_count);

    memset(g_can_fds, -1, sizeof(g_can_fds));
    memset(g_rx_count_by_iface, 0, sizeof(g_rx_count_by_iface));
    memset(g_tx_count_by_iface, 0, sizeof(g_tx_count_by_iface));
    memset(g_rx_errors_by_iface, 0, sizeof(g_rx_errors_by_iface));
    memset(g_tx_errors_by_iface, 0, sizeof(g_tx_errors_by_iface));
    memset(g_last_rx_ms, 0, sizeof(g_last_rx_ms));
    memset(g_last_tx_ms, 0, sizeof(g_last_tx_ms));
    g_can_fd_count = 0;

    for (int iface_idx = 0; iface_idx < g_config.interface_count; iface_idx++)
    {
        can_interface_config_t *iface = &g_config.interfaces[iface_idx];
        if (!iface->hardware.interface[0])
        {
            continue;
        }

        can_netlink_configure_and_up(&iface->hardware, &g_logger);

        int fd = can_socket_open(iface->hardware.interface, &g_logger);
        if (fd < 0)
        {
            plugin_logger_warn(
                &g_logger, "Could not open SocketCAN device %s; plugin running in degraded mode",
                iface->hardware.interface);
            continue;
        }

        g_can_fds[iface_idx] = fd;
        g_can_fd_count++;
    }

    if (g_can_fd_count == 0)
    {
        plugin_logger_warn(&g_logger, "No CAN socket opened; plugin will remain idle");
        return 0;
    }

    if (!g_rx_running)
    {
        g_rx_running = true;
        if (pthread_create(&g_rx_thread, NULL, can_rx_thread_proc, NULL) != 0)
        {
            plugin_logger_error(&g_logger, "Failed to create CAN RX thread");
            g_rx_running = false;
            return -1;
        }
    }
    return 0;
}

void cycle_start(void)
{
    /* Nothing to do here; RX thread writes asynchronously to Journal buffer */
}

void cycle_end(void)
{
    uint64_t now_ms = get_time_ms();
    update_status_outputs(now_ms);
    if (g_can_fd_count == 0)
        return;

    if (g_args.image_lock)
        g_args.image_lock();

    for (int iface_idx = 0; iface_idx < g_config.interface_count; iface_idx++)
    {
        can_interface_config_t *iface = &g_config.interfaces[iface_idx];
        int fd                        = g_can_fds[iface_idx];
        if (fd < 0 || iface->tx_frame_count == 0)
            continue;

        for (int i = 0; i < iface->tx_frame_count; i++)
        {
            can_tx_frame_config_t *frame = &iface->tx_frames[i];
            uint8_t payload[8]           = {0};

            for (int j = 0; j < frame->mapping_count; j++)
            {
                can_mapping_t *m = &frame->mappings[j];
                int width        = mapping_width(m->data_type);
                if (m->byte_offset < 0 || m->byte_offset + width > frame->dlc)
                    continue;

                switch (m->data_type)
                {
                case CAN_DATA_BOOL:
                {
                    if (g_args.bool_output && m->plc_index < g_args.buffer_size)
                    {
                        IEC_BOOL val = *g_args.bool_output[m->plc_index][m->plc_bit];
                        if (val)
                            payload[m->byte_offset] |= (1 << m->plc_bit);
                        else
                            payload[m->byte_offset] &= ~(1 << m->plc_bit);
                    }
                    break;
                }
                case CAN_DATA_U8:
                case CAN_DATA_I8:
                {
                    if (g_args.byte_output && m->plc_index < g_args.buffer_size)
                    {
                        IEC_BYTE val            = *g_args.byte_output[m->plc_index];
                        payload[m->byte_offset] = val;
                    }
                    break;
                }
                case CAN_DATA_U16:
                case CAN_DATA_I16:
                {
                    if (g_args.int_output && m->plc_index < g_args.buffer_size)
                    {
                        IEC_UINT val                = *g_args.int_output[m->plc_index];
                        payload[m->byte_offset]     = val & 0xFF;
                        payload[m->byte_offset + 1] = (val >> 8) & 0xFF;
                    }
                    break;
                }
                case CAN_DATA_U32:
                case CAN_DATA_I32:
                case CAN_DATA_F32:
                {
                    if (g_args.dint_output && m->plc_index < g_args.buffer_size)
                    {
                        IEC_UDINT val               = *g_args.dint_output[m->plc_index];
                        payload[m->byte_offset]     = val & 0xFF;
                        payload[m->byte_offset + 1] = (val >> 8) & 0xFF;
                        payload[m->byte_offset + 2] = (val >> 16) & 0xFF;
                        payload[m->byte_offset + 3] = (val >> 24) & 0xFF;
                    }
                    break;
                }
                case CAN_DATA_U64:
                case CAN_DATA_I64:
                case CAN_DATA_F64:
                {
                    if (g_args.lint_output && m->plc_index < g_args.buffer_size)
                    {
                        IEC_ULINT val = *g_args.lint_output[m->plc_index];
                        for (int byte = 0; byte < 8; byte++)
                            payload[m->byte_offset + byte] = (uint8_t)(val >> (byte * 8));
                    }
                    break;
                }
                default:
                    break;
                }
            }

            bool should_send = false;
            if (frame->trigger == CAN_TRIGGER_CYCLIC)
            {
                if (!frame->has_sent_once ||
                    (now_ms - frame->last_send_time_ms >= frame->cycle_time_ms))
                {
                    should_send = true;
                }
            }
            else if (frame->trigger == CAN_TRIGGER_ON_CHANGE)
            {
                if (!frame->has_sent_once || memcmp(payload, frame->prev_payload, frame->dlc) != 0)
                {
                    should_send = true;
                }
            }

            if (should_send)
            {
                int ret =
                    can_socket_write(fd, frame->can_id, frame->eff, false, frame->dlc, payload);
                if (ret == 0)
                {
                    g_tx_count++;
                    g_tx_count_by_iface[iface_idx]++;
                    g_last_tx_ms[iface_idx]  = now_ms;
                    frame->last_send_time_ms = now_ms;
                    memcpy(frame->prev_payload, payload, 8);
                    frame->has_sent_once = true;
                }
                else
                {
                    g_tx_errors++;
                    g_tx_errors_by_iface[iface_idx]++;
                }
            }
        }
    }

    if (g_args.image_unlock)
        g_args.image_unlock();
}

void stop_loop(void)
{
    plugin_logger_info(&g_logger, "Stopping CAN Plugin...");
    if (g_rx_running)
    {
        g_rx_running = false;
        pthread_join(g_rx_thread, NULL);
    }

    for (int i = 0; i < g_config.interface_count; i++)
    {
        if (g_can_fds[i] >= 0)
        {
            can_socket_close(g_can_fds[i]);
            g_can_fds[i] = -1;
        }
    }
    g_can_fd_count = 0;
    memset(g_can_fds, -1, sizeof(g_can_fds));
}

void cleanup(void)
{
    plugin_logger_info(&g_logger, "Cleaning up CAN Plugin...");
    stop_loop();

    can_config_free(&g_config);
}

int get_stats(char *out, size_t out_size)
{
    if (!out || out_size == 0)
        return -1;

    char iface_entries[2048] = {0};
    size_t iface_pos         = 0;
    int have_iface_data      = 0;

    for (int iface_idx = 0; iface_idx < g_config.interface_count && iface_idx < MAX_CAN_INTERFACES;
         iface_idx++)
    {
        can_interface_config_t *iface = &g_config.interfaces[iface_idx];
        if (!iface || !iface->hardware.interface[0])
        {
            continue;
        }

        const char *iface_name = iface->hardware.interface;
        uint64_t now_ms        = get_time_ms();
        bool port_ok           = g_can_fds[iface_idx] >= 0 && can_netlink_is_up(iface_name);
        bool data_ok           = port_ok && interface_data_ok(iface, iface_idx, now_ms);
        int n = snprintf(iface_entries + iface_pos, sizeof(iface_entries) - iface_pos,
                         "%s\"%s\":{\"label\":\"CAN Bus (%s)\",\"fields\":["
                         "{\"label\":\"Interface\",\"value\":\"%s\"},"
                         "{\"label\":\"RX Frames\",\"value\":%llu},"
                         "{\"label\":\"TX Frames\",\"value\":%llu},"
                         "{\"label\":\"RX Errors\",\"value\":%llu},"
                         "{\"label\":\"TX Errors\",\"value\":%llu},"
                         "{\"label\":\"Port OK\",\"value\":%s},"
                         "{\"label\":\"Data OK\",\"value\":%s}"
                         "]}",
                         have_iface_data ? "," : "", iface_name, iface_name, iface_name,
                         (unsigned long long)g_rx_count_by_iface[iface_idx],
                         (unsigned long long)g_tx_count_by_iface[iface_idx],
                         (unsigned long long)g_rx_errors_by_iface[iface_idx],
                         (unsigned long long)g_tx_errors_by_iface[iface_idx],
                         port_ok ? "true" : "false", data_ok ? "true" : "false");

        if (n < 0 || (size_t)n >= sizeof(iface_entries) - iface_pos)
        {
            break;
        }

        iface_pos += (size_t)n;
        have_iface_data = 1;
    }

    if (!have_iface_data)
    {
        const char *fallback = "can0";
        snprintf(out, out_size,
                 "{\"label\":\"CAN Bus\",\"fields\":["
                 "{\"label\":\"Interface\",\"value\":\"%s\"},"
                 "{\"label\":\"RX Frames\",\"value\":%llu},"
                 "{\"label\":\"TX Frames\",\"value\":%llu},"
                 "{\"label\":\"RX Errors\",\"value\":%llu},"
                 "{\"label\":\"TX Errors\",\"value\":%llu}"
                 "]}",
                 fallback, (unsigned long long)g_rx_count, (unsigned long long)g_tx_count,
                 (unsigned long long)g_rx_errors, (unsigned long long)g_tx_errors);
        return 0;
    }

    snprintf(out, out_size, "{\"label\":\"CAN Bus\",\"interfaces\":{%s}}", iface_entries);
    return 0;
}

int execute_command(const char *command_json, char *response, size_t response_size)
{
    (void)command_json;
    if (response && response_size > 0)
    {
        snprintf(response, response_size, "{\"status\":\"ok\",\"plugin\":\"can\"}");
    }
    return 0;
}
