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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>

static plugin_runtime_args_t g_args;
static plugin_logger_t g_logger;
static can_config_t g_config;
static int g_can_fds[MAX_CAN_INTERFACES];
static int g_can_fd_count = 0;

static pthread_t g_rx_thread;
static volatile bool g_rx_running = false;

/* Performance counters for get_stats */
static uint64_t g_rx_count = 0;
static uint64_t g_tx_count = 0;
static uint64_t g_rx_errors = 0;
static uint64_t g_tx_errors = 0;

static uint64_t get_time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000 + (uint64_t)(tv.tv_usec / 1000);
}

static void *can_rx_thread_proc(void *arg)
{
    (void)arg;
    plugin_logger_info(&g_logger, "CAN RX thread started");

    uint32_t can_id = 0;
    bool eff = false;
    bool rtr = false;
    uint8_t dlc = 0;
    uint8_t payload[8];

    while (g_rx_running) {
        bool any_fd_open = false;
        for (int iface_idx = 0; iface_idx < g_can_fd_count; iface_idx++) {
            int fd = g_can_fds[iface_idx];
            if (fd < 0) {
                continue;
            }
            any_fd_open = true;

            int res = can_socket_read(fd, &can_id, &eff, &rtr, &dlc, payload);
            if (res < 0) {
                continue;
            }

            g_rx_count++;

            can_interface_config_t *iface = &g_config.interfaces[iface_idx];
            for (int i = 0; i < iface->rx_frame_count; i++) {
                can_rx_frame_config_t *frame = &iface->rx_frames[i];
                if (frame->can_id == can_id && frame->eff == eff) {
                    for (int j = 0; j < frame->mapping_count; j++) {
                        can_mapping_t *m = &frame->mappings[j];
                        if (m->byte_offset >= dlc) continue;

                        switch (m->iec_type) {
                            case 0: { /* BOOL_INPUT */
                                int bit_val = (payload[m->byte_offset] >> m->iec_bit) & 0x01;
                                if (g_args.journal_write_bool) {
                                    g_args.journal_write_bool(0, m->iec_index, m->iec_bit, bit_val);
                                }
                                break;
                            }
                            case 3: { /* BYTE_INPUT */
                                uint8_t val = payload[m->byte_offset];
                                if (g_args.journal_write_byte) {
                                    g_args.journal_write_byte(3, m->iec_index, val);
                                }
                                break;
                            }
                            case 5: { /* INT_INPUT */
                                if (m->byte_offset + 1 < dlc) {
                                    uint16_t val = (uint16_t)payload[m->byte_offset] |
                                                  ((uint16_t)payload[m->byte_offset + 1] << 8);
                                    if (g_args.journal_write_int) {
                                        g_args.journal_write_int(5, m->iec_index, (int)val);
                                    }
                                }
                                break;
                            }
                            case 8: { /* DINT_INPUT */
                                if (m->byte_offset + 3 < dlc) {
                                    uint32_t val = (uint32_t)payload[m->byte_offset] |
                                                  ((uint32_t)payload[m->byte_offset + 1] << 8) |
                                                  ((uint32_t)payload[m->byte_offset + 2] << 16) |
                                                  ((uint32_t)payload[m->byte_offset + 3] << 24);
                                    if (g_args.journal_write_dint) {
                                        g_args.journal_write_dint(8, m->iec_index, val);
                                    }
                                }
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

        if (!any_fd_open) {
            usleep(100000);
        }
    }

    plugin_logger_info(&g_logger, "CAN RX thread stopped");
    return NULL;
}

int init(void *args)
{
    if (!args) return -1;

    /* Copy runtime args by value as required */
    memcpy(&g_args, args, sizeof(plugin_runtime_args_t));

    /* Initialize logger */
    if (!plugin_logger_init(&g_logger, "CAN_PLUGIN", &g_args)) {
        return -1;
    }

    plugin_logger_info(&g_logger, "Initializing CAN Plugin...");

    /* Parse configuration. If no editor-side CAN config is provided,
     * do not auto-init the interface; the plugin remains inactive. */
    const char *cfg_path = g_args.plugin_specific_config_file_path;
    if (!cfg_path || !cfg_path[0]) {
        plugin_logger_info(&g_logger, "No CAN config provided; skipping CAN initialization");
        return 0;
    }

    if (access(cfg_path, F_OK) != 0) {
        plugin_logger_info(&g_logger, "CAN config file %s not found; skipping CAN initialization",
                           cfg_path);
        return 0;
    }

    if (can_config_parse(cfg_path, &g_config, &g_logger) != 0) {
        plugin_logger_error(&g_logger, "Failed to parse CAN configuration file");
        return -1;
    }

    plugin_logger_info(&g_logger, "CAN Plugin initialized successfully");
    return 0;
}

int start_loop(void)
{
    if (g_config.interface_count <= 0) {
        plugin_logger_info(&g_logger, "No CAN interface configured; skipping CAN start");
        return 0;
    }

    plugin_logger_info(&g_logger, "Starting CAN Plugin for %d interfaces...", g_config.interface_count);

    memset(g_can_fds, -1, sizeof(g_can_fds));
    g_can_fd_count = 0;

    for (int iface_idx = 0; iface_idx < g_config.interface_count; iface_idx++) {
        can_interface_config_t *iface = &g_config.interfaces[iface_idx];
        if (!iface->hardware.interface[0]) {
            continue;
        }

        can_netlink_configure_and_up(&iface->hardware, &g_logger);

        int fd = can_socket_open(iface->hardware.interface, &g_logger);
        if (fd < 0) {
            plugin_logger_warn(&g_logger, "Could not open SocketCAN device %s; plugin running in degraded mode",
                               iface->hardware.interface);
            continue;
        }

        g_can_fds[g_can_fd_count++] = fd;
    }

    if (g_can_fd_count == 0) {
        plugin_logger_warn(&g_logger, "No CAN socket opened; plugin will remain idle");
        return 0;
    }

    if (!g_rx_running) {
        g_rx_running = true;
        if (pthread_create(&g_rx_thread, NULL, can_rx_thread_proc, NULL) != 0) {
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
    if (g_can_fd_count == 0) return;

    uint64_t now_ms = get_time_ms();

    if (g_args.image_lock) g_args.image_lock();

    for (int iface_idx = 0; iface_idx < g_config.interface_count; iface_idx++) {
        can_interface_config_t *iface = &g_config.interfaces[iface_idx];
        int fd = g_can_fds[iface_idx];
        if (fd < 0 || iface->tx_frame_count == 0) continue;

        for (int i = 0; i < iface->tx_frame_count; i++) {
            can_tx_frame_config_t *frame = &iface->tx_frames[i];
            uint8_t payload[8] = {0};

            for (int j = 0; j < frame->mapping_count; j++) {
                can_mapping_t *m = &frame->mappings[j];
                if (m->byte_offset >= 8) continue;

                switch (m->iec_type) {
                    case 1: { /* BOOL_OUTPUT */
                        if (g_args.bool_output && m->iec_index < g_args.buffer_size) {
                            IEC_BOOL val = *g_args.bool_output[m->iec_index][m->iec_bit];
                            if (val) {
                                payload[m->byte_offset] |= (1 << m->iec_bit);
                            } else {
                                payload[m->byte_offset] &= ~(1 << m->iec_bit);
                            }
                        }
                        break;
                    }
                    case 4: { /* BYTE_OUTPUT */
                        if (g_args.byte_output && m->iec_index < g_args.buffer_size) {
                            IEC_BYTE val = *g_args.byte_output[m->iec_index];
                            payload[m->byte_offset] = val;
                        }
                        break;
                    }
                    case 6: { /* INT_OUTPUT */
                        if (g_args.int_output && m->iec_index < g_args.buffer_size) {
                            IEC_UINT val = *g_args.int_output[m->iec_index];
                            payload[m->byte_offset] = val & 0xFF;
                            if (m->byte_offset + 1 < 8) {
                                payload[m->byte_offset + 1] = (val >> 8) & 0xFF;
                            }
                        }
                        break;
                    }
                    case 9: { /* DINT_OUTPUT */
                        if (g_args.dint_output && m->iec_index < g_args.buffer_size) {
                            IEC_UDINT val = *g_args.dint_output[m->iec_index];
                            payload[m->byte_offset] = val & 0xFF;
                            if (m->byte_offset + 1 < 8) payload[m->byte_offset + 1] = (val >> 8) & 0xFF;
                            if (m->byte_offset + 2 < 8) payload[m->byte_offset + 2] = (val >> 16) & 0xFF;
                            if (m->byte_offset + 3 < 8) payload[m->byte_offset + 3] = (val >> 24) & 0xFF;
                        }
                        break;
                    }
                    default:
                        break;
                }
            }

            bool should_send = false;
            if (frame->trigger == CAN_TRIGGER_CYCLIC) {
                if (!frame->has_sent_once || (now_ms - frame->last_send_time_ms >= frame->cycle_time_ms)) {
                    should_send = true;
                }
            } else if (frame->trigger == CAN_TRIGGER_ON_CHANGE) {
                if (!frame->has_sent_once || memcmp(payload, frame->prev_payload, frame->dlc) != 0) {
                    should_send = true;
                }
            }

            if (should_send) {
                int ret = can_socket_write(fd, frame->can_id, frame->eff, false, frame->dlc, payload);
                if (ret == 0) {
                    g_tx_count++;
                    frame->last_send_time_ms = now_ms;
                    memcpy(frame->prev_payload, payload, 8);
                    frame->has_sent_once = true;
                } else {
                    g_tx_errors++;
                }
            }
        }
    }

    if (g_args.image_unlock) g_args.image_unlock();
}

void stop_loop(void)
{
    plugin_logger_info(&g_logger, "Stopping CAN Plugin...");
    if (g_rx_running) {
        g_rx_running = false;
        pthread_join(g_rx_thread, NULL);
    }

    for (int i = 0; i < g_can_fd_count; i++) {
        if (g_can_fds[i] >= 0) {
            can_socket_close(g_can_fds[i]);
            g_can_fds[i] = -1;
        }
    }
    g_can_fd_count = 0;
}

void cleanup(void)
{
    plugin_logger_info(&g_logger, "Cleaning up CAN Plugin...");
    stop_loop();

    can_config_free(&g_config);
}

int get_stats(char *out, size_t out_size)
{
    if (!out || out_size == 0) return -1;

    const char *iface = "can0";
    if (g_config.interface_count > 0 && g_config.interfaces[0].hardware.interface[0]) {
        iface = g_config.interfaces[0].hardware.interface;
    }

    snprintf(out, out_size,
             "{\"label\":\"CAN Bus (%s)\",\"fields\":["
             "{\"label\":\"Interface\",\"value\":\"%s\"},"
             "{\"label\":\"RX Frames\",\"value\":%llu},"
             "{\"label\":\"TX Frames\",\"value\":%llu},"
             "{\"label\":\"RX Errors\",\"value\":%llu},"
             "{\"label\":\"TX Errors\",\"value\":%llu}"
             "]}",
             iface,
             iface,
             (unsigned long long)g_rx_count,
             (unsigned long long)g_tx_count,
             (unsigned long long)g_rx_errors,
             (unsigned long long)g_tx_errors);
    return 0;
}

int execute_command(const char *command_json, char *response, size_t response_size)
{
    (void)command_json;
    if (response && response_size > 0) {
        snprintf(response, response_size, "{\"status\":\"ok\",\"plugin\":\"can\"}");
    }
    return 0;
}
