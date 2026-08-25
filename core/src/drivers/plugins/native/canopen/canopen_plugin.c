/**
 * @file canopen_plugin.c
 * @brief Native CANopen plugin adapter for OpenPLC runtime.
 *
 * This adapter is intentionally lightweight: it parses the editor-generated
 * CANopen JSON configuration and exposes a runtime-ready structure for the
 * CANopenNode stack integration layer to consume. The actual CANopenNode
 * transport / OD initialization can then be added without changing the plugin
 * lifecycle contract.
 */

#include "canopen_plugin.h"
#include "canopen_config.h"
#include "plugin_logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static plugin_runtime_args_t g_args;
static plugin_logger_t g_logger;
static canopen_config_t g_config;

static void write_json_status(char *out, size_t out_size)
{
    if (!out || out_size == 0) return;

    snprintf(out, out_size,
             "{\n"
             "  \"bus_count\": %d,\n"
             "  \"enabled_buses\": %d\n"
             "}",
             g_config.bus_count, g_config.bus_count);
}

int init(void *args)
{
    if (!args) return -1;
    memcpy(&g_args, args, sizeof(plugin_runtime_args_t));

    if (!plugin_logger_init(&g_logger, "CANOPEN_PLUGIN", &g_args)) {
        return -1;
    }

    plugin_logger_info(&g_logger, "Initializing CANopen runtime adapter");

    const char *cfg_path = g_args.plugin_specific_config_file_path;
    if (!cfg_path || !cfg_path[0]) {
        plugin_logger_info(&g_logger, "No CANopen config file path configured; plugin remains idle");
        canopen_config_init_defaults(&g_config);
        return 0;
    }

    if (access(cfg_path, F_OK) != 0) {
        plugin_logger_warn(&g_logger, "CANopen config file not found: %s", cfg_path);
        canopen_config_init_defaults(&g_config);
        return 0;
    }

    int rc = canopen_config_parse(cfg_path, &g_config, &g_logger);
    if (rc != 0) {
        plugin_logger_error(&g_logger, "Failed to parse CANopen configuration");
        return -1;
    }

    plugin_logger_info(&g_logger, "CANopen runtime adapter initialized with %d bus(es)", g_config.bus_count);
    return 0;
}

int start_loop(void)
{
    plugin_logger_info(&g_logger, "Starting CANopen runtime adapter");
    if (g_config.bus_count == 0) {
        plugin_logger_warn(&g_logger, "No active CANopen bus configured");
        return 0;
    }

    for (int i = 0; i < g_config.bus_count; i++) {
        const canopen_bus_config_t *bus = &g_config.buses[i];
        plugin_logger_info(&g_logger,
                           "Bus[%d]: name=%s interface=%s node_id=%u bitrate=%u od_entries=%d tpdo=%d rpdo=%d",
                           i, bus->name, bus->interface, bus->node_id, bus->bitrate,
                           bus->od_entry_count, bus->tpdo_count, bus->rpdo_count);
    }
    return 0;
}

void cycle_start(void)
{
}

void cycle_end(void)
{
}

void stop_loop(void)
{
    plugin_logger_info(&g_logger, "Stopping CANopen runtime adapter");
}

void cleanup(void)
{
    plugin_logger_info(&g_logger, "Cleaning up CANopen runtime adapter");
    canopen_config_free(&g_config);
}

int get_stats(char *out, size_t out_size)
{
    if (!out || out_size == 0) return -1;
    write_json_status(out, out_size);
    return 0;
}

int execute_command(const char *command_json, char *response, size_t response_size)
{
    if (!response || response_size == 0) return -1;
    response[0] = '\0';

    if (!command_json || !command_json[0]) {
        snprintf(response, response_size, "{\"status\":\"empty_command\"}");
        return 0;
    }

    snprintf(response, response_size,
             "{\"status\":\"accepted\",\"bus_count\":%d,\"command\":%s}",
             g_config.bus_count, command_json);
    return 0;
}
