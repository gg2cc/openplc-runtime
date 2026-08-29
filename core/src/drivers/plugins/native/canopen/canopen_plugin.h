/**
 * @file canopen_plugin.h
 * @brief OpenPLC native CANopen plugin adapter interface.
 */

#ifndef CANOPEN_PLUGIN_H
#define CANOPEN_PLUGIN_H

#include "plugin_types.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int init(void *args);
int start_loop(void);
void cycle_start(void);
void cycle_end(void);
void stop_loop(void);
void cleanup(void);
int get_stats(char *out, size_t out_size);
int execute_command(const char *command_json, char *response, size_t response_size);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_PLUGIN_H */
