/**
 * @file canopen_config.h
 * @brief Parsed editor-generated CANopen configuration for the native runtime.
 */

#ifndef CANOPEN_CONFIG_H
#define CANOPEN_CONFIG_H

#include "plugin_logger.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_CANOPEN_BUSES 8
#define MAX_CANOPEN_OD_ENTRIES 128
#define MAX_CANOPEN_PDO_COUNT 16
#define MAX_CANOPEN_PDO_MAPPING 32
#define MAX_CANOPEN_SDO_COUNT 32
#define MAX_CANOPEN_NAME_LEN 64
#define MAX_CANOPEN_SLAVES 64

    typedef struct
    {
        char name[MAX_CANOPEN_NAME_LEN];
        uint16_t index;
        uint8_t sub_index;
        char data_type[16];
        char access[8];
        int32_t default_value;
        char description[MAX_CANOPEN_NAME_LEN];
        char pdo_map[8];
    } canopen_od_entry_t;

    typedef struct
    {
        char name[MAX_CANOPEN_NAME_LEN];
        uint16_t index;
        uint8_t sub_index;
        uint16_t bit_length;
        char data_type[16];
        char plc_address[32];
        char direction[8];
        bool bound;
    } canopen_pdo_mapping_t;

    typedef struct
    {
        char name[MAX_CANOPEN_NAME_LEN];
        uint16_t index;
        uint8_t sub_index;
        int mapping_count;
        canopen_pdo_mapping_t mapping[MAX_CANOPEN_PDO_MAPPING];
    } canopen_pdo_t;

    typedef struct
    {
        char name[MAX_CANOPEN_NAME_LEN];
        uint16_t index;
        uint8_t sub_index;
        char data_type[16];
        int32_t default_value;
        char description[MAX_CANOPEN_NAME_LEN];
    } canopen_sdo_entry_t;

    typedef struct
    {
        char name[MAX_CANOPEN_NAME_LEN];
        bool enabled;
        uint16_t node_id;
        int od_entry_count;
        canopen_od_entry_t od_entries[MAX_CANOPEN_OD_ENTRIES];
        int tpdo_count;
        canopen_pdo_t tpdo[MAX_CANOPEN_PDO_COUNT];
        int rpdo_count;
        canopen_pdo_t rpdo[MAX_CANOPEN_PDO_COUNT];
        int sdo_count;
        canopen_sdo_entry_t sdo[MAX_CANOPEN_SDO_COUNT];
    } canopen_slave_config_t;

    typedef struct
    {
        char name[MAX_CANOPEN_NAME_LEN];
        char interface[32];
        bool enabled;
        uint16_t local_node_id;
        uint32_t bitrate;
        uint32_t sjw;
        double sample_point;
        uint32_t restart_ms;
        bool triple_sampling;
        bool auto_bringup;
        uint32_t heartbeat_ms;
        uint32_t sync_period_ms;
        int od_entry_count;
        canopen_od_entry_t od_entries[MAX_CANOPEN_OD_ENTRIES];
        int tpdo_count;
        canopen_pdo_t tpdo[MAX_CANOPEN_PDO_COUNT];
        int rpdo_count;
        canopen_pdo_t rpdo[MAX_CANOPEN_PDO_COUNT];
        int sdo_count;
        canopen_sdo_entry_t sdo[MAX_CANOPEN_SDO_COUNT];
        int slave_count;
        canopen_slave_config_t slaves[MAX_CANOPEN_SLAVES];
    } canopen_bus_config_t;

    typedef struct
    {
        int bus_count;
        canopen_bus_config_t buses[MAX_CANOPEN_BUSES];
    } canopen_config_t;

    void canopen_config_init_defaults(canopen_config_t *config);
    int canopen_config_parse(const char *json_path, canopen_config_t *config,
                             plugin_logger_t *logger);
    void canopen_config_free(canopen_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_CONFIG_H */
