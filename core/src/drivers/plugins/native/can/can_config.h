/**
 * @file can_config.h
 * @brief Configuration parser header for OpenPLC Native CAN Plugin
 */

#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

#include "plugin_logger.h"
#include "plugin_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_CAN_IFNAME_LEN 32
#define MAX_CAN_MAPPINGS_PER_FRAME 16
#define MAX_CAN_INTERFACES 8

    typedef enum
    {
        CAN_TRIGGER_CYCLIC    = 0,
        CAN_TRIGGER_ON_CHANGE = 1
    } can_tx_trigger_t;

    typedef enum
    {
        CAN_DATA_BOOL = 0,
        CAN_DATA_U8,
        CAN_DATA_I8,
        CAN_DATA_U16,
        CAN_DATA_I16,
        CAN_DATA_U32,
        CAN_DATA_I32,
        CAN_DATA_U64,
        CAN_DATA_I64,
        CAN_DATA_F32,
        CAN_DATA_F64
    } can_data_type_t;

    typedef struct
    {
        int byte_offset; /* Byte offset inside CAN payload (0..7) */
        can_data_type_t data_type;
        int journal_type;
        int plc_index;
        int plc_bit;
    } can_mapping_t;

    typedef struct
    {
        uint32_t can_id;
        bool eff;    /* Extended Frame Format (29-bit ID) */
        bool rtr;    /* Remote Transmission Request */
        uint8_t dlc; /* Data Length Code (0..8) */
        int mapping_count;
        can_mapping_t mappings[MAX_CAN_MAPPINGS_PER_FRAME];
    } can_rx_frame_config_t;

    typedef struct
    {
        uint32_t can_id;
        bool eff; /* Extended Frame Format */
        uint8_t dlc;
        can_tx_trigger_t trigger;
        uint32_t cycle_time_ms;
        uint64_t last_send_time_ms;
        uint8_t prev_payload[8];
        bool has_sent_once;
        int mapping_count;
        can_mapping_t mappings[MAX_CAN_MAPPINGS_PER_FRAME];
    } can_tx_frame_config_t;

    typedef struct
    {
        char interface[MAX_CAN_IFNAME_LEN];
        uint32_t bitrate;
        uint32_t sjw;
        double sample_point;
        uint32_t restart_ms;
        bool triple_sampling;
        bool auto_bringup;
    } can_hardware_config_t;

    typedef struct
    {
        can_hardware_config_t hardware;
        char port_status_plc_address[32];
        char data_status_plc_address[32];
        uint32_t data_status_timeout_ms;
        int rx_frame_count;
        can_rx_frame_config_t *rx_frames;
        int tx_frame_count;
        can_tx_frame_config_t *tx_frames;
    } can_interface_config_t;

    typedef struct
    {
        int interface_count;
        can_interface_config_t *interfaces;
    } can_config_t;

    /**
     * @brief Initialize configuration structure to defaults
     */
    void can_config_init_defaults(can_config_t *config);

    /**
     * @brief Parse CAN configuration from a JSON file
     *
     * @param json_path Path to the JSON configuration file
     * @param config Output config structure
     * @param logger Pointer to initialized plugin logger
     * @return 0 on success, negative value on error
     */
    int can_config_parse(const char *json_path, can_config_t *config, plugin_logger_t *logger);

    /**
     * @brief Free allocated memory in configuration structure
     */
    void can_config_free(can_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* CAN_CONFIG_H */
