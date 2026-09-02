/**
 * @file can_config.c
 * @brief Implementation of CAN configuration parser using cJSON
 */

#include "can_config.h"
#include "cJSON.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static int parse_data_type(const char *str)
{
    if (!str)
        return -1;
    const char *types[] = {"bool", "u8",  "i8",  "u16", "i16", "u32",
                           "i32",  "u64", "i64", "f32", "f64"};
    for (int i = 0; i < 11; i++)
        if (strcasecmp(str, types[i]) == 0)
            return i;
    return -1;
}

static int data_type_width(can_data_type_t type)
{
    return type == CAN_DATA_BOOL || type == CAN_DATA_U8 || type == CAN_DATA_I8    ? 1
           : type == CAN_DATA_U16 || type == CAN_DATA_I16                         ? 2
           : type == CAN_DATA_U32 || type == CAN_DATA_I32 || type == CAN_DATA_F32 ? 4
                                                                                  : 8;
}

static int parse_plc_address(const char *address, bool input, can_data_type_t type,
                             int *journal_type, int *index, int *bit)
{
    if (!address || !journal_type || !index || !bit || address[0] != '%' ||
        (input ? strncasecmp(address + 1, "I", 1) : strncasecmp(address + 1, "Q", 1)) != 0)
        return -1;
    char kind = (char)address[2];
    if (kind >= 'a' && kind <= 'z')
        kind -= 'a' - 'A';
    const char *digits = address + 3;
    char *end          = NULL;
    long parsed        = strtol(digits, &end, 10);
    if (end == digits || parsed < 0 || parsed > INT_MAX)
        return -1;
    int parsed_bit = -1;
    if (*end == '.')
    {
        parsed_bit = (int)strtol(end + 1, &end, 10);
        if (end == address || parsed_bit < 0 || parsed_bit > 7)
            return -1;
    }
    if (*end != '\0')
        return -1;
    char expected = type == CAN_DATA_BOOL        ? 'X'
                    : data_type_width(type) == 1 ? 'B'
                    : data_type_width(type) == 2 ? 'W'
                    : data_type_width(type) == 4 ? 'D'
                                                 : 'L';
    if (kind != expected || (type == CAN_DATA_BOOL ? parsed_bit < 0 : parsed_bit >= 0))
        return -1;
    *journal_type = input ? (type == CAN_DATA_BOOL        ? 0
                             : data_type_width(type) == 1 ? 3
                             : data_type_width(type) == 2 ? 5
                             : data_type_width(type) == 4 ? 8
                                                          : 11)
                          : (type == CAN_DATA_BOOL        ? 1
                             : data_type_width(type) == 1 ? 4
                             : data_type_width(type) == 2 ? 6
                             : data_type_width(type) == 4 ? 9
                                                          : 12);
    *index        = (int)parsed;
    *bit          = parsed_bit < 0 ? 0 : parsed_bit;
    return 0;
}

static void parse_status_config(cJSON *obj, can_interface_config_t *iface)
{
    cJSON *port_status = cJSON_GetObjectItem(obj, "port_status_plc_address");
    if (cJSON_IsString(port_status) && port_status->valuestring)
    {
        snprintf(iface->port_status_plc_address, sizeof(iface->port_status_plc_address), "%s",
                 port_status->valuestring);
    }

    cJSON *data_status = cJSON_GetObjectItem(obj, "data_status_plc_address");
    if (cJSON_IsString(data_status) && data_status->valuestring)
    {
        snprintf(iface->data_status_plc_address, sizeof(iface->data_status_plc_address), "%s",
                 data_status->valuestring);
    }

    cJSON *timeout = cJSON_GetObjectItem(obj, "data_status_timeout_ms");
    iface->data_status_timeout_ms =
        (cJSON_IsNumber(timeout) && timeout->valueint >= 100) ? (uint32_t)timeout->valueint : 3000U;
}

static uint32_t parse_hex_or_dec(const char *str)
{
    if (!str)
        return 0;
    if (strncmp(str, "0x", 2) == 0 || strncmp(str, "0X", 2) == 0)
    {
        return (uint32_t)strtoul(str + 2, NULL, 16);
    }
    return (uint32_t)strtoul(str, NULL, 10);
}

static int parse_byte_order(cJSON *item, can_byte_order_t *byte_order)
{
    if (!byte_order)
        return -1;
    *byte_order = CAN_BYTE_ORDER_LITTLE;
    if (!item)
        return 0;
    if (!cJSON_IsString(item) || !item->valuestring)
        return -1;
    if (strcasecmp(item->valuestring, "little") == 0)
        return 0;
    if (strcasecmp(item->valuestring, "big") == 0)
    {
        *byte_order = CAN_BYTE_ORDER_BIG;
        return 0;
    }
    return -1;
}

void can_config_init_defaults(can_config_t *config)
{
    if (!config)
        return;
    memset(config, 0, sizeof(can_config_t));
    config->interface_count = 0;
    config->interfaces      = NULL;
}

static void init_hardware_defaults(can_hardware_config_t *hw)
{
    if (!hw)
        return;
    memset(hw, 0, sizeof(*hw));
    snprintf(hw->interface, sizeof(hw->interface), "can0");
    hw->bitrate         = 500000;
    hw->sjw             = 1;
    hw->sample_point    = 0.875;
    hw->restart_ms      = 100;
    hw->auto_bringup    = true;
    hw->triple_sampling = false;
}

static int parse_hardware(cJSON *hw_item, can_hardware_config_t *hw, plugin_logger_t *logger)
{
    if (!hw_item)
        return 0;

    cJSON *iface = cJSON_GetObjectItem(hw_item, "interface");
    if (cJSON_IsString(iface) && iface->valuestring)
    {
        snprintf(hw->interface, sizeof(hw->interface), "%s", iface->valuestring);
    }

    cJSON *bitrate = cJSON_GetObjectItem(hw_item, "bitrate");
    if (cJSON_IsNumber(bitrate) && bitrate->valueint > 0)
    {
        hw->bitrate = (uint32_t)bitrate->valueint;
    }

    cJSON *sjw = cJSON_GetObjectItem(hw_item, "sjw");
    if (cJSON_IsNumber(sjw) && sjw->valueint > 0)
    {
        hw->sjw = (uint32_t)sjw->valueint;
    }

    cJSON *sp = cJSON_GetObjectItem(hw_item, "sample_point");
    if (cJSON_IsNumber(sp) && sp->valuedouble > 0.0 && sp->valuedouble < 1.0)
    {
        hw->sample_point = sp->valuedouble;
    }

    cJSON *restart = cJSON_GetObjectItem(hw_item, "restart_ms");
    if (cJSON_IsNumber(restart) && restart->valueint >= 0)
    {
        hw->restart_ms = (uint32_t)restart->valueint;
    }

    cJSON *triple_sampling = cJSON_GetObjectItem(hw_item, "triple_sampling");
    if (cJSON_IsBool(triple_sampling))
    {
        hw->triple_sampling = cJSON_IsTrue(triple_sampling);
    }

    cJSON *auto_bringup = cJSON_GetObjectItem(hw_item, "auto_bringup");
    if (cJSON_IsBool(auto_bringup))
    {
        hw->auto_bringup = cJSON_IsTrue(auto_bringup);
    }

    plugin_logger_info(logger,
                       "Hardware Config: iface=%s, bitrate=%u, sjw=%u, sample_point=%.3f, "
                       "restart_ms=%u, auto_bringup=%s, triple_sampling=%s",
                       hw->interface, hw->bitrate, hw->sjw, hw->sample_point, hw->restart_ms,
                       hw->auto_bringup ? "true" : "false", hw->triple_sampling ? "true" : "false");
    return 0;
}

static int parse_mappings(cJSON *mappings_arr, can_mapping_t *mappings, int *count, bool input,
                          uint8_t dlc, plugin_logger_t *logger)
{
    (void)logger;
    *count = 0;
    if (!cJSON_IsArray(mappings_arr))
        return 0;

    int num = cJSON_GetArraySize(mappings_arr);
    for (int i = 0; i < num && i < MAX_CAN_MAPPINGS_PER_FRAME; i++)
    {
        cJSON *item = cJSON_GetArrayItem(mappings_arr, i);
        if (!item)
            continue;

        cJSON *bo = cJSON_GetObjectItem(item, "byte_offset");
        cJSON *dt = cJSON_GetObjectItem(item, "data_type");
        cJSON *pa = cJSON_GetObjectItem(item, "plc_address");

        if (!cJSON_IsNumber(bo) || !cJSON_IsString(dt) || !cJSON_IsString(pa) || bo->valueint < 0 ||
            bo->valueint + 1 > dlc)
            return -1;
        int type_val     = parse_data_type(dt->valuestring);
        int width        = type_val < 0 ? 0 : data_type_width((can_data_type_t)type_val);
        int journal_type = 0, index = 0, bit = 0;
        if (type_val < 0 || bo->valueint + width > dlc ||
            parse_plc_address(pa->valuestring, input, (can_data_type_t)type_val, &journal_type,
                              &index, &bit) != 0)
            return -1;
        for (int previous = 0; previous < *count; previous++)
        {
            can_mapping_t *other = &mappings[previous];
            if (bo->valueint < other->byte_offset + data_type_width(other->data_type) &&
                bo->valueint + width > other->byte_offset &&
                !(type_val == CAN_DATA_BOOL && other->data_type == CAN_DATA_BOOL &&
                  bo->valueint == other->byte_offset && bit != other->plc_bit))
                return -1;
        }
        can_mapping_t *m = &mappings[*count];
        m->byte_offset   = bo->valueint;
        m->data_type     = (can_data_type_t)type_val;
        m->journal_type  = journal_type;
        m->plc_index     = index;
        m->plc_bit       = bit;
        (*count)++;
    }
    if (num > MAX_CAN_MAPPINGS_PER_FRAME)
        return -1;
    return 0;
}

static int parse_rx_frames(cJSON *rx_arr, can_interface_config_t *iface, plugin_logger_t *logger)
{
    if (!iface || !cJSON_IsArray(rx_arr))
        return 0;
    int num = cJSON_GetArraySize(rx_arr);
    if (num <= 0)
        return 0;

    iface->rx_frames = (can_rx_frame_config_t *)calloc(num, sizeof(can_rx_frame_config_t));
    if (!iface->rx_frames)
    {
        plugin_logger_error(logger, "Memory allocation failed for RX frames");
        return -1;
    }
    iface->rx_frame_count = num;

    for (int i = 0; i < num; i++)
    {
        cJSON *item                  = cJSON_GetArrayItem(rx_arr, i);
        can_rx_frame_config_t *frame = &iface->rx_frames[i];

        cJSON *id_item = cJSON_GetObjectItem(item, "can_id");
        if (cJSON_IsString(id_item))
        {
            frame->can_id = parse_hex_or_dec(id_item->valuestring);
        }
        else if (cJSON_IsNumber(id_item))
        {
            frame->can_id = (uint32_t)id_item->valueint;
        }

        cJSON *eff = cJSON_GetObjectItem(item, "eff");
        if (cJSON_IsBool(eff))
            frame->eff = cJSON_IsTrue(eff);

        cJSON *rtr = cJSON_GetObjectItem(item, "rtr");
        if (cJSON_IsBool(rtr))
            frame->rtr = cJSON_IsTrue(rtr);

        cJSON *dlc = cJSON_GetObjectItem(item, "dlc");
        frame->dlc = (dlc && cJSON_IsNumber(dlc)) ? (uint8_t)dlc->valueint : 8;

        if (parse_byte_order(cJSON_GetObjectItem(item, "byte_order"), &frame->byte_order) != 0)
        {
            plugin_logger_error(logger, "Invalid RX frame byte_order at index %d", i);
            return -1;
        }

        cJSON *mappings = cJSON_GetObjectItem(item, "mappings");
        if (frame->dlc > 8 || parse_mappings(mappings, frame->mappings, &frame->mapping_count, true,
                                             frame->dlc, logger) != 0)
            return -1;

        plugin_logger_info(
            logger, "Parsed RX Frame #%d on %s: ID=0x%X, eff=%d, dlc=%d, mappings=%d", i,
            iface->hardware.interface, frame->can_id, frame->eff, frame->dlc, frame->mapping_count);
    }
    return 0;
}

static int parse_tx_frames(cJSON *tx_arr, can_interface_config_t *iface, plugin_logger_t *logger)
{
    if (!iface || !cJSON_IsArray(tx_arr))
        return 0;
    int num = cJSON_GetArraySize(tx_arr);
    if (num <= 0)
        return 0;

    iface->tx_frames = (can_tx_frame_config_t *)calloc(num, sizeof(can_tx_frame_config_t));
    if (!iface->tx_frames)
    {
        plugin_logger_error(logger, "Memory allocation failed for TX frames");
        return -1;
    }
    iface->tx_frame_count = num;

    for (int i = 0; i < num; i++)
    {
        cJSON *item                  = cJSON_GetArrayItem(tx_arr, i);
        can_tx_frame_config_t *frame = &iface->tx_frames[i];

        cJSON *id_item = cJSON_GetObjectItem(item, "can_id");
        if (cJSON_IsString(id_item))
        {
            frame->can_id = parse_hex_or_dec(id_item->valuestring);
        }
        else if (cJSON_IsNumber(id_item))
        {
            frame->can_id = (uint32_t)id_item->valueint;
        }

        cJSON *eff = cJSON_GetObjectItem(item, "eff");
        if (cJSON_IsBool(eff))
            frame->eff = cJSON_IsTrue(eff);

        cJSON *dlc = cJSON_GetObjectItem(item, "dlc");
        frame->dlc = (dlc && cJSON_IsNumber(dlc)) ? (uint8_t)dlc->valueint : 8;

        if (parse_byte_order(cJSON_GetObjectItem(item, "byte_order"), &frame->byte_order) != 0)
        {
            plugin_logger_error(logger, "Invalid TX frame byte_order at index %d", i);
            return -1;
        }

        cJSON *trig = cJSON_GetObjectItem(item, "trigger");
        if (cJSON_IsString(trig) && strcasecmp(trig->valuestring, "on_change") == 0)
        {
            frame->trigger = CAN_TRIGGER_ON_CHANGE;
        }
        else
        {
            frame->trigger = CAN_TRIGGER_CYCLIC;
        }

        cJSON *cycle         = cJSON_GetObjectItem(item, "cycle_time_ms");
        frame->cycle_time_ms = (cycle && cJSON_IsNumber(cycle)) ? (uint32_t)cycle->valueint : 10;

        cJSON *mappings = cJSON_GetObjectItem(item, "mappings");
        if (frame->dlc > 8 || parse_mappings(mappings, frame->mappings, &frame->mapping_count,
                                             false, frame->dlc, logger) != 0)
            return -1;

        plugin_logger_info(logger,
                           "Parsed TX Frame #%d on %s: ID=0x%X, eff=%d, dlc=%d, trigger=%s, "
                           "cycle_ms=%u, mappings=%d",
                           i, iface->hardware.interface, frame->can_id, frame->eff, frame->dlc,
                           frame->trigger == CAN_TRIGGER_ON_CHANGE ? "on_change" : "cyclic",
                           frame->cycle_time_ms, frame->mapping_count);
    }
    return 0;
}

static int parse_interface_object(cJSON *obj, can_interface_config_t *iface,
                                  plugin_logger_t *logger)
{
    if (!obj || !iface)
        return -1;
    memset(iface, 0, sizeof(*iface));
    init_hardware_defaults(&iface->hardware);

    cJSON *hw = cJSON_GetObjectItem(obj, "hardware_config");
    if (!hw)
    {
        hw = obj;
    }
    parse_hardware(hw, &iface->hardware, logger);
    parse_status_config(obj, iface);

    cJSON *rx = cJSON_GetObjectItem(obj, "rx_frames");
    if (parse_rx_frames(rx, iface, logger) != 0)
    {
        return -1;
    }

    cJSON *tx = cJSON_GetObjectItem(obj, "tx_frames");
    if (parse_tx_frames(tx, iface, logger) != 0)
    {
        return -1;
    }

    return 0;
}

int can_config_parse(const char *json_path, can_config_t *config, plugin_logger_t *logger)
{
    if (!json_path || !config)
        return -1;
    can_config_init_defaults(config);

    FILE *f = fopen(json_path, "rb");
    if (!f)
    {
        plugin_logger_warn(logger, "Config file not found: %s (using defaults)", json_path);
        return 0;
    }
    else
    {
        plugin_logger_info(logger, "Parsing CAN config from: %s", json_path);
    }

    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (len <= 0)
    {
        fclose(f);
        plugin_logger_warn(logger, "Config file empty: %s", json_path);
        return 0;
    }

    char *buf = (char *)malloc(len + 1);
    if (!buf)
    {
        fclose(f);
        plugin_logger_error(logger, "Out of memory reading config file");
        return -1;
    }

    size_t read_bytes = fread(buf, 1, len, f);
    fclose(f);
    buf[read_bytes] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    if (!root)
    {
        plugin_logger_error(logger, "Failed to parse JSON config in %s", json_path);
        return -1;
    }

    cJSON *interfaces = cJSON_GetObjectItem(root, "interfaces");
    if (!cJSON_IsArray(interfaces) || cJSON_GetArraySize(interfaces) <= 0)
    {
        cJSON_Delete(root);
        plugin_logger_error(logger, "CAN config must contain an interfaces[] array");
        return -1;
    }

    int num = cJSON_GetArraySize(interfaces);
    if (num > MAX_CAN_INTERFACES)
    {
        num = MAX_CAN_INTERFACES;
        plugin_logger_warn(logger, "CAN config contains more than %d interfaces; truncating to %d",
                           MAX_CAN_INTERFACES, MAX_CAN_INTERFACES);
    }

    config->interface_count = num;
    config->interfaces      = (can_interface_config_t *)calloc(num, sizeof(can_interface_config_t));
    if (!config->interfaces)
    {
        cJSON_Delete(root);
        plugin_logger_error(logger, "Out of memory allocating CAN interfaces");
        return -1;
    }

    for (int i = 0; i < num; i++)
    {
        cJSON *item = cJSON_GetArrayItem(interfaces, i);
        if (parse_interface_object(item, &config->interfaces[i], logger) != 0)
        {
            cJSON_Delete(root);
            plugin_logger_error(logger, "Failed to parse CAN interface #%d", i);
            return -1;
        }
    }

    cJSON_Delete(root);
    return 0;
}

void can_config_free(can_config_t *config)
{
    if (!config)
        return;

    if (config->interfaces)
    {
        for (int i = 0; i < config->interface_count; i++)
        {
            if (config->interfaces[i].rx_frames)
            {
                free(config->interfaces[i].rx_frames);
                config->interfaces[i].rx_frames = NULL;
            }
            if (config->interfaces[i].tx_frames)
            {
                free(config->interfaces[i].tx_frames);
                config->interfaces[i].tx_frames = NULL;
            }
        }
        free(config->interfaces);
        config->interfaces = NULL;
    }

    config->interface_count = 0;
}
