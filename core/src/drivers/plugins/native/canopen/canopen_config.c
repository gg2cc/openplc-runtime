/**
 * @file canopen_config.c
 * @brief Runtime config parser for editor-generated CANopen configuration.
 */

#include "canopen_config.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static uint16_t parse_u16_literal(const char *value)
{
    if (!value)
        return 0;
    if (strncmp(value, "0x", 2) == 0 || strncmp(value, "0X", 2) == 0)
    {
        return (uint16_t)strtoul(value + 2, NULL, 16);
    }
    return (uint16_t)strtoul(value, NULL, 10);
}

static void copy_string(char *dst, size_t dst_size, const char *src)
{
    if (!dst || dst_size == 0)
        return;
    if (!src)
    {
        dst[0] = '\0';
        return;
    }

    size_t len = strlen(src);
    if (len >= dst_size)
    {
        len = dst_size - 1U;
    }
    if (len > 0U)
    {
        memcpy(dst, src, len);
    }
    dst[len] = '\0';
}

static void parse_od_entry(cJSON *item, canopen_od_entry_t *entry, plugin_logger_t *logger)
{
    if (!item || !entry)
        return;
    memset(entry, 0, sizeof(*entry));

    cJSON *name = cJSON_GetObjectItem(item, "name");
    if (cJSON_IsString(name) && name->valuestring)
    {
        copy_string(entry->name, sizeof(entry->name), name->valuestring);
    }

    cJSON *index = cJSON_GetObjectItem(item, "index");
    if (cJSON_IsString(index) && index->valuestring)
    {
        entry->index = parse_u16_literal(index->valuestring);
    }
    else if (cJSON_IsNumber(index))
    {
        entry->index = (uint16_t)index->valueint;
    }

    cJSON *sub_index = cJSON_GetObjectItem(item, "sub_index");
    if (cJSON_IsString(sub_index) && sub_index->valuestring)
    {
        entry->sub_index = (uint8_t)parse_u16_literal(sub_index->valuestring);
    }
    else if (cJSON_IsNumber(sub_index))
    {
        entry->sub_index = (uint8_t)sub_index->valueint;
    }

    cJSON *data_type = cJSON_GetObjectItem(item, "data_type");
    if (cJSON_IsString(data_type) && data_type->valuestring)
    {
        copy_string(entry->data_type, sizeof(entry->data_type), data_type->valuestring);
    }

    cJSON *access = cJSON_GetObjectItem(item, "access");
    if (cJSON_IsString(access) && access->valuestring)
    {
        copy_string(entry->access, sizeof(entry->access), access->valuestring);
    }

    cJSON *default_value = cJSON_GetObjectItem(item, "default_value");
    if (cJSON_IsNumber(default_value))
    {
        entry->default_value = (int32_t)default_value->valueint;
    }
    else if (cJSON_IsString(default_value) && default_value->valuestring)
    {
        entry->default_value = (int32_t)strtol(default_value->valuestring, NULL, 0);
    }

    cJSON *description = cJSON_GetObjectItem(item, "description");
    if (cJSON_IsString(description) && description->valuestring)
    {
        copy_string(entry->description, sizeof(entry->description), description->valuestring);
    }

    cJSON *pdo_map = cJSON_GetObjectItem(item, "pdo_map");
    if (cJSON_IsString(pdo_map) && pdo_map->valuestring)
    {
        copy_string(entry->pdo_map, sizeof(entry->pdo_map), pdo_map->valuestring);
    }

    if (logger)
    {
        plugin_logger_debug(logger, "OD entry: name=%s index=0x%04X sub=%u type=%s pdo_map=%s",
                            entry->name, entry->index, entry->sub_index,
                            entry->data_type[0] ? entry->data_type : "u32",
                            entry->pdo_map[0] ? entry->pdo_map : "none");
    }
}

static void parse_binding_fields(cJSON *item, char *plc_address, size_t plc_address_size,
                                 char *direction, size_t direction_size)
{
    if (!item || !plc_address || !direction)
        return;

    cJSON *plc_address_field = cJSON_GetObjectItem(item, "plc_address");
    if (cJSON_IsString(plc_address_field) && plc_address_field->valuestring)
    {
        copy_string(plc_address, plc_address_size, plc_address_field->valuestring);
    }

    cJSON *direction_field = cJSON_GetObjectItem(item, "direction");
    if (cJSON_IsString(direction_field) && direction_field->valuestring)
    {
        copy_string(direction, direction_size, direction_field->valuestring);
    }

    cJSON *binding = cJSON_GetObjectItem(item, "binding");
    if (cJSON_IsObject(binding))
    {
        cJSON *binding_direction = cJSON_GetObjectItem(binding, "direction");
        if (cJSON_IsString(binding_direction) && binding_direction->valuestring &&
            direction[0] == '\0')
        {
            copy_string(direction, direction_size, binding_direction->valuestring);
        }

        cJSON *binding_address = cJSON_GetObjectItem(binding, "iec_address");
        if (!plc_address[0] && cJSON_IsString(binding_address) && binding_address->valuestring)
        {
            copy_string(plc_address, plc_address_size, binding_address->valuestring);
        }

        cJSON *binding_plc_address = cJSON_GetObjectItem(binding, "plc_address");
        if (!plc_address[0] && cJSON_IsString(binding_plc_address) &&
            binding_plc_address->valuestring)
        {
            copy_string(plc_address, plc_address_size, binding_plc_address->valuestring);
        }

        cJSON *binding_iec_address = cJSON_GetObjectItem(binding, "iecAddress");
        if (!plc_address[0] && cJSON_IsString(binding_iec_address) &&
            binding_iec_address->valuestring)
        {
            copy_string(plc_address, plc_address_size, binding_iec_address->valuestring);
        }
    }

    cJSON *iec_address_field = cJSON_GetObjectItem(item, "iec_address");
    if (!plc_address[0] && cJSON_IsString(iec_address_field) && iec_address_field->valuestring)
    {
        copy_string(plc_address, plc_address_size, iec_address_field->valuestring);
    }

    cJSON *iec_address_camel = cJSON_GetObjectItem(item, "iecAddress");
    if (!plc_address[0] && cJSON_IsString(iec_address_camel) && iec_address_camel->valuestring)
    {
        copy_string(plc_address, plc_address_size, iec_address_camel->valuestring);
    }
}

static void parse_pdo_mapping(cJSON *item, canopen_pdo_mapping_t *mapping)
{
    if (!item || !mapping)
        return;
    memset(mapping, 0, sizeof(*mapping));

    cJSON *name = cJSON_GetObjectItem(item, "name");
    if (cJSON_IsString(name) && name->valuestring)
    {
        copy_string(mapping->name, sizeof(mapping->name), name->valuestring);
    }

    cJSON *index = cJSON_GetObjectItem(item, "index");
    if (cJSON_IsString(index) && index->valuestring)
    {
        mapping->index = parse_u16_literal(index->valuestring);
    }
    else if (cJSON_IsNumber(index))
    {
        mapping->index = (uint16_t)index->valueint;
    }

    cJSON *sub_index = cJSON_GetObjectItem(item, "sub_index");
    if (cJSON_IsString(sub_index) && sub_index->valuestring)
    {
        mapping->sub_index = (uint8_t)parse_u16_literal(sub_index->valuestring);
    }
    else if (cJSON_IsNumber(sub_index))
    {
        mapping->sub_index = (uint8_t)sub_index->valueint;
    }

    cJSON *bit_length = cJSON_GetObjectItem(item, "bit_length");
    if (cJSON_IsNumber(bit_length))
    {
        mapping->bit_length = (uint16_t)bit_length->valueint;
    }

    cJSON *data_type = cJSON_GetObjectItem(item, "data_type");
    if (cJSON_IsString(data_type) && data_type->valuestring)
    {
        copy_string(mapping->data_type, sizeof(mapping->data_type), data_type->valuestring);
    }

    parse_binding_fields(item, mapping->plc_address, sizeof(mapping->plc_address),
                         mapping->direction, sizeof(mapping->direction));
    mapping->bound = (mapping->plc_address[0] != '\0' && mapping->direction[0] != '\0');
}

static void parse_sdo_entry(cJSON *item, canopen_sdo_entry_t *entry, plugin_logger_t *logger)
{
    if (!item || !entry)
        return;
    memset(entry, 0, sizeof(*entry));

    cJSON *name = cJSON_GetObjectItem(item, "name");
    if (cJSON_IsString(name) && name->valuestring)
    {
        copy_string(entry->name, sizeof(entry->name), name->valuestring);
    }

    cJSON *index = cJSON_GetObjectItem(item, "index");
    if (cJSON_IsString(index) && index->valuestring)
    {
        entry->index = parse_u16_literal(index->valuestring);
    }
    else if (cJSON_IsNumber(index))
    {
        entry->index = (uint16_t)index->valueint;
    }

    cJSON *sub_index = cJSON_GetObjectItem(item, "sub_index");
    if (cJSON_IsString(sub_index) && sub_index->valuestring)
    {
        entry->sub_index = (uint8_t)parse_u16_literal(sub_index->valuestring);
    }
    else if (cJSON_IsNumber(sub_index))
    {
        entry->sub_index = (uint8_t)sub_index->valueint;
    }

    cJSON *data_type = cJSON_GetObjectItem(item, "data_type");
    if (cJSON_IsString(data_type) && data_type->valuestring)
    {
        copy_string(entry->data_type, sizeof(entry->data_type), data_type->valuestring);
    }

    cJSON *access = cJSON_GetObjectItem(item, "access");
    if (cJSON_IsString(access) && access->valuestring)
    {
        copy_string(entry->access, sizeof(entry->access), access->valuestring);
    }

    cJSON *default_value = cJSON_GetObjectItem(item, "default_value");
    if (cJSON_IsNumber(default_value))
    {
        entry->default_value = (int32_t)default_value->valueint;
    }
    else if (cJSON_IsString(default_value) && default_value->valuestring)
    {
        entry->default_value = (int32_t)strtol(default_value->valuestring, NULL, 0);
    }

    cJSON *description = cJSON_GetObjectItem(item, "description");
    if (cJSON_IsString(description) && description->valuestring)
    {
        copy_string(entry->description, sizeof(entry->description), description->valuestring);
    }

    parse_binding_fields(item, entry->plc_address, sizeof(entry->plc_address), entry->direction,
                         sizeof(entry->direction));
    entry->bound = (entry->plc_address[0] != '\0' && entry->direction[0] != '\0');

    if (logger)
    {
        plugin_logger_debug(
            logger, "SDO entry: name=%s index=0x%04X sub=%u data_type=%s bound=%s plc=%s dir=%s",
            entry->name, entry->index, entry->sub_index,
            entry->data_type[0] ? entry->data_type : "u32", entry->bound ? "yes" : "no",
            entry->plc_address[0] ? entry->plc_address : "none",
            entry->direction[0] ? entry->direction : "none");
    }
}

static void parse_pdo(cJSON *items, int *count, canopen_pdo_t *pdo_list, const char *pdo_name,
                      plugin_logger_t *logger)
{
    if (!cJSON_IsArray(items))
    {
        *count = 0;
        return;
    }

    int total = cJSON_GetArraySize(items);
    *count    = 0;

    for (int i = 0; i < total && i < MAX_CANOPEN_PDO_COUNT; i++)
    {
        cJSON *item = cJSON_GetArrayItem(items, i);
        if (!item)
            continue;

        canopen_pdo_t *pdo = &pdo_list[*count];
        memset(pdo, 0, sizeof(*pdo));

        cJSON *index = cJSON_GetObjectItem(item, "index");
        if (cJSON_IsString(index) && index->valuestring)
        {
            pdo->index = parse_u16_literal(index->valuestring);
        }
        else if (cJSON_IsNumber(index))
        {
            pdo->index = (uint16_t)index->valueint;
        }

        cJSON *sub_index = cJSON_GetObjectItem(item, "sub_index");
        if (cJSON_IsString(sub_index) && sub_index->valuestring)
        {
            pdo->sub_index = (uint8_t)parse_u16_literal(sub_index->valuestring);
        }
        else if (cJSON_IsNumber(sub_index))
        {
            pdo->sub_index = (uint8_t)sub_index->valueint;
        }

        cJSON *name = cJSON_GetObjectItem(item, "name");
        if (cJSON_IsString(name) && name->valuestring)
        {
            copy_string(pdo->name, sizeof(pdo->name), name->valuestring);
        }
        else
        {
            snprintf(pdo->name, sizeof(pdo->name), "%s_%d", pdo_name, *count + 1);
        }

        cJSON *mapping = cJSON_GetObjectItem(item, "mapping");
        if (cJSON_IsArray(mapping))
        {
            int map_total      = cJSON_GetArraySize(mapping);
            pdo->mapping_count = 0;
            for (int map_idx = 0; map_idx < map_total && map_idx < MAX_CANOPEN_PDO_MAPPING;
                 map_idx++)
            {
                cJSON *entry = cJSON_GetArrayItem(mapping, map_idx);
                if (!entry)
                    continue;
                parse_pdo_mapping(entry, &pdo->mapping[pdo->mapping_count]);
                pdo->mapping_count++;
            }
        }

        if (logger)
        {
            plugin_logger_debug(logger, "PDO parsed: %s index=0x%04X mappings=%d", pdo_name,
                                pdo->index, pdo->mapping_count);
        }
        (*count)++;
    }
}

static void parse_bus(cJSON *item, canopen_bus_config_t *bus, plugin_logger_t *logger)
{
    if (!item || !bus)
        return;
    memset(bus, 0, sizeof(*bus));
    bus->enabled = true;
    bus->bitrate = 500000;

    cJSON *name = cJSON_GetObjectItem(item, "name");
    if (cJSON_IsString(name) && name->valuestring)
    {
        copy_string(bus->name, sizeof(bus->name), name->valuestring);
    }

    cJSON *interface = cJSON_GetObjectItem(item, "interface");
    if (cJSON_IsString(interface) && interface->valuestring)
    {
        copy_string(bus->interface, sizeof(bus->interface), interface->valuestring);
    }

    cJSON *enabled = cJSON_GetObjectItem(item, "enabled");
    if (cJSON_IsBool(enabled))
    {
        bus->enabled = cJSON_IsTrue(enabled);
    }

    cJSON *node_id = cJSON_GetObjectItem(item, "node_id");
    if (cJSON_IsString(node_id) && node_id->valuestring)
    {
        bus->node_id = (uint16_t)parse_u16_literal(node_id->valuestring);
    }
    else if (cJSON_IsNumber(node_id))
    {
        bus->node_id = (uint16_t)node_id->valueint;
    }

    cJSON *bitrate = cJSON_GetObjectItem(item, "bitrate");
    if (cJSON_IsString(bitrate) && bitrate->valuestring)
    {
        bus->bitrate = (uint32_t)strtoul(bitrate->valuestring, NULL, 0);
    }
    else if (cJSON_IsNumber(bitrate))
    {
        bus->bitrate = (uint32_t)bitrate->valueint;
    }

    cJSON *heartbeat = cJSON_GetObjectItem(item, "heartbeat_ms");
    if (cJSON_IsNumber(heartbeat))
    {
        bus->heartbeat_ms = (uint32_t)heartbeat->valueint;
    }

    cJSON *sync_period = cJSON_GetObjectItem(item, "sync_period_ms");
    if (cJSON_IsNumber(sync_period))
    {
        bus->sync_period_ms = (uint32_t)sync_period->valueint;
    }

    cJSON *od_entries = cJSON_GetObjectItem(item, "od_entries");
    if (cJSON_IsArray(od_entries))
    {
        int total = cJSON_GetArraySize(od_entries);
        for (int i = 0; i < total && i < MAX_CANOPEN_OD_ENTRIES; i++)
        {
            cJSON *entry = cJSON_GetArrayItem(od_entries, i);
            if (!entry)
                continue;
            parse_od_entry(entry, &bus->od_entries[bus->od_entry_count], logger);
            bus->od_entry_count++;
        }
    }

    cJSON *tpdo = cJSON_GetObjectItem(item, "tpdo");
    parse_pdo(tpdo, &bus->tpdo_count, bus->tpdo, "tpdo", logger);

    cJSON *rpdo = cJSON_GetObjectItem(item, "rpdo");
    parse_pdo(rpdo, &bus->rpdo_count, bus->rpdo, "rpdo", logger);

    cJSON *sdo = cJSON_GetObjectItem(item, "sdo");
    if (cJSON_IsArray(sdo))
    {
        int total = cJSON_GetArraySize(sdo);
        for (int i = 0; i < total && i < MAX_CANOPEN_SDO_COUNT; i++)
        {
            cJSON *entry = cJSON_GetArrayItem(sdo, i);
            if (!entry)
                continue;
            parse_sdo_entry(entry, &bus->sdo[bus->sdo_count], logger);
            bus->sdo_count++;
        }
    }

    if (bus->name[0] == '\0' && bus->interface[0] != '\0')
    {
        copy_string(bus->name, sizeof(bus->name), bus->interface);
    }

    if (logger)
    {
        plugin_logger_debug(logger,
                            "CANopen bus parsed: name=%s interface=%s node_id=%u bitrate=%u "
                            "od_entries=%d tpdo=%d rpdo=%d sdo=%d",
                            bus->name, bus->interface, bus->node_id, bus->bitrate,
                            bus->od_entry_count, bus->tpdo_count, bus->rpdo_count, bus->sdo_count);
    }
}

void canopen_config_init_defaults(canopen_config_t *config)
{
    if (!config)
        return;
    memset(config, 0, sizeof(*config));
    config->bus_count = 0;
}

int canopen_config_parse(const char *json_path, canopen_config_t *config, plugin_logger_t *logger)
{
    if (!json_path || !config)
        return -1;
    canopen_config_init_defaults(config);

    FILE *f = fopen(json_path, "rb");
    if (!f)
    {
        if (logger)
            plugin_logger_warn(logger, "CANopen config file not found: %s", json_path);
        return 0;
    }

    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0)
    {
        fclose(f);
        return 0;
    }

    char *buf = (char *)malloc((size_t)len + 1U);
    if (!buf)
    {
        fclose(f);
        if (logger)
            plugin_logger_error(logger, "Out of memory reading CANopen config");
        return -1;
    }

    size_t read_bytes = fread(buf, 1, (size_t)len, f);
    fclose(f);
    buf[read_bytes] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root)
    {
        if (logger)
            plugin_logger_error(logger, "Failed to parse CANopen config JSON: %s", json_path);
        return -1;
    }

    cJSON *buses = cJSON_GetObjectItem(root, "buses");
    if (!cJSON_IsArray(buses))
    {
        cJSON_Delete(root);
        if (logger)
            plugin_logger_error(logger, "CANopen config must contain a buses[] array");
        return -1;
    }

    int bus_total = cJSON_GetArraySize(buses);
    if (bus_total > MAX_CANOPEN_BUSES)
    {
        bus_total = MAX_CANOPEN_BUSES;
        if (logger)
            plugin_logger_warn(logger, "Truncating CANopen buses to %d", MAX_CANOPEN_BUSES);
    }

    for (int i = 0; i < bus_total; i++)
    {
        cJSON *item = cJSON_GetArrayItem(buses, i);
        if (!item)
            continue;
        parse_bus(item, &config->buses[config->bus_count], logger);
        if (config->buses[config->bus_count].enabled)
        {
            config->bus_count++;
        }
        else if (logger)
        {
            plugin_logger_debug(logger, "Skipping disabled CANopen bus: %s",
                                config->buses[config->bus_count].name);
        }
    }

    cJSON_Delete(root);
    return 0;
}

void canopen_config_free(canopen_config_t *config)
{
    if (!config)
        return;
    canopen_config_init_defaults(config);
}
