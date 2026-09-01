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

static uint16_t canopen_data_type_bit_length(const char *data_type)
{
    if (!data_type || data_type[0] == '\0' || strcasecmp(data_type, "bool") == 0)
        return 1U;
    if (strcasecmp(data_type, "i8") == 0 || strcasecmp(data_type, "u8") == 0)
        return 8U;
    if (strcasecmp(data_type, "i16") == 0 || strcasecmp(data_type, "u16") == 0)
        return 16U;
    if (strcasecmp(data_type, "i32") == 0 || strcasecmp(data_type, "u32") == 0 ||
        strcasecmp(data_type, "f32") == 0)
        return 32U;
    if (strcasecmp(data_type, "i64") == 0 || strcasecmp(data_type, "u64") == 0 ||
        strcasecmp(data_type, "f64") == 0)
        return 64U;
    return 0U;
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

    cJSON *data_type = cJSON_GetObjectItem(item, "data_type");
    if (cJSON_IsString(data_type) && data_type->valuestring)
    {
        copy_string(mapping->data_type, sizeof(mapping->data_type), data_type->valuestring);
    }

    parse_binding_fields(item, mapping->plc_address, sizeof(mapping->plc_address),
                         mapping->direction, sizeof(mapping->direction));

    if (mapping->data_type[0] == '\0')
    {
        if (strncasecmp(mapping->plc_address, "%IX", 3) == 0 ||
            strncasecmp(mapping->plc_address, "%QX", 3) == 0)
            copy_string(mapping->data_type, sizeof(mapping->data_type), "bool");
        else if (strncasecmp(mapping->plc_address, "%IB", 3) == 0 ||
                 strncasecmp(mapping->plc_address, "%QB", 3) == 0)
            copy_string(mapping->data_type, sizeof(mapping->data_type), "u8");
        else if (strncasecmp(mapping->plc_address, "%IW", 3) == 0 ||
                 strncasecmp(mapping->plc_address, "%QW", 3) == 0)
            copy_string(mapping->data_type, sizeof(mapping->data_type), "u16");
        else if (strncasecmp(mapping->plc_address, "%ID", 3) == 0 ||
                 strncasecmp(mapping->plc_address, "%QD", 3) == 0)
            copy_string(mapping->data_type, sizeof(mapping->data_type), "u32");
        else if (strncasecmp(mapping->plc_address, "%IL", 3) == 0 ||
                 strncasecmp(mapping->plc_address, "%QL", 3) == 0)
            copy_string(mapping->data_type, sizeof(mapping->data_type), "u64");
    }
    mapping->bit_length = canopen_data_type_bit_length(mapping->data_type);
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

    if (logger)
    {
        plugin_logger_debug(logger, "SDO entry: name=%s index=0x%04X sub=%u data_type=%s",
                            entry->name, entry->index, entry->sub_index,
                            entry->data_type[0] ? entry->data_type : "u32");
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

static void parse_slave(cJSON *item, canopen_slave_config_t *slave, plugin_logger_t *logger)
{
    if (!item || !slave)
        return;
    memset(slave, 0, sizeof(*slave));
    slave->enabled = true;

    cJSON *name = cJSON_GetObjectItem(item, "name");
    if (cJSON_IsString(name) && name->valuestring)
    {
        copy_string(slave->name, sizeof(slave->name), name->valuestring);
    }

    cJSON *enabled = cJSON_GetObjectItem(item, "enabled");
    if (cJSON_IsBool(enabled))
    {
        slave->enabled = cJSON_IsTrue(enabled);
    }

    cJSON *node_id = cJSON_GetObjectItem(item, "node_id");
    if (cJSON_IsString(node_id) && node_id->valuestring)
    {
        slave->node_id = (uint16_t)parse_u16_literal(node_id->valuestring);
    }
    else if (cJSON_IsNumber(node_id))
    {
        slave->node_id = (uint16_t)node_id->valueint;
    }

    snprintf(slave->protection_mode, sizeof(slave->protection_mode), "node_guarding");
    cJSON *protection_mode = cJSON_GetObjectItem(item, "protection_mode");
    if (cJSON_IsString(protection_mode) && protection_mode->valuestring &&
        (strcmp(protection_mode->valuestring, "node_guarding") == 0 ||
         strcmp(protection_mode->valuestring, "heartbeat_producer") == 0))
    {
        copy_string(slave->protection_mode, sizeof(slave->protection_mode),
                    protection_mode->valuestring);
    }

    slave->node_guard_time_ms = 500U;
    cJSON *node_guard_time_ms = cJSON_GetObjectItem(item, "node_guard_time_ms");
    if (cJSON_IsNumber(node_guard_time_ms) && node_guard_time_ms->valueint > 0)
    {
        slave->node_guard_time_ms = (uint32_t)node_guard_time_ms->valueint;
    }

    slave->node_guard_life_factor = 3U;
    cJSON *node_guard_life_factor = cJSON_GetObjectItem(item, "node_guard_life_factor");
    if (cJSON_IsNumber(node_guard_life_factor) && node_guard_life_factor->valueint > 0)
    {
        slave->node_guard_life_factor = (uint8_t)node_guard_life_factor->valueint;
    }

    slave->heartbeat_producer_time_ms = 200U;
    cJSON *heartbeat_producer_time_ms =
        cJSON_GetObjectItem(item, "heartbeat_producer_time_ms");
    if (cJSON_IsNumber(heartbeat_producer_time_ms) && heartbeat_producer_time_ms->valueint > 0)
    {
        slave->heartbeat_producer_time_ms = (uint32_t)heartbeat_producer_time_ms->valueint;
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
            parse_od_entry(entry, &slave->od_entries[slave->od_entry_count], logger);
            slave->od_entry_count++;
        }
    }

    cJSON *tpdo = cJSON_GetObjectItem(item, "tpdo");
    parse_pdo(tpdo, &slave->tpdo_count, slave->tpdo, "tpdo", logger);

    cJSON *rpdo = cJSON_GetObjectItem(item, "rpdo");
    parse_pdo(rpdo, &slave->rpdo_count, slave->rpdo, "rpdo", logger);

    cJSON *sdo = cJSON_GetObjectItem(item, "sdo");
    if (cJSON_IsArray(sdo))
    {
        int total = cJSON_GetArraySize(sdo);
        for (int i = 0; i < total && i < MAX_CANOPEN_SDO_COUNT; i++)
        {
            cJSON *entry = cJSON_GetArrayItem(sdo, i);
            if (!entry)
                continue;
            parse_sdo_entry(entry, &slave->sdo[slave->sdo_count], logger);
            slave->sdo_count++;
        }
    }

    if (slave->name[0] == '\0' && slave->node_id > 0U)
    {
        snprintf(slave->name, sizeof(slave->name), "slave_%u", (unsigned int)slave->node_id);
    }

    if (logger)
    {
        plugin_logger_debug(logger,
                            "Slave parsed: name=%s node_id=%u od_entries=%d tpdo=%d rpdo=%d sdo=%d",
                            slave->name, slave->node_id, slave->od_entry_count, slave->tpdo_count,
                            slave->rpdo_count, slave->sdo_count);
    }
}

static void aggregate_bus_counts_from_slaves(canopen_bus_config_t *bus, plugin_logger_t *logger)
{
    if (!bus)
        return;

    bus->od_entry_count = 0;
    bus->tpdo_count     = 0;
    bus->rpdo_count     = 0;
    bus->sdo_count      = 0;

    for (int i = 0; i < bus->slave_count; i++)
    {
        const canopen_slave_config_t *slave = &bus->slaves[i];
        if (!slave->enabled)
        {
            continue;
        }
        bus->od_entry_count += slave->od_entry_count;
        bus->tpdo_count += slave->tpdo_count;
        bus->rpdo_count += slave->rpdo_count;
        bus->sdo_count += slave->sdo_count;
    }

    if (logger)
    {
        plugin_logger_debug(logger, "Bus aggregate: slaves=%d od_entries=%d tpdo=%d rpdo=%d sdo=%d",
                            bus->slave_count, bus->od_entry_count, bus->tpdo_count, bus->rpdo_count,
                            bus->sdo_count);
    }
}

static void parse_bus(cJSON *item, canopen_bus_config_t *bus, plugin_logger_t *logger)
{
    if (!item || !bus)
        return;
    memset(bus, 0, sizeof(*bus));
    bus->enabled         = true;
    bus->bitrate         = 500000;
    bus->sjw             = 1U;
    bus->sample_point    = 0.875;
    bus->restart_ms      = 100U;
    bus->triple_sampling = false;
    bus->auto_bringup    = true;
    bus->local_node_id   = 127U;

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

    cJSON *local_node_id = cJSON_GetObjectItem(item, "local_node_id");
    if (cJSON_IsString(local_node_id) && local_node_id->valuestring)
    {
        bus->local_node_id = (uint16_t)parse_u16_literal(local_node_id->valuestring);
    }
    else if (cJSON_IsNumber(local_node_id))
    {
        bus->local_node_id = (uint16_t)local_node_id->valueint;
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

    cJSON *sjw = cJSON_GetObjectItem(item, "sjw");
    if (cJSON_IsString(sjw) && sjw->valuestring)
    {
        bus->sjw = (uint32_t)strtoul(sjw->valuestring, NULL, 0);
    }
    else if (cJSON_IsNumber(sjw))
    {
        bus->sjw = (uint32_t)sjw->valueint;
    }

    cJSON *sample_point = cJSON_GetObjectItem(item, "sample_point");
    if (cJSON_IsNumber(sample_point))
    {
        bus->sample_point = sample_point->valuedouble;
    }
    else if (cJSON_IsString(sample_point) && sample_point->valuestring)
    {
        bus->sample_point = strtod(sample_point->valuestring, NULL);
    }

    cJSON *restart_ms = cJSON_GetObjectItem(item, "restart_ms");
    if (cJSON_IsString(restart_ms) && restart_ms->valuestring)
    {
        bus->restart_ms = (uint32_t)strtoul(restart_ms->valuestring, NULL, 0);
    }
    else if (cJSON_IsNumber(restart_ms))
    {
        bus->restart_ms = (uint32_t)restart_ms->valueint;
    }

    cJSON *triple_sampling = cJSON_GetObjectItem(item, "triple_sampling");
    if (cJSON_IsBool(triple_sampling))
    {
        bus->triple_sampling = cJSON_IsTrue(triple_sampling);
    }

    cJSON *auto_bringup = cJSON_GetObjectItem(item, "auto_bringup");
    if (cJSON_IsBool(auto_bringup))
    {
        bus->auto_bringup = cJSON_IsTrue(auto_bringup);
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

    cJSON *slaves = cJSON_GetObjectItem(item, "slaves");
    if (cJSON_IsArray(slaves))
    {
        int total = cJSON_GetArraySize(slaves);
        for (int i = 0; i < total && i < MAX_CANOPEN_SLAVES; i++)
        {
            cJSON *slave_item = cJSON_GetArrayItem(slaves, i);
            if (!slave_item)
                continue;
            parse_slave(slave_item, &bus->slaves[bus->slave_count], logger);
            bus->slave_count++;
        }
    }

    if (bus->name[0] == '\0' && bus->interface[0] != '\0')
    {
        copy_string(bus->name, sizeof(bus->name), bus->interface);
    }

    aggregate_bus_counts_from_slaves(bus, logger);

    if (logger)
    {
        plugin_logger_debug(logger,
                            "CANopen bus parsed: name=%s interface=%s local_node_id=%u bitrate=%u "
                            "od_entries=%d tpdo=%d rpdo=%d sdo=%d slaves=%d",
                            bus->name, bus->interface, bus->local_node_id, bus->bitrate,
                            bus->od_entry_count, bus->tpdo_count, bus->rpdo_count, bus->sdo_count,
                            bus->slave_count);
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
