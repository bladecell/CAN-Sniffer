#include "middleware.hpp"

#include <sys/_intsup.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "obd2.hpp"
#include "sd_card.hpp"
#include "string"
#include "supervisor.hpp"
#include "utilities.h"

cJSON* single_pid_def_get(uint16_t pid)
{
    // Create a local struct to hold the snapshot
    PIDDefinitionData def;

    if (OBD2::getInstance().getDef(pid, def) != ESP_OK)
    {
        return nullptr;
    }

    cJSON* item = cJSON_CreateObject();

    // Use the struct data - no more mutex calls here
    cJSON_AddNumberToObject(item, "pid", def.pid);
    cJSON_AddNumberToObject(item, "mode", def.mode);
    cJSON_AddNumberToObject(item, "id", def.id);
    cJSON_AddNumberToObject(item, "length", def.len);
    cJSON_AddStringToObject(item, "name", def.name.c_str());
    cJSON_AddStringToObject(item, "unit", def.unit.c_str());
    cJSON_AddStringToObject(item, "description", def.description.c_str());
    cJSON_AddNumberToObject(item, "minValue", def.minValue);
    cJSON_AddNumberToObject(item, "maxValue", def.maxValue);
    cJSON_AddNumberToObject(item, "priority", def.priority);
    cJSON_AddNumberToObject(item, "update_interval_ms", def.updateInterval_ms);
    cJSON_AddNumberToObject(item, "color", def.color);
    cJSON_AddStringToObject(item, "icon", def.icon.c_str());
    cJSON_AddStringToObject(item, "formula", def.formula.c_str());

    return item;
}

cJSON* single_pid_data_get(uint16_t pid)
{
    PIDData_t data;

    if (OBD2::getInstance().getData(pid, data) != ESP_OK)
    {
        return nullptr;
    }

    cJSON* item = cJSON_CreateObject();

    cJSON_AddNumberToObject(item, "id", data.id);
    cJSON_AddNumberToObject(item, "pid", pid);
    cJSON_AddNumberToObject(item, "value", data.value);
    cJSON_AddNumberToObject(item, "lastUpdated", data.lastUpdated);
    cJSON_AddBoolToObject(item, "isSupported", data.isSupported);
    cJSON_AddBoolToObject(item, "isValid", data.isValid);
    cJSON_AddNumberToObject(item, "update_interval_ms", data.updateInterval_ms);

    return item;
}

cJSON* m_pid_def_get(int filter_id)
{
    cJSON* root       = cJSON_CreateObject();
    cJSON* data_array = cJSON_CreateArray();
    int    count      = 0;

    if (filter_id >= 0)
    {
        cJSON* item = single_pid_def_get((uint16_t)filter_id);
        if (item != nullptr)
        {
            cJSON_AddItemToArray(data_array, item);
            count++;
        }
    }
    else
    {
        std::vector<uint16_t> pids = OBD2::getInstance().getPIDs();
        for (const auto& pid : pids)
        {
            cJSON* item = single_pid_def_get(pid);
            if (item != nullptr)
            {
                cJSON_AddItemToArray(data_array, item);
                count++;
            }
        }
    }

    cJSON_AddItemToObject(root, "data", data_array);
    cJSON_AddNumberToObject(root, "count", count);

    return root;
}

cJSON* m_pid_data_get(int filter_id)
{
    cJSON* root       = cJSON_CreateObject();
    cJSON* data_array = cJSON_CreateArray();
    int    count      = 0;

    if (filter_id >= 0)
    {
        cJSON* item = single_pid_data_get((uint16_t)filter_id);
        if (item != nullptr)
        {
            cJSON_AddItemToArray(data_array, item);
            count++;
        }
    }
    else
    {
        std::vector<uint16_t> pids = OBD2::getInstance().getPIDs();
        for (const auto& pid : pids)
        {
            cJSON* item = single_pid_data_get(pid);
            if (item != nullptr)
            {
                cJSON_AddItemToArray(data_array, item);
                count++;
            }
        }
    }

    cJSON_AddItemToObject(root, "data", data_array);
    cJSON_AddNumberToObject(root, "count", count);

    return root;
}

void m_pid_poll_set_running(bool running)
{
    if (running)
    {
        OBD2::getInstance().startContinuousMode();
    }
    else
    {
        OBD2::getInstance().stopContinuousMode();
    }
}

cJSON* m_can_bus_get()
{
    cJSON* root = cJSON_CreateObject();

    const auto& nodeConfig = CanDriver::getInstance().getNodeConfig();
    const auto& config     = CanDriver::getInstance().getConfig();
    const auto& state      = CanDriver::getInstance().getState();

    const char* can_bus_state_name[] = {"not_initialized", "bus_off", "not_connected", "connected"};

    // Add CAN bus information to the JSON object
    cJSON_AddStringToObject(root, "state", can_bus_state_name[static_cast<int>(state)]);
    cJSON_AddNumberToObject(root, "bitrate", nodeConfig.bit_timing.bitrate);
    cJSON_AddStringToObject(root, "rs_mode",
                            config.rs_mode == CanDriver::RS_MODE::HIGH_SPEED ? "high_speed" : "slope_control");
    cJSON_AddBoolToObject(root, "debug_mode", config.debug);
    cJSON_AddBoolToObject(root, "initialized", CanDriver::getInstance().isInitialized());
    cJSON_AddBoolToObject(root, "bus_connected", CanDriver::getInstance().isBusConnected());

    cJSON* node_status = cJSON_CreateObject();

    auto        status            = CanDriver::getInstance().getStatus();
    const char* twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};

    cJSON_AddStringToObject(node_status, "twai_error_state", twai_state_name[status.state]);
    cJSON_AddNumberToObject(node_status, "tx_error_count", status.tx_error_count);
    cJSON_AddNumberToObject(node_status, "rx_error_count", status.rx_error_count);
    cJSON_AddItemToObject(root, "node_status", node_status);

    return root;
}

cJSON* m_obdii_get()
{
    cJSON* root = cJSON_CreateObject();

    // Add OBD-II information to the JSON object
    cJSON_AddBoolToObject(root, "continuous_running", OBD2::getInstance().isContinuousRunning());
    cJSON_AddBoolToObject(root, "pid_initialized", OBD2::getInstance().isPidInit());
    cJSON_AddNumberToObject(root, "pid_def_count", OBD2::getInstance().getPIDDEFSize());
    cJSON_AddNumberToObject(root, "pid_data_count", OBD2::getInstance().getPIDDataSize());
    cJSON_AddNumberToObject(root, "poll_task_utilization", OBD2::getInstance().getPollTaskUtilization());
    cJSON*               supported_pids = cJSON_CreateObject();
    supportedPIDsGroup_t supportedPIDsGroup;
    OBD2::getInstance().getSupportedPids(supportedPIDsGroup);
    cJSON_AddNumberToObject(supported_pids, "count", supportedPIDsGroup.numberOfSupportedPIDs);
    cJSON* groups = cJSON_CreateArray();
    for (int i = 0; i < SUPPORTED_PIDS_GROUP_COUNT; ++i)
    {
        cJSON_AddItemToArray(groups, cJSON_CreateNumber(supportedPIDsGroup.pidGroup[i]));
    }
    cJSON_AddItemToObject(supported_pids, "groups", groups);

    cJSON_AddItemToObject(root, "supported_pids", supported_pids);

    return root;
}

cJSON* m_system_get()
{
    cJSON* root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "app_version", APP_VERSION_MAJOR + APP_VERION_MINOR * 0.1);
    cJSON_AddNumberToObject(root, "uptime_s", SUPERVISOR::getInstance().get_uptime_seconds());
    cJSON_AddStringToObject(root, "restart_reason", SUPERVISOR::getInstance().get_restart_reason().c_str());
    cJSON_AddStringToObject(root, "mac", SUPERVISOR::getInstance().get_MAC_address().c_str());
    cJSON_AddNumberToObject(root, "state", static_cast<uint32_t>(SUPERVISOR::getInstance().get_state()));
    cJSON_AddNumberToObject(root, "battery_voltage", SUPERVISOR::getInstance().get_battery_voltage());
    cJSON_AddBoolToObject(root, "sd_card_detected", SDCard::getInstance().card_present());

    return root;
}

cJSON* m_sdcard_info_get()
{
    cJSON* root = cJSON_CreateObject();

    SDCard::SDInfo sd_info;

    SDCard::getInstance().get_sd_info(sd_info);

    cJSON_AddStringToObject(root, "name", sd_info.name);
    cJSON_AddStringToObject(root, "mount_path", sd_info.mount_path);
    cJSON_AddNumberToObject(root, "capacity", sd_info.capacity_mb);
    cJSON_AddNumberToObject(root, "used_space_mb", sd_info.used_space_mb);
    cJSON_AddNumberToObject(root, "max_freq_mhz", sd_info.max_freq_mhz);
    cJSON_AddBoolToObject(root, "is_sdio", sd_info.is_sdio);
    cJSON_AddBoolToObject(root, "is_mmc", sd_info.is_mmc);
    cJSON_AddBoolToObject(root, "is_mounted", sd_info.is_mounted);
    cJSON_AddBoolToObject(root, "is_present", sd_info.is_present);

    return root;
}

cJSON* m_sdcard_file_tree_get(const char* path)
{
    SDCard::SDInfo sd_info;
    auto&          sd = SDCard::getInstance();

    sd.get_sd_info(sd_info);

    if (!sd_info.is_mounted)
    {
        cJSON* err_root = cJSON_CreateObject();
        cJSON_AddStringToObject(err_root, "status", "error");
        cJSON_AddStringToObject(err_root, "reason", esp_err_to_name(ESP_ERR_NOT_FOUND));
        return err_root;
    }

    cJSON* root = sd.scan_directory(path, 5);

    if (root != nullptr)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }

    return root;
}

cJSON* m_sdcard_file_delete_delete(const char* path)
{
    SDCard::SDInfo sd_info;
    auto&          sd   = SDCard::getInstance();
    esp_err_t      err  = ESP_ERR_NOT_FOUND;
    cJSON*         root = cJSON_CreateObject();

    sd.get_sd_info(sd_info);

    if (sd_info.is_mounted)
    {
        auto filepath = std::make_unique<char[]>(PATH_MAX);

        snprintf(filepath.get(), PATH_MAX, "%s%s", sd_info.mount_path, path);

        err = sd.delete_file(filepath.get());
    }

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return root;
}

cJSON* m_vin_get()
{
    cJSON* root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "vin", OBD2::getInstance().getVIN().c_str());

    return root;
}

cJSON* m_dtc_get(int mode)
{
    cJSON* root         = cJSON_CreateObject();
    cJSON* items        = cJSON_CreateArray();
    int    global_count = 0;

    auto add_dtc_section = [&](int target_mode, const char* name)
    {
        if (mode == -1 || mode == target_mode)
        {
            global_count++;

            cJSON* item      = cJSON_CreateObject();
            cJSON* section   = cJSON_CreateArray();
            int    dtc_count = 0;

            std::vector<std::string> dtc = OBD2::getInstance().getDTC(static_cast<uint8_t>(target_mode));

            for (const auto& d : dtc)
            {
                cJSON_AddItemToArray(section, cJSON_CreateString(d.c_str()));
                dtc_count++;
            }

            cJSON_AddNumberToObject(item, "mode", target_mode);
            cJSON_AddStringToObject(item, "type", name);
            cJSON_AddItemToObject(item, "dtc", section);
            cJSON_AddNumberToObject(item, "dtc_count", dtc_count);

            cJSON_AddItemToArray(items, item);
        }
    };

    add_dtc_section(MODE_DTCS, "confirmed_dtcs");
    add_dtc_section(MODE_PENDING_DTCS, "pending_dtcs");
    add_dtc_section(MODE_PERMANENT_DTCS, "permanent_dtcs");

    cJSON_AddItemToObject(root, "dtcs", items);
    cJSON_AddNumberToObject(root, "count", global_count);
    cJSON_AddStringToObject(root, "status", "success");

    return root;
}

cJSON* m_dtc_description_get(const char* target_codes[], size_t count)
{
    cJSON* root = cJSON_CreateObject();

    std::unique_ptr<FILE, decltype(&fclose)> file(fopen(DTC_DESC_DB_PATH, "rb"), fclose);

    if (!file)
    {
        cJSON_AddStringToObject(root, "status", "error");

        std::string reason =
            "File " + std::string(DTC_DESC_DB_PATH) + " not found (resolved to " + DTC_DESC_DB_PATH + ")";

        cJSON_AddStringToObject(root, "reason", reason.c_str());

        cJSON_AddNumberToObject(root, "dtc_count", 0);
        return root;
    }

    fseek(file.get(), 0, SEEK_END);
    long total_records = ftell(file.get()) / 128;

    if (total_records <= 0)
    {
        cJSON_AddStringToObject(root, "status", "error");

        std::string reason = "No records found in " + std::string(DTC_DESC_DB_PATH);

        cJSON_AddStringToObject(root, "reason", reason.c_str());
        cJSON_AddNumberToObject(root, "dtc_count", 0);
        return root;
    }

    cJSON* dtcs_array = cJSON_CreateArray();
    char   read_code[6];
    char   desc_buf[128];

    for (size_t i = 0; i < count; i++)
    {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "dtc", target_codes[i]);

        long left = 0, right = total_records - 1;
        bool found = false;

        while (left <= right)
        {
            long mid = left + ((right - left) >> 1);

            fseek(file.get(), mid * 128, SEEK_SET);
            size_t bytes = fread(read_code, 1, 6, file.get());

            if (bytes == 0 || ferror(file.get()))
            {
                cJSON_Delete(dtcs_array);
                cJSON_AddStringToObject(root, "status", "error");

                std::string reason = "Failed to read file " + std::string(DTC_DESC_DB_PATH);

                cJSON_AddStringToObject(root, "reason", reason.c_str());
                cJSON_AddNumberToObject(root, "dtc_count", 0);
                return root;
            }

            read_code[5] = '\0';
            int cmp      = strcmp(read_code, target_codes[i]);

            if (cmp == 0)
            {
                fread(desc_buf, 1, 122, file.get());
                desc_buf[122] = '\0';

                cJSON_AddStringToObject(item, "description", desc_buf);
                found = true;
                break;
            }

            if (cmp < 0)
                left = mid + 1;
            else
                right = mid - 1;
        }

        if (!found)
        {
            cJSON_AddStringToObject(item, "description", "Description not found");
        }

        cJSON_AddItemToArray(dtcs_array, item);
    }

    cJSON_AddStringToObject(root, "status", "success");
    cJSON_AddNumberToObject(root, "dtc_count", count);
    cJSON_AddItemToObject(root, "dtcs", dtcs_array);

    return root;
}

cJSON* m_vin_request()
{
    cJSON*    root = cJSON_CreateObject();
    esp_err_t err  = OBD2::getInstance().requestVIN();

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return root;
}

cJSON* m_dtc_request(int mode)
{
    esp_err_t err = ESP_OK;
    if (mode == -1)
    {
        err = OBD2::getInstance().requestDTC(MODE_DTCS);
        if (err == ESP_OK)
            err = OBD2::getInstance().requestDTC(MODE_PENDING_DTCS);
        // if (err == ESP_OK)
        //     err = OBD2::getInstance().requestDTC(MODE_PERMANENT_DTCS);
    }
    else
    {
        err = OBD2::getInstance().requestDTC(mode);
    }

    cJSON* root = cJSON_CreateObject();
    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
        // add DTC data here
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }
    return root;
}

cJSON* m_clear_dtc_request()
{
    cJSON* root = cJSON_CreateObject();
    OBD2::getInstance().requestClearDTCs();

    cJSON_AddStringToObject(root, "status", "success");

    return root;
}

cJSON* m_static_pid_request()
{
    cJSON* root = cJSON_CreateObject();
    OBD2::getInstance().pollRequestStaticPids();

    cJSON_AddStringToObject(root, "status", "success");

    return root;
}

esp_err_t pid_stream_packet_get(uint16_t pid, uint8_t* out_packet)
{
    size_t offset = 0;

    // 8 bit message type
    out_packet[offset++] = MSG_TYPE_PID;

    // Length
    out_packet[offset++] = PID_STREAM_PACKET_SIZE - 2;

    // 32 bit pid
    out_packet[offset++] = (pid >> 0) & 0xFF;
    out_packet[offset++] = (pid >> 8) & 0xFF;
    out_packet[offset++] = (pid >> 16) & 0xFF;
    out_packet[offset++] = (pid >> 24) & 0xFF;

    // float value
    uint32_t float_bits;
    float    float_value = OBD2::getInstance().getValue(pid);
    memcpy(&float_bits, &float_value, sizeof(float_value));
    out_packet[offset++] = (float_bits >> 0) & 0xFF;
    out_packet[offset++] = (float_bits >> 8) & 0xFF;
    out_packet[offset++] = (float_bits >> 16) & 0xFF;
    out_packet[offset++] = (float_bits >> 24) & 0xFF;

    // 32 bit lastUpdated
    uint32_t lastUpdated = OBD2::getInstance().getLastUpdated(pid);
    out_packet[offset++] = (lastUpdated >> 0) & 0xFF;
    out_packet[offset++] = (lastUpdated >> 8) & 0xFF;
    out_packet[offset++] = (lastUpdated >> 16) & 0xFF;
    out_packet[offset++] = (lastUpdated >> 24) & 0xFF;

    // 32 bit interval
    uint32_t updateInterval_ms = OBD2::getInstance().getUpdateInterval(pid);
    out_packet[offset++]       = (updateInterval_ms >> 0) & 0xFF;
    out_packet[offset++]       = (updateInterval_ms >> 8) & 0xFF;
    out_packet[offset++]       = (updateInterval_ms >> 16) & 0xFF;
    out_packet[offset++]       = (updateInterval_ms >> 24) & 0xFF;

    // 8 bit isSupported
    out_packet[offset++] = OBD2::getInstance().isSup(pid) ? 1 : 0;

    // 8 bit isValid
    out_packet[offset++] = OBD2::getInstance().isValid(pid) ? 1 : 0;

    return ESP_OK;
}

esp_err_t can_status_packet_get(uint8_t* out_packet)
{
    size_t offset = 0;

    // 8 bit message type
    out_packet[offset++] = MSG_TYPE_CAN_STATUS;

    // Length
    out_packet[offset++] = CAN_STATUS_PACKET_SIZE - 2;

    // 8 bit state
    const auto& state    = CanDriver::getInstance().getState();
    out_packet[offset++] = static_cast<uint8_t>(state);

    // float utilization
    float    utilization = OBD2::getInstance().getPollTaskUtilization();
    uint32_t float_bits;
    memcpy(&float_bits, &utilization, sizeof(utilization));
    out_packet[offset++] = (float_bits >> 0) & 0xFF;
    out_packet[offset++] = (float_bits >> 8) & 0xFF;
    out_packet[offset++] = (float_bits >> 16) & 0xFF;
    out_packet[offset++] = (float_bits >> 24) & 0xFF;

    // float battery voltage
    float battery_voltage = SUPERVISOR::getInstance().get_battery_voltage();
    memcpy(&float_bits, &battery_voltage, sizeof(battery_voltage));
    out_packet[offset++] = (float_bits >> 0) & 0xFF;
    out_packet[offset++] = (float_bits >> 8) & 0xFF;
    out_packet[offset++] = (float_bits >> 16) & 0xFF;
    out_packet[offset++] = (float_bits >> 24) & 0xFF;

    // 8 bit isBusConnected
    out_packet[offset++] = CanDriver::getInstance().isBusConnected() ? 1 : 0;

    return ESP_OK;
}

cJSON* m_pid_def_delete(int filter_id)
{
    cJSON*    root = cJSON_CreateObject();
    esp_err_t err  = OBD2::getInstance().removePID(filter_id);

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return root;
}

cJSON* m_pid_def_post(cJSON* data)
{
    if (data == nullptr || !cJSON_IsArray(data))
    {
        cJSON* error_resp = cJSON_CreateObject();
        cJSON_AddStringToObject(error_resp, "status", "error");
        cJSON_AddStringToObject(error_resp, "reason", "Payload must be a JSON array");
        return error_resp;
    }

    int success_count = 0;
    int error_count   = 0;

    cJSON* added_array  = cJSON_CreateArray();
    cJSON* failed_array = cJSON_CreateArray();

    cJSON* item = nullptr;

    cJSON_ArrayForEach(item, data)
    {
        if (!cJSON_IsObject(item))
        {
            cJSON* fail_obj = cJSON_CreateObject();
            cJSON_AddNullToObject(fail_obj, "pid");
            cJSON_AddStringToObject(fail_obj, "error", esp_err_to_name(ESP_ERR_INVALID_ARG));
            cJSON_AddItemToArray(failed_array, fail_obj);

            error_count++;
            continue;
        }

        cJSON* pid         = cJSON_GetObjectItem(item, "pid");
        int    current_pid = -1;

        if (cJSON_IsNumber(pid))
        {
            current_pid = pid->valueint;
        }

        cJSON* id       = cJSON_GetObjectItem(item, "id");
        cJSON* mode     = cJSON_GetObjectItem(item, "mode");
        cJSON* name     = cJSON_GetObjectItem(item, "name");
        cJSON* formula  = cJSON_GetObjectItem(item, "formula");
        cJSON* interval = cJSON_GetObjectItem(item, "interval");

        if (!cJSON_IsNumber(id) || !cJSON_IsNumber(mode) || !cJSON_IsNumber(pid) || !cJSON_IsString(name) ||
            !cJSON_IsString(formula) || !cJSON_IsNumber(interval))
        {
            cJSON* fail_obj = cJSON_CreateObject();
            if (current_pid >= 0)
            {
                cJSON_AddNumberToObject(fail_obj, "pid", current_pid);
            }
            else
            {
                cJSON_AddNullToObject(fail_obj, "pid");
            }
            cJSON_AddStringToObject(fail_obj, "error", esp_err_to_name(ESP_ERR_INVALID_ARG));
            cJSON_AddItemToArray(failed_array, fail_obj);

            error_count++;
            continue;
        }

        uint16_t parsed_pid = static_cast<uint16_t>(current_pid);

        cJSON* len      = cJSON_GetObjectItem(item, "len");
        cJSON* unit     = cJSON_GetObjectItem(item, "unit");
        cJSON* desc     = cJSON_GetObjectItem(item, "desc");
        cJSON* minV     = cJSON_GetObjectItem(item, "minV");
        cJSON* maxV     = cJSON_GetObjectItem(item, "maxV");
        cJSON* priority = cJSON_GetObjectItem(item, "priority");
        cJSON* color    = cJSON_GetObjectItem(item, "color");
        cJSON* icon     = cJSON_GetObjectItem(item, "icon");

        uint8_t parsed_len = (parsed_pid > 0xFF) ? 3 : 2;

        std::string parsed_unit     = "";
        std::string parsed_desc     = "";
        float       parsed_minV     = 0.0f;
        float       parsed_maxV     = 0.0f;
        uint8_t     parsed_priority = 0;
        uint32_t    parsed_color    = 0x4EB31B;
        std::string parsed_icon     = "";

        if (cJSON_IsNumber(len))
        {
            parsed_len = static_cast<uint8_t>(len->valueint);
        }
        if (cJSON_IsString(unit))
        {
            parsed_unit = std::string(unit->valuestring);
        }
        if (cJSON_IsString(desc))
        {
            parsed_desc = std::string(desc->valuestring);
        }
        if (cJSON_IsNumber(minV))
        {
            parsed_minV = static_cast<float>(minV->valuedouble);
        }
        if (cJSON_IsNumber(maxV))
        {
            parsed_maxV = static_cast<float>(maxV->valuedouble);
        }
        if (cJSON_IsNumber(priority))
        {
            parsed_priority = static_cast<uint8_t>(priority->valueint);
        }
        if (cJSON_IsNumber(color))
        {
            parsed_color = static_cast<uint32_t>(color->valuedouble);
        }
        if (cJSON_IsString(icon))
        {
            parsed_icon = std::string(icon->valuestring);
        }

        esp_err_t err = OBD2::getInstance().addPID(
            static_cast<uint32_t>(id->valuedouble), static_cast<uint8_t>(mode->valueint), parsed_pid, parsed_len,
            std::string(name->valuestring), parsed_unit, parsed_desc, std::string(formula->valuestring), parsed_minV,
            parsed_maxV, parsed_priority, static_cast<uint16_t>(interval->valueint), parsed_color, parsed_icon);

        if (err == ESP_OK)
        {
            cJSON_AddItemToArray(added_array, cJSON_CreateNumber(parsed_pid));
            success_count++;
        }
        else
        {
            cJSON* fail_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(fail_obj, "pid", parsed_pid);
            cJSON_AddStringToObject(fail_obj, "error", esp_err_to_name(err));
            cJSON_AddItemToArray(failed_array, fail_obj);
            error_count++;
        }
    }

    cJSON* root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "added", added_array);
    cJSON_AddItemToObject(root, "failed", failed_array);

    if (error_count > 0 && success_count == 0)
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", "All items failed validation or hardware addition.");
    }
    else if (error_count > 0)
    {
        cJSON_AddStringToObject(root, "status", "partial_success");
        cJSON_AddStringToObject(root, "reason", "Some items were added, but others encountered errors.");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "success");
    }

    return root;
}

cJSON* m_pid_def_save()
{
    esp_err_t ret = SUPERVISOR::getInstance().save_pid_def_to_json(PID_DEF_DB_PATH);

    cJSON* root = cJSON_CreateObject();

    if (ret == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(ret));
    }

    return root;
}

cJSON* m_system_copy_file(cJSON* payload)
{
    if (payload == nullptr || !cJSON_IsObject(payload))
    {
        cJSON* error_resp = cJSON_CreateObject();
        cJSON_AddStringToObject(error_resp, "status", "error");
        cJSON_AddStringToObject(error_resp, "reason", "Payload must be a JSON object");
        return error_resp;
    }

    cJSON* src_node  = cJSON_GetObjectItem(payload, "source_path");
    cJSON* dest_node = cJSON_GetObjectItem(payload, "destination_path");

    if (!cJSON_IsString(src_node) || !cJSON_IsString(dest_node))
    {
        cJSON* error_resp = cJSON_CreateObject();
        cJSON_AddStringToObject(error_resp, "status", "error");
        cJSON_AddStringToObject(error_resp, "reason", "Missing or invalid 'source_path' or 'destination_path'");
        return error_resp;
    }

    const char* src_path  = src_node->valuestring;
    const char* dest_path = dest_node->valuestring;

    esp_err_t err = SUPERVISOR::getInstance().copy_file(src_path, dest_path);

    cJSON* root = cJSON_CreateObject();

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return root;
}