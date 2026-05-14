#include "middleware.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

#include "cJSON.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "obd2.hpp"
#include "supervisor.hpp"
#include "utilities.h"

cJSON* single_pid_def_get(uint16_t pid)
{
    cJSON* item = cJSON_CreateObject();

    cJSON_AddNumberToObject(item, "pid", pid);
    cJSON_AddNumberToObject(item, "mode", OBD2::getInstance().getMode(pid));
    cJSON_AddStringToObject(item, "name", OBD2::getInstance().getName(pid).c_str());
    cJSON_AddStringToObject(item, "unit", OBD2::getInstance().getUnit(pid).c_str());
    cJSON_AddStringToObject(item, "description", OBD2::getInstance().getDescription(pid).c_str());
    cJSON_AddNumberToObject(item, "minValue", OBD2::getInstance().getMinValue(pid));
    cJSON_AddNumberToObject(item, "maxValue", OBD2::getInstance().getMaxValue(pid));
    cJSON_AddNumberToObject(item, "priority", OBD2::getInstance().getPriority(pid));
    cJSON_AddNumberToObject(item, "update_interval_ms", OBD2::getInstance().getUpdateInterval(pid));
    cJSON_AddNumberToObject(item, "color", OBD2::getInstance().getColor(pid));
    cJSON_AddStringToObject(item, "icon", OBD2::getInstance().getIcon(pid).c_str());
    cJSON_AddStringToObject(item, "formula", OBD2::getInstance().getFormula(pid).c_str());

    return item;
}

cJSON* single_pid_data_get(uint16_t pid)
{
    cJSON* item = cJSON_CreateObject();

    auto& obd = OBD2::getInstance();

    cJSON_AddNumberToObject(item, "id", obd.getId(pid));
    cJSON_AddNumberToObject(item, "pid", pid);
    cJSON_AddNumberToObject(item, "value", obd.getValue(pid));
    cJSON_AddNumberToObject(item, "lastUpdated", obd.getLastUpdated(pid));
    cJSON_AddBoolToObject(item, "isSupported", obd.isSup(pid));
    cJSON_AddBoolToObject(item, "isValid", obd.isValid(pid));
    cJSON_AddNumberToObject(item, "update_interval_ms", obd.getUpdateInterval(pid));

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
        if (item)
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
            if (item)
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
        if (item)
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
            if (item)
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

    // Add System information to the JSON object
    cJSON_AddNumberToObject(root, "app_version", APP_VERSION_MAJOR + APP_VERION_MINOR * 0.1);
    cJSON_AddNumberToObject(root, "uptime_s", SUPERVISOR::getInstance().get_uptime_seconds());
    cJSON_AddStringToObject(root, "pid_initialized", SUPERVISOR::getInstance().get_restart_reason().c_str());
    cJSON_AddStringToObject(root, "mac", SUPERVISOR::getInstance().get_MAC_address().c_str());
    cJSON_AddNumberToObject(root, "state", static_cast<uint32_t>(SUPERVISOR::getInstance().get_state()));
    cJSON_AddNumberToObject(root, "battery_voltage", SUPERVISOR::getInstance().get_battery_voltage());

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

    return root;
}

cJSON* m_vin_request()
{
    cJSON* root = cJSON_CreateObject();

    if (esp_err_t err = OBD2::getInstance().requestVIN() == ESP_OK)
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
        if (err == ESP_OK)
            err = OBD2::getInstance().requestDTC(MODE_PERMANENT_DTCS);
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
    return cJSON_CreateObject();
}