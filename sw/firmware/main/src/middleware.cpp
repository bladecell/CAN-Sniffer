#include "middleware.hpp"
#include "esp_check.h"

cJSON *get_single_pid_json(uint8_t pid, const PIDDef_t &pi)
{
    cJSON *item = cJSON_CreateObject();

    cJSON_AddNumberToObject(item, "pid", pid);
    cJSON_AddNumberToObject(item, "mode", pi.mode);
    cJSON_AddStringToObject(item, "name", pi.name ? pi.name : "");
    cJSON_AddStringToObject(item, "unit", pi.unit ? pi.unit : "");
    cJSON_AddStringToObject(item, "description", pi.description ? pi.description : "");
    cJSON_AddNumberToObject(item, "minValue", pi.minValue);
    cJSON_AddNumberToObject(item, "maxValue", pi.maxValue);
    cJSON_AddNumberToObject(item, "priority", pi.priority);
    cJSON_AddNumberToObject(item, "update_interval_ms", pi.updateInterval_ms);

    return item;
}

cJSON *m_pid_data_json(int filter_id)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *data_array = cJSON_CreateArray();
    int count = 0;

    if (filter_id >= 0)
    {
        const auto &pids = OBD2::getInstance().getPID_DEF();
        auto it = pids.find((uint8_t)filter_id);

        if (it != pids.end())
        {
            cJSON *item = get_single_pid_json(it->first, it->second);
            if (item)
            {
                cJSON_AddItemToArray(data_array, item);
                count++;
            }
        }
    }
    else
    {
        for (const auto &pair : OBD2::getInstance().getPID_DEF())
        {
            cJSON *item = get_single_pid_json(pair.first, pair.second);
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

cJSON *m_pid_poll_json()
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "running", OBD2::getInstance().isContinuousRunning());
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

cJSON *m_can_bus_json()
{
    cJSON *root = cJSON_CreateObject();

    const auto &nodeConfig = CanDriver::getInstance().getNodeConfig();
    const auto &config = CanDriver::getInstance().getConfig();
    const auto &state = CanDriver::getInstance().getState();

    const char *can_bus_state_name[] = {"not_initialized", "bus_off", "not_connected", "connected"};

    // Add CAN bus information to the JSON object
    cJSON_AddStringToObject(root, "state", can_bus_state_name[static_cast<int>(state)]);
    cJSON_AddNumberToObject(root, "bitrate", nodeConfig.bit_timing.bitrate);
    cJSON_AddStringToObject(root, "rs_mode", config.rs_mode == CanDriver::RS_MODE::HIGH_SPEED ? "high_speed" : "slope_control");
    cJSON_AddBoolToObject(root, "debug_mode", config.debug);
    cJSON_AddBoolToObject(root, "initialized", CanDriver::getInstance().isInitialized());
    cJSON_AddBoolToObject(root, "bus_connected", CanDriver::getInstance().isBusConnected());

    cJSON *node_status = cJSON_CreateObject();

    auto status = CanDriver::getInstance().getStatus();
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};

    cJSON_AddStringToObject(node_status, "twai_error_state", twai_state_name[status.state]);
    cJSON_AddNumberToObject(node_status, "tx_error_count", status.tx_error_count);
    cJSON_AddNumberToObject(node_status, "rx_error_count", status.rx_error_count);
    cJSON_AddItemToObject(root, "node_status", node_status);

    return root;
}
