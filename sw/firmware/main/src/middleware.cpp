#include "middleware.hpp"

#include "esp_check.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

cJSON* get_single_pid_def_json(const PIDDefinition* def)
{
    cJSON* item = cJSON_CreateObject();

    if (def == nullptr)
    {
        return item;
    }

    cJSON_AddNumberToObject(item, "pid", def->pid());
    cJSON_AddNumberToObject(item, "mode", def->mode());
    cJSON_AddStringToObject(item, "name", def->name().c_str());
    cJSON_AddStringToObject(item, "unit", def->unit().c_str());
    cJSON_AddStringToObject(item, "description", def->description().c_str());
    cJSON_AddNumberToObject(item, "minValue", def->minValue());
    cJSON_AddNumberToObject(item, "maxValue", def->maxValue());
    cJSON_AddNumberToObject(item, "priority", def->priority());
    cJSON_AddNumberToObject(item, "update_interval_ms", def->updateInterval());
    cJSON_AddNumberToObject(item, "color", def->color());
    cJSON_AddStringToObject(item, "icon", def->icon().c_str());
    cJSON_AddStringToObject(item, "formula", def->formula().c_str());

    return item;
}

cJSON* get_single_pid_data_json(uint16_t pid, const PIDData_t& pd)
{
    uint32_t id, lastUpdated, updateInterval;
    float    value;
    bool     isSupported, isValid;

    if (xSemaphoreTake(pd.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return nullptr;
    }

    id             = pd.id;
    value          = pd.value;
    lastUpdated    = pd.lastUpdated;
    isSupported    = pd.isSupported;
    isValid        = pd.isValid;
    updateInterval = pd.updateInterval_ms;

    xSemaphoreGive(pd.mtx_);

    cJSON* item = cJSON_CreateObject();

    cJSON_AddNumberToObject(item, "id", id);
    cJSON_AddNumberToObject(item, "pid", pid);
    cJSON_AddNumberToObject(item, "value", value);
    cJSON_AddNumberToObject(item, "lastUpdated", lastUpdated);
    cJSON_AddBoolToObject(item, "isSupported", isSupported);
    cJSON_AddBoolToObject(item, "isValid", isValid);
    cJSON_AddNumberToObject(item, "update_interval_ms", updateInterval);

    return item;
}

cJSON* m_pid_def_json(int filter_id)
{
    cJSON* root       = cJSON_CreateObject();
    cJSON* data_array = cJSON_CreateArray();
    int    count      = 0;

    if (filter_id >= 0)
    {
        const PIDDefinition* def = nullptr;

        if (OBD2::getInstance().getDef((uint16_t)filter_id, def) == ESP_OK)
        {
            cJSON* item = get_single_pid_def_json(def);
            if (item)
            {
                cJSON_AddItemToArray(data_array, item);
                count++;
            }
        }
    }
    else
    {
        std::vector<uint16_t> pids = OBD2::getInstance().getPIDs();
        for (const auto& pid : pids)
        {
            const PIDDefinition* def = nullptr;
            if (OBD2::getInstance().getDef(pid, def) == ESP_OK)
            {
                cJSON* item = get_single_pid_def_json(def);
                if (item)
                {
                    cJSON_AddItemToArray(data_array, item);
                    count++;
                }
            }
        }
    }

    cJSON_AddItemToObject(root, "data", data_array);
    cJSON_AddNumberToObject(root, "count", count);

    return root;
}

cJSON* m_pid_data_json(int filter_id)
{
    cJSON* root       = cJSON_CreateObject();
    cJSON* data_array = cJSON_CreateArray();
    int    count      = 0;

    if (filter_id >= 0)
    {
        PIDData_t pd;

        if (OBD2::getInstance().getData((uint16_t)filter_id, pd) == ESP_OK)
        {
            cJSON* item = get_single_pid_data_json((uint16_t)filter_id, pd);
            if (item)
            {
                cJSON_AddItemToArray(data_array, item);
                count++;
            }
        }
    }
    else
    {
        std::vector<uint16_t> pids = OBD2::getInstance().getPIDs();
        for (const auto& pid : pids)
        {
            PIDData_t pd;
            if (OBD2::getInstance().getData(pid, pd) == ESP_OK)
            {
                cJSON* item = get_single_pid_data_json(pid, pd);
                if (item)
                {
                    cJSON_AddItemToArray(data_array, item);
                    count++;
                }
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

cJSON* m_can_bus_json()
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

cJSON* m_obdii_json()
{
    cJSON* root = cJSON_CreateObject();

    // Add OBD-II information to the JSON object
    cJSON_AddBoolToObject(root, "continuous_running", OBD2::getInstance().isContinuousRunning());
    cJSON_AddBoolToObject(root, "pid_initialized", OBD2::getInstance().isPidInit());
    cJSON_AddNumberToObject(root, "pid_def_count", OBD2::getInstance().getPIDDEFSize());
    cJSON_AddNumberToObject(root, "pid_data_count", OBD2::getInstance().getPIDDataSize());

    return root;
}

cJSON* m_vin_json()
{
    cJSON* root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "vin", OBD2::getInstance().getVIN().c_str());

    return root;
}

cJSON* m_dtc_json(int mode)
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

    if (OBD2::getInstance().requestVIN() == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
    }

    return root;
}

cJSON* m_dtc_request(int mode)
{
    esp_err_t err = ESP_OK;
    if (mode == -1)
    {
        OBD2::getInstance().requestConfirmedDTCs();
        vTaskDelay(pdMS_TO_TICKS(200));
        OBD2::getInstance().requestPendingDTCs();
        vTaskDelay(pdMS_TO_TICKS(200));
        OBD2::getInstance().requestPermanentDTCs();
    }
    else
    {
        switch (mode)
        {
            case MODE_DTCS:
                OBD2::getInstance().requestConfirmedDTCs();
                break;
            case MODE_PENDING_DTCS:
                OBD2::getInstance().requestPendingDTCs();
                break;
            case MODE_PERMANENT_DTCS:
                OBD2::getInstance().requestPermanentDTCs();
                break;
            default:
                err = ESP_ERR_INVALID_ARG;
                break;
        }
    }
    cJSON* root = cJSON_CreateObject();

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
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

esp_err_t get_pid_stream_packet(uint16_t pid, uint8_t* out_packet)
{
    PIDData_t data;

    esp_err_t err = OBD2::getInstance().getData(pid, data);
    if (err != ESP_OK)
    {
        return err;
    }

    size_t offset = 0;

    // 8 bit message type
    out_packet[offset++] = MSG_TYPE_PID;

    // 32 bit pid
    out_packet[offset++] = (pid >> 0) & 0xFF;
    out_packet[offset++] = (pid >> 8) & 0xFF;
    out_packet[offset++] = (pid >> 16) & 0xFF;
    out_packet[offset++] = (pid >> 24) & 0xFF;

    // float value
    uint32_t float_bits;
    memcpy(&float_bits, &data.value, sizeof(data.value));
    out_packet[offset++] = (float_bits >> 0) & 0xFF;
    out_packet[offset++] = (float_bits >> 8) & 0xFF;
    out_packet[offset++] = (float_bits >> 16) & 0xFF;
    out_packet[offset++] = (float_bits >> 24) & 0xFF;

    // 32 bit lastUpdated
    out_packet[offset++] = (data.lastUpdated >> 0) & 0xFF;
    out_packet[offset++] = (data.lastUpdated >> 8) & 0xFF;
    out_packet[offset++] = (data.lastUpdated >> 16) & 0xFF;
    out_packet[offset++] = (data.lastUpdated >> 24) & 0xFF;

    // 32 bit interval
    out_packet[offset++] = (data.updateInterval_ms >> 0) & 0xFF;
    out_packet[offset++] = (data.updateInterval_ms >> 8) & 0xFF;
    out_packet[offset++] = (data.updateInterval_ms >> 16) & 0xFF;
    out_packet[offset++] = (data.updateInterval_ms >> 24) & 0xFF;

    // 8 bit isSupported
    out_packet[offset++] = data.isSupported ? 1 : 0;

    // 8 bit isValid
    out_packet[offset++] = data.isValid ? 1 : 0;

    return ESP_OK;
}