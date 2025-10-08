#include "esp_log.h"
#include "esp_err.h"
#include "obd2.hpp"
#include "freertos/FreeRTOS.h"

static const char *TAG = "OBD2";

esp_err_t OBD2::init() {
    if (!can_driver.isInitialized()) {
        ESP_LOGE(TAG, "CAN driver not initialized");
        return ESP_FAIL;
    }
    connected = true;

    init_definitions();
    query_supported_pids(PID_PIDS_SUPPORTED_1_20);

    ESP_LOGI(TAG, "OBD-II interface initialized");
    return ESP_OK;
}

const std::map<uint8_t, PIDInfo> OBD2::pid_def = {
    {PID_ENGINE_LOAD, {
        MODE_CURRENT_DATA, PID_ENGINE_LOAD, "Engine Load", "%",
        "Calculated engine load", OBDFormulas::engineLoad,
        1, 0.0f, 100.0f, 100, 2
    }},
    {PID_COOLANT_TEMP, {
        MODE_CURRENT_DATA, PID_COOLANT_TEMP, "Coolant Temp", "°C",
        "Engine coolant temperature", OBDFormulas::coolantTemp,
        1, -40.0f, 215.0f, 1000, 3
    }},
    {PID_ENGINE_RPM, {
        MODE_CURRENT_DATA, PID_ENGINE_RPM, "Engine RPM", "RPM",
        "Engine speed", OBDFormulas::engineRPM,
        2, 0.0f, 16383.75f, 50, 1
    }},
    // ... more definitions
};

const PIDData* OBD2::get_pid_data(uint8_t pid) const {
    auto it = pid_data.find(pid);
    return it != pid_data.end() ? &it->second : nullptr;
}

void OBD2::init_definitions() {
    for (const auto& [pid, _] : pid_def) {
        pid_data[pid] = {0.0f, 0, false, {0}, false};
    }
}

esp_err_t OBD2::query_supported_pids(uint8_t pid_group){
        CanDriver::can_frame responseFrame;
        
        esp_err_t ret = query_msg(MODE_CURRENT_DATA, pid_group, 0x02, responseFrame, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to query supported PIDs for group 0x%02X: %s", pid_group, esp_err_to_name(ret));
            return ret;
        }

        if (responseFrame.length >= 4) {
            uint32_t supportedPIDs = (responseFrame.data[3] << 24) |
                                    (responseFrame.data[4] << 16) |
                                    (responseFrame.data[5] << 8)  |
                                    (responseFrame.data[6]);
            
            ESP_LOGD(TAG, "Supported PIDs for group 0x%02X: 0x%08X", pid_group, supportedPIDs);
            
            for (uint8_t i = 0; i < 32; ++i) {
                if (supportedPIDs & (1UL << (31 - i))) {
                    uint8_t supportedPID = pid_group + 1 + i;

                    auto it_data = pid_data.find(supportedPID);
                    if (it_data != pid_data.end()) {
                        it_data->second.isSupported = true;

                        auto it_def = pid_def.find(supportedPID);
                        if (it_def != pid_def.end()) {
                            ESP_LOGI(TAG, "PID 0x%02X (%s) is supported",
                                    supportedPID, it_def->second.name);
                        } else {
                            ESP_LOGI(TAG, "PID 0x%02X is supported (unknown in pid_def)", supportedPID);
                        }
                    } else {
                        ESP_LOGW(TAG, "PID 0x%02X is supported but not in database", supportedPID);
                    }
                }
            }
        } else {
            ESP_LOGW(TAG, "Response frame too short to determine supported PIDs");
        }

        return ESP_OK;
    }

esp_err_t OBD2::query_msg(uint8_t mode, uint8_t pid, uint8_t len, CanDriver::can_frame& rx_frame, uint32_t timeout_ms) {
    if (!connected) {
        ESP_LOGE(TAG, "OBD-II interface not connected");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx_data[8] = {0};
    
    tx_data[0] = len;               // Number of additional data bytes
    tx_data[1] = mode;              // Mode
    tx_data[2] = pid;               // PID
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    
    twai_frame_t tx = {};
    
    tx.header.id = OBD2_FUNCTIONAL_ID;        // OBD-II functional request ID
    tx.header.dlc = twaifd_len2dlc(sizeof(tx_data));
    tx.header.ide = false;                   // Standard Frame Format (11-bit ID)
    tx.header.rtr = 0;                       // Data frame (not remote frame)
    tx.header.fdf = 0;                       // Classic CAN format
    tx.header.brs = 0;                       // No bit rate switching
    tx.header.esi = 0;                       // No error state indicator
    tx.header.timestamp = 0;                 // Not used for TX
    tx.header.trigger_time = 0;              // Not used for immediate transmission
    
    tx.buffer = tx_data;
    tx.buffer_len = sizeof(tx_data);

    ESP_ERROR_CHECK_WITHOUT_ABORT(can_driver.flushRxQueue());

    esp_err_t ret = can_driver.transmit(&tx, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send OBD request: %s", esp_err_to_name(ret));
    }

    const uint32_t start = xTaskGetTickCount();
    const uint8_t expected_mode = 0x40 | mode;

    while (pdTICKS_TO_MS(xTaskGetTickCount() - start) < timeout_ms) {
        CanDriver::can_frame f{};
        ret = can_driver.receive(f, 50);
        if (ret != ESP_OK) continue; 

        if (f.id < OBD2_RESPONSE_BASE_ID  || f.id > (OBD2_RESPONSE_BASE_ID + 7)) continue;

        if (f.length < 3) continue;

        if (f.data[1] == expected_mode && f.data[2] == pid) {
            rx_frame = f;
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

bool OBD2::is_supported(uint8_t pid)  {
    return pid_data.find(pid) != pid_data.end() && pid_data[pid].isSupported;
}