// obd2.hpp
#pragma once

#include <math.h>
#include "obd2_utils.hpp"
#include "can_driver.hpp"
#include "esp_err.h"


class OBD2 {
    public:
        explicit OBD2(CanDriver& can_driver) : can_driver(can_driver), connected(false), continuousRunning(false) {}
        esp_err_t init();
        esp_err_t query_supported_pids(uint8_t pid_group);
        bool is_supported(uint8_t pid);
        const PIDData* get_pid_data(uint8_t pid) const;
    private:
        esp_err_t query_msg(uint8_t mode, uint8_t pid, uint8_t len, CanDriver::can_frame& rx_frame, uint32_t timeout_ms = 1000);
        CanDriver& can_driver;
        bool connected;
        bool continuousRunning;
        static const std::map<uint8_t, PIDInfo> pid_def;
        std::map<uint8_t, PIDData> pid_data;
        
        void init_definitions();
        
};

namespace OBDFormulas {
    inline float engineLoad(const uint8_t* data, uint8_t len) {
        return len >= 1 ? (data[0] * 100.0f) / 255.0f : -1.0f;
    }
    
    inline float coolantTemp(const uint8_t* data, uint8_t len) {
        return len >= 1 ? data[0] - 40 : -1.0f;
    }
    
    inline float engineRPM(const uint8_t* data, uint8_t len) {
        return len >= 2 ? ((data[0] * 256) + data[1]) / 4.0f : -1.0f;
    }
}
