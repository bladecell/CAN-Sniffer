// obd2_utils.hpp
#pragma once

#include <map>
#include <cstdint>

enum OBDMode {
    MODE_CURRENT_DATA = 0x01,
    MODE_FREEZE_FRAME = 0x02,
    MODE_DTCS = 0x03,
    MODE_CLEAR_DTCS = 0x04,
    MODE_TEST_RESULTS_O2 = 0x05,
    MODE_TEST_RESULTS_OTHER = 0x06,
    MODE_PENDING_DTCS = 0x07,
    MODE_CONTROL = 0x08,
    MODE_VEHICLE_INFO = 0x09,
    MODE_PERMANENT_DTCS = 0x0A
};

enum OBDResponse {
    RESPONSE_CURRENT_DATA = 0x41,
    RESPONSE_FREEZE_FRAME = 0x42,
    RESPONSE_DTCS = 0x43,
    RESPONSE_CLEAR_DTCS = 0x44,
    RESPONSE_TEST_RESULTS_O2 = 0x45,
    RESPONSE_TEST_RESULTS_OTHER = 0x46,
    RESPONSE_PENDING_DTCS = 0x47,
    RESPONSE_CONTROL = 0x48,
    RESPONSE_VEHICLE_INFO = 0x49,
    RESPONSE_PERMANENT_DTCS = 0x4A
};

enum OBDPID {
    PID_PIDS_SUPPORTED_1_20 = 0x00,
    PID_ENGINE_LOAD = 0x04,
    PID_COOLANT_TEMP = 0x05,
    PID_ENGINE_RPM = 0x0C,
};

struct PIDInfo {
    uint8_t mode;
    uint8_t pid;
    const char* name;
    const char* unit;
    const char* description;
    
    float (*formula)(const uint8_t* data, uint8_t len);
    
    // Metadata
    uint8_t expectedBytes;
    float minValue;
    float maxValue;
    uint16_t updateInterval_ms;
    uint8_t priority;
    bool isSupported;
};

struct PIDData {
    float value;
    uint32_t lastUpdated;
    bool isValid;
    uint8_t data[8];
};

struct PIDEntry {
    PIDInfo info;
    PIDData data;
};

static const std::map<uint8_t, PIDEntry> PIDDatabase = {
            {PID_ENGINE_LOAD, {
                MODE_CURRENT_DATA, PID_ENGINE_LOAD, 
                "Engine Load", "%", "Calculated engine load",
                OBDFormulas::engineLoad, 1, 0.0f, 100.0f, 100, 2, false
            }},
            {PID_COOLANT_TEMP, {
                MODE_CURRENT_DATA, PID_COOLANT_TEMP,
                "Coolant Temp", "°C", "Engine coolant temperature",
                OBDFormulas::coolantTemp, 1, -40.0f, 215.0f, 1000, 3, false
            }},
            {PID_ENGINE_RPM, {
                MODE_CURRENT_DATA, PID_ENGINE_RPM,
                "Engine RPM", "RPM", "Engine speed",
                OBDFormulas::engineRPM, 2, 0.0f, 16383.75f, 50, 1, false
            }},
            // ... more PIDs
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