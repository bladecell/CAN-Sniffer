// obd2_utils.hpp
#pragma once

#include <map>
#include <cstdint>

#define OBD2_FUNCTIONAL_ID       0x7DF
#define OBD2_RESPONSE_BASE_ID    0x7E8

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
    uint8_t expectedBytes;
    float minValue;
    float maxValue;
    uint16_t updateInterval_ms;
    uint8_t priority;
};

struct PIDData {
    float value;
    uint32_t lastUpdated;
    bool isValid;
    uint8_t data[8];
    bool isSupported;
};
