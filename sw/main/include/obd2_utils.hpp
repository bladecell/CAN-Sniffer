// obd2_utils.hpp
#pragma once

#include <map>
#include <cstdint>

#define OBD2_FUNCTIONAL_ID 0x7DF
#define OBD2_RESPONSE_BASE_ID 0x7E8
#define PID_DATA_LENGTH 8

enum OBDMode
{
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

enum OBDResponse
{
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

enum OBDPID
{
    PID_PIDS_SUPPORTED_1_20 = 0x00,
    PID_ENGINE_LOAD = 0x04,
    PID_COOLANT_TEMP = 0x05,
    PID_ENGINE_RPM = 0x0C,
};

const char PERCENTAGE[] = "%";
const char KPA[] = "kPa";
const char PA[] = "Pa";
const char RPM[] = "rpm";
const char KPH[] = "km/h";
const char DEGREES_BEFORE_TDC[] = "° before TDC";
const char GRAMS_PER_SECOND[] = "grams/sec";
const char SECONDS[] = "seconds";
const char RATIO[] = "ratio";
const char COUNT[] = "count";
const char KM[] = "km";
const char VOLTS[] = "V";
const char MINUTES[] = "minutes";
const char GPS[] = "g/s";
const char DEGREES[] = "°";
const char DEGREES_CELCIUS[] = "°C";
const char LPH[] = "L/h";

struct PIDInfo
{
    uint8_t mode;
    uint8_t pid;
    const char *name;
    const char *unit;
    const char *description;
    float (*formula)(const uint8_t *data, uint8_t len);
    float minValue;
    float maxValue;
    uint8_t priority;
};

struct PIDData
{
    float value;
    uint32_t lastUpdated;
    uint8_t data[PID_DATA_LENGTH];
    bool isSupported;
    bool isValid;
    uint16_t updateInterval_ms;
};

namespace OBDFormulas
{
    inline float engineLoad(const uint8_t *data, uint8_t len)
    {
        return len >= 4 ? (data[3] * 100.0f) / 255.0f : -1.0f;
    }

    inline float coolantTemp(const uint8_t *data, uint8_t len)
    {
        return len >= 4 ? data[3] - 40 : -1.0f;
    }

    inline float engineRPM(const uint8_t *data, uint8_t len)
    {
        return len >= 5 ? ((data[3] << 8) | data[4]) / 4.0f : -1.0f;
    }
}