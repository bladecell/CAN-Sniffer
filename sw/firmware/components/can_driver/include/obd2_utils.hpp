// obd2_utils.hpp
#pragma once

#include <map>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include "can_driver.hpp"

#define OBD2_FUNCTIONAL_ID 0x7DF
#define OBD2_RESPONSE_BASE_ID 0x7E8
#define PID_DATA_LENGTH 8
#define VIN_LENGTH 17
#define PID_REQUEST_DELAY_MS MIN_TRANSMIT_PERIOD_MS
#define RESPONSE_ID_OFFSET 8
#define DAGNOSTIC_SESSION_TIMEOUT 2000

#define OBD2_MODE_TO_STR(mode)                                                                                       \
    ((mode) == MODE_CURRENT_DATA ? "Current Data" : (mode) == MODE_FREEZE_FRAME          ? "Freeze Frame"            \
                                                : (mode) == MODE_DTCS                    ? "DTCs"                    \
                                                : (mode) == MODE_CLEAR_DTCS              ? "Clear DTCs"              \
                                                : (mode) == MODE_TEST_RESULTS_O2         ? "O2 Test Results"         \
                                                : (mode) == MODE_TEST_RESULTS_OTHER      ? "Other Test Results"      \
                                                : (mode) == MODE_PENDING_DTCS            ? "Pending DTCs"            \
                                                : (mode) == MODE_CONTROL                 ? "Control"                 \
                                                : (mode) == MODE_VEHICLE_INFO            ? "Vehicle Info"            \
                                                : (mode) == MODE_PERMANENT_DTCS          ? "Permanent DTCs"          \
                                                : (mode) == MODE_READ_DATA_BY_IDENTIFIER ? "Read Data by Identifier" \
                                                                                         : "Unknown")

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
    MODE_PERMANENT_DTCS = 0x0A,
    MODE_READ_DATA_BY_IDENTIFIER = 0x22,
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
    RESPONSE_PERMANENT_DTCS = 0x4A,
    RESPONSE_READ_DATA_BY_IDENTIFIER = 0x62
};

typedef enum
{
    PID_PIDS_SUPPORTED_1_20 = 0x00,
    PID_ENGINE_LOAD = 0x04,
    PID_COOLANT_TEMP = 0x05,
    PID_ENGINE_RPM = 0x0C,
    PID_PIDS_SUPPORTED_21_40 = 0x20,
    PID_PIDS_SUPPORTED_41_60 = 0x40,
    PID_PIDS_SUPPORTED_61_80 = 0x60,
    PID_PIDS_SUPPORTED_81_A0 = 0x80,
    PID_PIDS_SUPPORTED_A1_C0 = 0xA0,
    PID_PIDS_SUPPORTED_C1_E0 = 0xC0,
    PID_VIN = 0x02
} OBDPID;

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
const char LITER[] = "L";

typedef enum
{
    UPDATE_STATIC = 16,
    UPDATE_FAST = 256,
    UPDATE_MEDIUM = 1024,
    UPDATE_SLOW = 4096,
} UpdateRate;

struct PIDDef_t
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
    UpdateRate updateInterval_ms;
    uint32_t color;
    const char *icon;
};

struct PIDData_t
{
    uint32_t id;
    float value;
    uint32_t lastUpdated;
    uint8_t data[PID_DATA_LENGTH];
    bool isSupported;
    bool isValid;
    UpdateRate updateInterval_ms;
    SemaphoreHandle_t mtx_;
};

typedef struct
{
    char vin[18];
    uint32_t lastUpdated;
    bool isValid;
    SemaphoreHandle_t vinReadySemaphore;
    SemaphoreHandle_t mtx_;
} VINData_t;

typedef struct
{
    std::vector<std::string> confirmed;
    std::vector<std::string> pending;
    std::vector<std::string> permanent;
    SemaphoreHandle_t confirmedReadySemaphore;
    SemaphoreHandle_t pendingReadySemaphore;
    SemaphoreHandle_t permanentReadySemaphore;
    SemaphoreHandle_t mtx_;
} DTCData_t;

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