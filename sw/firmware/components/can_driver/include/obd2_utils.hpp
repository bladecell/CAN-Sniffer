// obd2_utils.hpp
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define OBD2_FUNCTIONAL_ID 0x7DF
#define OBD2_RESPONSE_BASE_ID 0x7E8
#define PID_DATA_LENGTH 8
#define VIN_LENGTH 17
#define PID_REQUEST_DELAY_MS MIN_TRANSMIT_PERIOD_MS
#define RESPONSE_ID_OFFSET 8
#define DAGNOSTIC_SESSION_TIMEOUT 2000
#define SUPPORTED_PIDS_GROUP_COUNT 7

#define OBD2_MODE_TO_STR(mode)                                            \
    ((mode) == MODE_CURRENT_DATA              ? "Current Data"            \
     : (mode) == MODE_FREEZE_FRAME            ? "Freeze Frame"            \
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
    MODE_CURRENT_DATA               = 0x01,
    MODE_FREEZE_FRAME               = 0x02,
    MODE_DTCS                       = 0x03,
    MODE_CLEAR_DTCS                 = 0x04,
    MODE_TEST_RESULTS_O2            = 0x05,
    MODE_TEST_RESULTS_OTHER         = 0x06,
    MODE_PENDING_DTCS               = 0x07,
    MODE_CONTROL                    = 0x08,
    MODE_VEHICLE_INFO               = 0x09,
    MODE_PERMANENT_DTCS             = 0x0A,
    MODE_DIAGNOSTIC_SESSION_CONTROL = 0x10,
    MODE_READ_DATA_BY_IDENTIFIER    = 0x22,
    MODE_DERIVED_DATA               = 0x45,
};

enum OBDResponse
{
    RESPONSE_CURRENT_DATA            = 0x41,
    RESPONSE_FREEZE_FRAME            = 0x42,
    RESPONSE_DTCS                    = 0x43,
    RESPONSE_CLEAR_DTCS              = 0x44,
    RESPONSE_TEST_RESULTS_O2         = 0x45,
    RESPONSE_TEST_RESULTS_OTHER      = 0x46,
    RESPONSE_PENDING_DTCS            = 0x47,
    RESPONSE_CONTROL                 = 0x48,
    RESPONSE_VEHICLE_INFO            = 0x49,
    RESPONSE_PERMANENT_DTCS          = 0x4A,
    RESPONSE_READ_DATA_BY_IDENTIFIER = 0x62,
    RESPONSE_MODE_DERIVED_DATA       = 0x85,
    RESPONSE_NEGATIVE_RESPONSE_CODE  = 0x7F,
};

typedef enum
{
    PID_PIDS_SUPPORTED_0_19  = 0x00,
    PID_ENGINE_LOAD          = 0x04,
    PID_COOLANT_TEMP         = 0x05,
    PID_ENGINE_RPM           = 0x0C,
    PID_PIDS_SUPPORTED_20_39 = 0x20,
    PID_PIDS_SUPPORTED_40_59 = 0x40,
    PID_PIDS_SUPPORTED_60_79 = 0x60,
    PID_PIDS_SUPPORTED_80_99 = 0x80,
    PID_PIDS_SUPPORTED_A0_BF = 0xA0,
    PID_PIDS_SUPPORTED_C0_DF = 0xC0,
    PID_VIN                  = 0x02
} OBDPID;

const char PERCENTAGE[]         = "%";
const char KPA[]                = "kPa";
const char PA[]                 = "Pa";
const char RPM[]                = "rpm";
const char KPH[]                = "km/h";
const char DEGREES_BEFORE_TDC[] = "° before TDC";
const char GRAMS_PER_SECOND[]   = "grams/sec";
const char SECONDS[]            = "seconds";
const char RATIO[]              = "ratio";
const char COUNT[]              = "count";
const char KM[]                 = "km";
const char VOLTS[]              = "V";
const char MINUTES[]            = "minutes";
const char GPS[]                = "g/s";
const char DEGREES[]            = "°";
const char DEGREES_CELCIUS[]    = "°C";
const char LPH[]                = "L/h";
const char LITER[]              = "L";

typedef enum
{
    UPDATE_STATIC = 16,
    UPDATE_FAST   = 256,
    UPDATE_MEDIUM = 1024,
    UPDATE_SLOW   = 4096,
} UpdateRate;

struct supportedPIDsGroup_t
{
    uint32_t pidGroup[SUPPORTED_PIDS_GROUP_COUNT];
    uint16_t numberOfSupportedPIDs;
};

struct PIDDef_t
{
    uint8_t     mode;
    uint8_t     pid;
    const char* name;
    const char* unit;
    const char* description;
    float (*formula)(const uint8_t* data, uint8_t len);
    float       minValue;
    float       maxValue;
    uint8_t     priority;
    UpdateRate  updateInterval_ms;
    uint32_t    color;
    const char* icon;
};

struct PIDData_t
{
    uint32_t   id;
    float      value;
    uint32_t   lastUpdated;
    uint8_t    data[PID_DATA_LENGTH];
    bool       isSupported;
    bool       isValid;
    UpdateRate updateInterval_ms;
};

struct PollRequest
{
    uint16_t   pid;
    TickType_t nextWake;
    uint32_t   interval;
    uint8_t    priority;
    bool       isRecurring;
    uint32_t   id;
    uint8_t    mode;
    uint8_t    len;

    // Min-Heap logic: Sooner wake time = higher priority.
    // If times are equal, lower priority value (0 is highest) wins.
    bool operator<(const PollRequest& other) const
    {
        if (nextWake == other.nextWake)
        {
            return priority < other.priority;
        }
        return nextWake < other.nextWake;
    }
};

typedef struct
{
    char              vin[18];
    uint32_t          lastUpdated;
    bool              isValid;
    SemaphoreHandle_t vinReadySemaphore;
    SemaphoreHandle_t mtx_;
} VINData_t;

typedef struct
{
    std::vector<std::string> confirmed;
    std::vector<std::string> pending;
    std::vector<std::string> permanent;
    SemaphoreHandle_t        confirmedReadySemaphore;
    SemaphoreHandle_t        pendingReadySemaphore;
    SemaphoreHandle_t        permanentReadySemaphore;
    SemaphoreHandle_t        mtx_;
} DTCData_t;

class MutexGuard
{
public:
    explicit MutexGuard(SemaphoreHandle_t mtx, TickType_t timeout = portMAX_DELAY) : _mtx(mtx)
    {
        if (_mtx != nullptr)
        {
            _locked = (xSemaphoreTake(_mtx, timeout) == pdTRUE);
        }
    }

    ~MutexGuard()
    {
        if (_mtx != nullptr && _locked)
        {
            xSemaphoreGive(_mtx);
        }
    }

    bool isLocked() const
    {
        return _locked;
    }

private:
    SemaphoreHandle_t _mtx;
    bool              _locked = false;
};