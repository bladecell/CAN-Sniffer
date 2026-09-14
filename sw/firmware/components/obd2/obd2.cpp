/**
 * @file obd2.cpp
 * @author bladecell (github@bum.anonaddy.com)
 * @brief
 * @version 0.1
 * @date 2025-12-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "obd2.hpp"

#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <utility>

#include "can_driver.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "obd2_common.hpp"
#include "obd2_data_model.hpp"

#define HEALTHCHECK_RETRIES 3
#define OBD2_HEALTHCHECK_PING_PERIOD_MS                                                                              \
    (NO_MESSAGE_SENT_HEALTHCHECK_PING_PERIOD_MS - 1000) > 1000 ? (NO_MESSAGE_SENT_HEALTHCHECK_PING_PERIOD_MS - 1000) \
                                                               : 1000
#define HEALTHCHECK_RESPONSE_TIMEOUT_MS 3000
#define SUPPORTED_PID_RESPONSE_TIMEOUT_MS 500
#define SUPPORTED_PID_RESPONSE_WINDOW_MS 50

static const char* TAG = "OBD2";

// tinyexpr formula helpers: read other PID values/raw bytes during evaluation.
double get_pid_value(double id)
{
    return (double)OBD2::getInstance().getValueUnsafe((uint16_t)id);
}

double get_pid_raw(double id, double byte_idx)
{
    return (double)OBD2::getInstance().getRawDataByteUnsafe((uint16_t)id, (uint8_t)byte_idx);
}

class CanLoadTracker
{
private:
    TickType_t  last_slice_tick;
    uint32_t    tx_count_this_slice;
    float       current_ema_utilization;
    const float ALPHA = 0.15f;

public:
    CanLoadTracker()
    {
        last_slice_tick         = xTaskGetTickCount();
        tx_count_this_slice     = 0;
        current_ema_utilization = 0.0f;
    }

    void recordTx()
    {
        tx_count_this_slice++;
    }

    float updateAndGet()
    {
        TickType_t now           = xTaskGetTickCount();
        TickType_t elapsed_ticks = now - last_slice_tick;

        if (elapsed_ticks >= pdMS_TO_TICKS(25))
        {
            float elapsed_ms = pdTICKS_TO_MS(elapsed_ticks);

            float max_possible_frames = elapsed_ms / (float)MIN_TRANSMIT_PERIOD_MS;

            float instant_slice_util = 0.0f;
            if (max_possible_frames > 0.0f)
            {
                instant_slice_util = (float)tx_count_this_slice / max_possible_frames;
            }

            if (instant_slice_util > 1.0f)
                instant_slice_util = 1.0f;

            current_ema_utilization = (instant_slice_util * ALPHA) + (current_ema_utilization * (1.0f - ALPHA));

            tx_count_this_slice = 0;
            last_slice_tick     = now;
        }

        return current_ema_utilization;
    }
};

/**
 * @brief Construct a new OBD2::OBD2 object
 *
 */
OBD2::OBD2()
    : continuousRunning(false),
      configurationMtx_(xSemaphoreCreateMutex()),
      xPidConnectedSemaphore(nullptr),
      xBusConnectionSemaphore(nullptr),
      xBusArbitrationMutex(nullptr),
      xRequestNextPIDSemaphore(nullptr)
{
}

/**
 * @brief Destroy the OBD2::OBD2 object
 *
 */
OBD2::~OBD2()
{
    deinit();
}

void OBD2::deinit()
{
    stopContinuousMode();
    canDriver.setConnectionChangeCallback(nullptr, nullptr);

    if (ReceiveTaskHandle)
    {
        vTaskDelete(ReceiveTaskHandle);
        ReceiveTaskHandle = nullptr;
    }
    if (PollTaskHandle)
    {
        vTaskDelete(PollTaskHandle);
        PollTaskHandle = nullptr;
    }
    if (callbackWorkerTaskHandle)
    {
        vTaskDelete(callbackWorkerTaskHandle);
        callbackWorkerTaskHandle = nullptr;
    }
    if (xPidConnectedSemaphore)
    {
        vSemaphoreDelete(xPidConnectedSemaphore);
        xPidConnectedSemaphore = nullptr;
    }
    if (xBusArbitrationMutex)
    {
        vSemaphoreDelete(xBusArbitrationMutex);
        xBusArbitrationMutex = nullptr;
    }
    if (xBusConnectionSemaphore)
    {
        vSemaphoreDelete(xBusConnectionSemaphore);
        xBusConnectionSemaphore = nullptr;
    }
    if (healthCheckSemaphore)
    {
        vSemaphoreDelete(healthCheckSemaphore);
        healthCheckSemaphore = nullptr;
    }
    if (xRequestNextPIDSemaphore != nullptr)
    {
        vSemaphoreDelete(xRequestNextPIDSemaphore);
        xRequestNextPIDSemaphore = nullptr;
    }
    if (derivedPidQueue_)
    {
        vQueueDelete(derivedPidQueue_);
        derivedPidQueue_ = nullptr;
    }
    if (event_queue)
    {
        vQueueDelete(event_queue);
        event_queue = nullptr;
    }

    if (dtcData.confirmedReadySemaphore)
    {
        vSemaphoreDelete(dtcData.confirmedReadySemaphore);
        dtcData.confirmedReadySemaphore = nullptr;
    }
    if (dtcData.pendingReadySemaphore)
    {
        vSemaphoreDelete(dtcData.pendingReadySemaphore);
        dtcData.pendingReadySemaphore = nullptr;
    }
    if (dtcData.permanentReadySemaphore)
    {
        vSemaphoreDelete(dtcData.permanentReadySemaphore);
        dtcData.permanentReadySemaphore = nullptr;
    }
    if (dtcData.clearReadySemaphore)
    {
        vSemaphoreDelete(dtcData.clearReadySemaphore);
        dtcData.clearReadySemaphore = nullptr;
    }
    if (dtcData.mtx_)
    {
        vSemaphoreDelete(dtcData.mtx_);
        dtcData.mtx_ = nullptr;
    }
    if (vinData.vinReadySemaphore)
    {
        vSemaphoreDelete(vinData.vinReadySemaphore);
        vinData.vinReadySemaphore = nullptr;
    }
    if (vinData.mtx_)
    {
        vSemaphoreDelete(vinData.mtx_);
        vinData.mtx_ = nullptr;
    }

    if (pidMapMtx)
    {
        vSemaphoreDelete(pidMapMtx);
        pidMapMtx = nullptr;
    }
    if (subscribers_mtx_)
    {
        vSemaphoreDelete(subscribers_mtx_);
        subscribers_mtx_ = nullptr;
    }
    if (connected_subscribers_mtx_)
    {
        vSemaphoreDelete(connected_subscribers_mtx_);
        connected_subscribers_mtx_ = nullptr;
    }

    connected_subscribers_.clear();
    subscribers_.clear();
    nrc_list_size = 0;

    pidsInitialized = false;
}

/**
 * @brief Initialize the OBD2 interface
 *
 * @return esp_err_t ESP_OK if initialization was successful, otherwise an error code
 */
esp_err_t OBD2::init()
{
    if (!canDriver.isInitialized())
    {
        ESP_LOGE(TAG, "CAN driver not initialized");
        return ESP_FAIL;
    }

    if (xPidConnectedSemaphore == nullptr)
    {
        xPidConnectedSemaphore   = xSemaphoreCreateBinary();
        xBusArbitrationMutex     = xSemaphoreCreateMutex();
        xBusConnectionSemaphore  = xSemaphoreCreateBinary();
        xRequestNextPIDSemaphore = xSemaphoreCreateBinary();

        healthCheckSemaphore = xSemaphoreCreateBinary();

        connected_subscribers_mtx_ = xSemaphoreCreateMutex();
    }

    if (derivedPidQueue_ == nullptr)
    {
        derivedPidQueue_ = xQueueCreate(10, sizeof(uint16_t));
    }

    initDef();

    if (event_queue == nullptr)
    {
        event_queue = xQueueCreate(5, sizeof(bool));
    }
    BaseType_t taskCreated =
        xTaskCreatePinnedToCore(callbackWorkerTaskWrapper, "OBD_Connection_callback_task", 4096, this, tskIDLE_PRIORITY,
                                &callbackWorkerTaskHandle, CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create callback task");
        return ESP_FAIL;
    }

    taskCreated = xTaskCreatePinnedToCore(receiveTaskWrapper, "OBD2_receive_task", 8192, this, tskIDLE_PRIORITY + 2,
                                          &ReceiveTaskHandle, CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_FAIL;
    }

    taskCreated = xTaskCreatePinnedToCore(pollTaskWrapper, "OBD2_PollTask", 8192, this, tskIDLE_PRIORITY + 1,
                                          &PollTaskHandle, CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create poll task");
        return ESP_FAIL;
    }

    pollQueue.consumerTask = PollTaskHandle;

    if (canDriver.isBusConnected())
    {
        ESP_LOGI(TAG, "CAN bus already connected, getting supported PIDs");
        handleCanConnected();
    }
    else
    {
        ESP_LOGW(TAG, "CAN bus not connected, waiting for connection...");
    }

    canDriver.setConnectionChangeCallback(onCanStateChange, this);

    ESP_LOGI(TAG, "OBD-II interface initialized");
    return ESP_OK;
}

void OBD2::requestSuppPids()
{
    esp_err_t resetResult = ESP_OK;
    {
        MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
        if (!configurationGuard.isLocked())
            resetResult = ESP_ERR_TIMEOUT;
        else
        {
            resetResult = withPidMapLock(
                [&]() -> esp_err_t
                {
                    pollQueue.clearRecurringMode(MODE_CURRENT_DATA);
                    for (auto& [pid, definition] : PID_DEF)
                    {
                        if (definition.mode() != MODE_CURRENT_DATA)
                            continue;
                        _setDataField(pid, &PIDData_t::isSupported, false);
                        _setDataField(pid, &PIDData_t::isValid, false);
                        _setDataField(pid, &PIDData_t::id, definition.id());
                    }
                    supportedPIDsGroup = {};
                    discoveryActive_ = false;
                    discoveryFailed_ = false;
                    discoveryExpectedGroup_ = 0xFF;
                    discoverySeenGroups_ = 0;
                    pidsInitialized = false;
                    return ESP_OK;
                });
        }
    }

    while (xSemaphoreTake(xRequestNextPIDSemaphore, 0) == pdTRUE)
        ;
    while (xSemaphoreTake(xPidConnectedSemaphore, 0) == pdTRUE)
        ;

    if (resetResult != ESP_OK)
    {
        pidsInitialized = false;
        xSemaphoreGive(xPidConnectedSemaphore);
        return;
    }

    bool terminalSuccess = false;
    bool transitionFailed = false;

    for (uint8_t group = 0; group < SUPPORTED_PIDS_GROUP_COUNT && !terminalSuccess; ++group)
    {
        while (xSemaphoreTake(xRequestNextPIDSemaphore, 0) == pdTRUE)
            ;

        bool opened = false;
        {
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (!configurationGuard.isLocked())
                transitionFailed = true;
            else
            {
                // The active generation and its expected group are published
                // together. Response waits never hold this lock.
                discoveryActive_        = true;
                discoveryExpectedGroup_ = group;
                opened                  = true;
            }
        }

        if (!opened)
            break;

        PollRequest probe         = {};
        probe.id                  = OBD2_FUNCTIONAL_ID;
        probe.isRaw               = true;
        probe.payload.raw.data[0] = 0x02;
        probe.payload.raw.data[1] = MODE_CURRENT_DATA;
        probe.payload.raw.data[2] = (uint8_t)(group << 5);
        probe.payload.raw.dlc     = 8;
        probe.interval            = 0;
        probe.nextWake            = xTaskGetTickCount();
        probe.priority            = 0;
        probe.isRecurring         = false;
        probe.retries_left        = 1;

        if (!pollQueue.push(probe))
        {
            ESP_LOGW(TAG, "Poll queue full; supported-PID probe group %u dropped", group);
            bool closed = false;
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (configurationGuard.isLocked())
            {
                discoveryFailed_        = true;
                discoveryActive_        = false;
                discoveryExpectedGroup_ = 0xFF;
                closed                  = true;
            }
            if (!closed)
                transitionFailed = true;
            break;
        }

        const bool receivedExpected =
            xSemaphoreTake(xRequestNextPIDSemaphore, pdMS_TO_TICKS(SUPPORTED_PID_RESPONSE_TIMEOUT_MS)) == pdTRUE;
        if (!receivedExpected)
        {
            bool closed = false;
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (configurationGuard.isLocked())
            {
                discoveryFailed_        = true;
                discoveryActive_        = false;
                discoveryExpectedGroup_ = 0xFF;
                closed                  = true;
            }
            if (!closed)
                transitionFailed = true;
            break;
        }

        // Keep collecting valid responses for this group so multiple ECUs
        // contribute to the union before the next raw probe is sent.
        while (xSemaphoreTake(xRequestNextPIDSemaphore, pdMS_TO_TICKS(SUPPORTED_PID_RESPONSE_WINDOW_MS)) == pdTRUE)
            ;

        bool windowClosed    = false;
        bool continueDiscovery = false;
        {
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (configurationGuard.isLocked())
            {
                const bool groupSeen = (discoverySeenGroups_ & (1UL << group)) != 0;
                const bool continuation = (supportedPIDsGroup.pidGroup[group] & 0x00000001UL) != 0;

                if (!groupSeen)
                    discoveryFailed_ = true;

                // Close the response generation before deciding whether a
                // subsequent group may be requested.
                discoveryActive_        = false;
                discoveryExpectedGroup_ = 0xFF;
                continueDiscovery       = groupSeen && continuation && group + 1 < SUPPORTED_PIDS_GROUP_COUNT;
                terminalSuccess         = groupSeen && (!continuation || group + 1 == SUPPORTED_PIDS_GROUP_COUNT);
                windowClosed             = true;
            }
        }

        if (!windowClosed)
        {
            transitionFailed = true;
            break;
        }

        if (terminalSuccess || !continueDiscovery)
            break;
    }

    {
        MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
        if (configurationGuard.isLocked())
        {
            discoveryActive_        = false;
            discoveryExpectedGroup_ = 0xFF;
            supportedPIDsGroup.numberOfSupportedPIDs = 0;
            for (const auto& group : supportedPIDsGroup.pidGroup)
                supportedPIDsGroup.numberOfSupportedPIDs += __builtin_popcount(group);

            if (transitionFailed)
                discoveryFailed_ = true;

            const bool complete = terminalSuccess && !discoveryFailed_;
            pidsInitialized = complete;
            if (!complete)
                ESP_LOGW(TAG, "Supported-PID discovery incomplete (seen=0x%08lX)",
                         (unsigned long)discoverySeenGroups_);
        }
        else
        {
            pidsInitialized = false;
        }
    }

    // Wake the connection path on both complete and incomplete terminal
    // outcomes; it must not wait indefinitely for discovery.
    xSemaphoreGive(xPidConnectedSemaphore);
}

void OBD2::getSupportedPids(supportedPIDsGroup_t& supportedPIDsGroup)
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (configurationGuard.isLocked())
        supportedPIDsGroup = this->supportedPIDsGroup;
};

/**
 * @brief Adds PID to the OBD2 Data Model
 * Function that adds a PID to the OBD2 data model at runtime.
 * This allows dynamic configuration of which PIDs to monitor without needing to hardcode them at compile time.
 * The function checks if the PID is supported based on previously retrieved supported PID bitmaps and then adds it to
 * the polling queue if valid. Calls the addPID function of the OBD2DataModel class to maintain the data model and
 * ensure thread safety when accessing PID data.
 *
 * @param id The CAN ID to use for requests related to this PID (e.g., functional or physical ID)
 * @param mode The OBD-II mode (e.g., current data, data by identifier)
 * @param pid The PID number to add (e.g., 0x0C for engine RPM)
 * @param len The expected length of the PID data in bytes
 * @param name A human-readable name for the PID (e.g., "Engine RPM")
 * @param unit The unit of measurement for the PID value (e.g., "RPM", "°C")
 * @param desc The description of the PID, explaining what it represents and how it can be used (e.g., "Current engine
 * revolutions per minute")
 * @param formula A string representing the formula to calculate the actual value from the raw data bytes (e.g.,
 * "((A*256)+B)/4" for engine RPM)
 * @param minV The minimum valid value for the PID
 * @param maxV The maximum valid value for the PID
 * @param priority The priority level for polling this PID (lower number means higher priority)
 * @param interval The desired update interval for this PID in milliseconds
 * @param color A hexadecimal color code for UI representation of this PID (e.g., 0xFF0000 for red)
 * @param icon A string representing the icon name to use for this PID in the UI (e.g., "tachometer" for engine RPM)
 * @return esp_err_t ESP_OK if the PID was successfully added, or an appropriate error code if it failed (e.g.,
 * ESP_ERR_INVALID_ARG if the PID is not supported or already exists)
 */
esp_err_t OBD2::addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                       std::string desc, std::string formula, float minV, float maxV, uint8_t priority,
                       uint16_t interval, uint32_t color, std::string icon)
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return ESP_ERR_TIMEOUT;

    if (interval < MIN_TRANSMIT_PERIOD_MS)
    {
        ESP_LOGW(TAG, "Requested interval %d ms is too low, setting to minimum %d ms", interval,
                 MIN_TRANSMIT_PERIOD_MS);
        interval = MIN_TRANSMIT_PERIOD_MS;
    }

    esp_err_t ret = OBD2DataModel::addPID(id, mode, pid, len, name, unit, desc, formula, minV, maxV, priority, interval,
                                          color, icon);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (mode == MODE_CURRENT_DATA)
    {
        if (pid == 0)
            return ESP_ERR_INVALID_ARG;

        uint8_t groupIdx = ((pid - 1) & 0xE0) >> 5;
        if (groupIdx < SUPPORTED_PIDS_GROUP_COUNT)
        {
            uint8_t bitPos = 31 - ((pid - 1) % 32);

            if (supportedPIDsGroup.pidGroup[groupIdx] & (1UL << bitPos))
            {
                withPidMapLock(
                    [&]()
                    {
                        _setDataField(pid, &PIDData_t::isSupported, true);
                        return ESP_OK;
                    });

                if (continuousRunning)
                {
                    PollRequest request = makePollRequest(id, mode, pid, len, interval, priority, true);
                    enqueueDefinitionRequestLocked(request);
                }
            }
        }
    }
    else if (mode == MODE_READ_DATA_BY_IDENTIFIER || mode == MODE_DERIVED_DATA)
    {
        if (continuousRunning)
        {
            PollRequest request = makePollRequest(id, mode, pid, len, interval, priority, true);
            enqueueDefinitionRequestLocked(request);
        }
    }

    return ESP_OK;
}

esp_err_t OBD2::removePID(uint16_t pid)
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return ESP_ERR_TIMEOUT;
    return OBD2DataModel::removePID(pid);
}

PollRequest OBD2::makePollRequest(uint32_t id, uint8_t mode, uint32_t pid, uint8_t len, uint32_t interval,
                                  uint8_t priority, bool isRecurring)
{
    static std::atomic<uint32_t> staggerOffsetMs{0};

    PollRequest request = {};
    request.isRaw       = false;
    request.payload.obd.mode = mode;
    request.payload.obd.pid  = (uint16_t)pid;
    request.payload.obd.len  = len;
    request.interval          = interval;
    request.priority          = priority;
    request.isRecurring       = isRecurring;
    request.id                = id;
    request.retries_left      = DEFAULT_NUMER_OF_RETRIES;

    uint32_t maxDelay     = (interval > 0 && interval < 15) ? interval : 15;
    uint32_t initialDelay = staggerOffsetMs.fetch_add(7, std::memory_order_relaxed) % maxDelay;
    request.nextWake      = xTaskGetTickCount() + pdMS_TO_TICKS(initialDelay);
    return request;
}

esp_err_t OBD2::replacePIDDefinitions(const std::vector<PIDDefinitionData>& definitions)
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return ESP_ERR_TIMEOUT;

    std::vector<PollRequest> recurringRequests;
    std::vector<uint16_t> supportedCurrentPids;
    recurringRequests.reserve(definitions.size());
    supportedCurrentPids.reserve(definitions.size());
    for (const auto& definition : definitions)
    {
        if (definition.mode == MODE_CURRENT_DATA && definition.pid > 0)
        {
            uint8_t groupIdx = ((definition.pid - 1) & 0xE0) >> 5;
            if (groupIdx < SUPPORTED_PIDS_GROUP_COUNT)
            {
                uint8_t bitPos = 31 - ((definition.pid - 1) % 32);
                if (supportedPIDsGroup.pidGroup[groupIdx] & (1UL << bitPos))
                    supportedCurrentPids.push_back(definition.pid);
            }
        }

        const bool supported =
            definition.mode == MODE_READ_DATA_BY_IDENTIFIER || definition.mode == MODE_DERIVED_DATA ||
            std::find(supportedCurrentPids.begin(), supportedCurrentPids.end(), definition.pid) !=
                supportedCurrentPids.end();
        if (continuousRunning && definition.updateInterval_ms != UPDATE_DISABLED && supported)
        {
            recurringRequests.push_back(makePollRequest(definition.id, definition.mode, definition.pid,
                                                        definition.len, definition.updateInterval_ms,
                                                        definition.priority, true));
        }
    }

    esp_err_t result =
        OBD2DataModel::replacePIDDefinitions(definitions, recurringRequests, supportedCurrentPids);
    if (result == ESP_OK && derivedPidQueue_ != nullptr)
    {
        // The receive task takes the same configuration lock while evaluating
        // derived jobs. Resetting here therefore cannot leave a queued job
        // from the previous definition generation to run after the swap.
        xQueueReset(derivedPidQueue_);
    }
    return result;
}

esp_err_t OBD2::requestPID(uint16_t pid)
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return ESP_ERR_TIMEOUT;

    return withPidMapLock(
        [&]()
        {
            if (!_getDataField(pid, &PIDData_t::isSupported, false))
            {
                ESP_LOGW(TAG, "PID 0x%02X is not supported or not recognized", pid);
                return ESP_ERR_NOT_SUPPORTED;
            }

            uint32_t id       = _getDataField(pid, &PIDData_t::id, (uint32_t)0);
            uint8_t  mode     = _getDefField(pid, &PIDDefinition::mode, (uint8_t)0);
            uint8_t  len      = _getDefField(pid, &PIDDefinition::len, (uint8_t)0);
            uint8_t  priority = _getDefField(pid, &PIDDefinition::priority, (uint8_t)0);

            PollRequest request = makePollRequest(id, mode, pid, len, 0, priority, false);
            return enqueueDefinitionRequestLocked(request);
        });
}

void OBD2::req(uint32_t id, uint8_t mode, uint32_t pid, uint8_t len, uint32_t interval, uint8_t priority,
               bool isRecurring)
{
    PollRequest request = makePollRequest(id, mode, pid, len, interval, priority, isRecurring);
    req(request);
}

void OBD2::req(PollRequest& req)
{
    if (!req.isRaw)
    {
        MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
        if (!configurationGuard.isLocked())
            return;
        enqueueDefinitionRequestLocked(req);
        return;
    }

    pollQueue.push(req);
}

esp_err_t OBD2::enqueueDefinitionRequestLocked(const PollRequest& request)
{
    return pollQueue.push(request) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

esp_err_t OBD2::queryMsg(PollRequest& req)
{
    if (!canDriver.isBusConnected())
    {
        ESP_LOGE(TAG, "OBD-II interface not connected");
        return ESP_ERR_INVALID_STATE;
    }

    if (req.payload.raw.dlc > 8)
    {
        return ESP_ERR_INVALID_ARG;
    }

    CanDriver::CanFrame tx = {};
    uint8_t             dlc;

    // memset(tx.data, 0x55, sizeof(tx.data));

    if (req.isRaw)
    {
        memcpy(tx.data, req.payload.raw.data, req.payload.raw.dlc);
        dlc = req.payload.raw.dlc;
    }
    else
    {
        if (req.payload.obd.mode == MODE_READ_DATA_BY_IDENTIFIER)
        {
            tx.data[0] = req.payload.obd.len;
            tx.data[1] = req.payload.obd.mode;
            tx.data[2] = (req.payload.obd.pid >> 8) & 0xFF;
            tx.data[3] = req.payload.obd.pid & 0xFF;
            dlc        = 8;
        }
        else
        {
            tx.data[0] = req.payload.obd.len;
            tx.data[1] = req.payload.obd.mode;
            tx.data[2] = (uint8_t)req.payload.obd.pid;
            dlc        = 8;
        }
    }

    tx.header.id           = req.id;  // OBD-II  request ID
    tx.header.dlc          = twaifd_len2dlc(dlc);
    tx.header.ide          = false;  // Standard Frame Format (11-bit ID)
    tx.header.rtr          = 0;      // Data frame (not remote frame)
    tx.header.fdf          = 0;      // Classic CAN format
    tx.header.brs          = 0;      // No bit rate switching
    tx.header.esi          = 0;      // No error state indicator
    tx.header.timestamp    = 0;      // Not used for TX
    tx.header.trigger_time = 0;      // Not used for immediate transmission

    tx.length = dlc;

    esp_err_t ret = canDriver.transmit(tx, 100);

    return ret;
}

bool OBD2::isPidInit() const
{
    return pidsInitialized;
}

void OBD2::onCanStateChange(void* arg, bool connected)
{
    OBD2* instance = static_cast<OBD2*>(arg);

    if (connected)
    {
        instance->handleCanConnected();
    }
    else
    {
        instance->handleCanDisconnected();
    }
}

void OBD2::handleCanConnected()
{
    ESP_LOGI(TAG, "CAN bus connected event received");
    xSemaphoreGive(xBusConnectionSemaphore);
    if (!pidsInitialized)
    {
        ESP_LOGI(TAG, "Retrieving supported PIDs...");
        requestSuppPids();
    }

    xSemaphoreTake(xPidConnectedSemaphore, pdMS_TO_TICKS(5000));

    runOBDIIConnectedCallbacks(true);
}

void OBD2::handleCanDisconnected()
{
    ESP_LOGW(TAG, "CAN bus disconnected event received");
    pidsInitialized = false;
    runOBDIIConnectedCallbacks(false);
}

bool OBD2::isContinuousRunning() const
{
    return continuousRunning;
}

void OBD2::startContinuousMode()
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return;
    if (continuousRunning)
        return;

    continuousRunning = true;
    startPollingLocked();
}

void OBD2::stopContinuousMode()
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return;
    if (!continuousRunning)
        return;

    continuousRunning = false;
    pollQueue.clearRecurring();
}

void OBD2::startPolling()
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return;
    startPollingLocked();
}

void OBD2::startPollingLocked()
{
    std::set<uint32_t> RequestByDataIdentifierIds;

    withPidMapLock(
        [&]()
        {
            for (const auto& [pid, info] : PID_DEF)
            {
                if (!_getDataField(pid, &PIDData_t::isSupported, false))
                    continue;

                uint32_t interval = info.updateInterval();
                if (interval == 0)
                    continue;

                PollRequest request = makePollRequest(info.id(), info.mode(), pid, info.len(), interval,
                                                      info.priority(), true);
                enqueueDefinitionRequestLocked(request);

                if (info.mode() == MODE_READ_DATA_BY_IDENTIFIER)
                {
                    RequestByDataIdentifierIds.insert(info.id());
                }
            }
            return ESP_OK;
        });

    // for (const uint32_t id : RequestByDataIdentifierIds)
    // {
    //     requestDefaultDiagnosticSession(id, true);
    // }
}

void OBD2::pollRequestStaticPids()
{
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
        return;

    std::set<uint32_t> RequestByDataIdentifierIds;

    withPidMapLock(
        [&]()
        {
            for (const auto& [pid, def] : PID_DEF)
            {
                if (def.updateInterval() == UPDATE_DISABLED)
                {
                    PollRequest request = makePollRequest(def.id(), def.mode(), pid, def.len(),
                                                          def.updateInterval(), def.priority(), false);
                    enqueueDefinitionRequestLocked(request);
                }

                if (def.mode() == MODE_READ_DATA_BY_IDENTIFIER)
                {
                    RequestByDataIdentifierIds.insert(def.id());
                }
            }
            return ESP_OK;
        });

    // for (const uint32_t id : RequestByDataIdentifierIds)
    // {
    //     requestDefaultDiagnosticSession(id, false);
    // }
}

void OBD2::pollTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
    obd2->pollTask();
}

void OBD2::pollTask()
{
    CanLoadTracker                 busTracker;
    std::map<uint32_t, TickType_t> last_tx_time_per_ecu;

    while (1)
    {
        TickType_t delay = pollQueue.getWait();
        vTaskDelay(delay);

        pollTaskUtilization = busTracker.updateAndGet();

        // Keep the lock order bus -> configuration -> PID map/queue. The
        // non-blocking configuration attempt prevents a poll cycle from
        // holding the bus while a replacement is waiting to commit.
        MutexGuard busGuard(xBusArbitrationMutex, portMAX_DELAY);
        if (!busGuard.isLocked())
            continue;

        MutexGuard configurationGuard(configurationMtx_, 0);
        if (!configurationGuard.isLocked() || !canDriver.isBusConnected())
        {
            // RAII releases configuration first, then bus arbitration, before
            // the next iteration can pop any work.
            continue;
        }

        // 2. Take the most urgent appointment
        PollRequest current = {};
        if (!pollQueue.tryPop(current))
            continue;

        TickType_t now = xTaskGetTickCount();

        // if (!current.isRaw && current.isRecurring)
        // {
        //     uint16_t pid = current.payload.obd.pid;

        if (!current.isRaw && current.isRecurring)
        {
            uint16_t pid = current.payload.obd.pid;

            withPidMapLock(
                [&]()
                {
                    if (_pidExists(pid))
                    {
                        uint32_t last_updated = _getDataField(pid, &PIDData_t::lastUpdated, (uint32_t)0);
                        uint32_t now_ms       = pdTICKS_TO_MS(now);

                        uint32_t threshold = current.interval * 3;
                        if (threshold < 1000)
                            threshold = 1000;

                        if (last_updated == 0 || (now_ms - last_updated > threshold))
                        {
                            if (current.retries_left > 0)
                            {
                                current.retries_left--;
                            }
                            else
                            {
                                _setDataField(pid, &PIDData_t::isValid, false);
                                current.isRecurring = false;
                            }
                        }
                        else
                        {
                            current.retries_left = DEFAULT_NUMER_OF_RETRIES;
                        }
                    }
                    return ESP_OK;
                });
        }

        bool scheduled = false;
        auto schedule = [&](PollRequest& request, const char* reason)
        {
            if (!pollQueue.push(request))
                ESP_LOGW(TAG, "Poll queue full; dropped %s PID 0x%04X", reason, request.payload.obd.pid);
            scheduled = true;
        };

        if (last_tx_time_per_ecu.count(current.id) > 0)
        {
            TickType_t time_since_last = now - last_tx_time_per_ecu[current.id];
            if (time_since_last < pdMS_TO_TICKS(30))
            {
                current.nextWake = last_tx_time_per_ecu[current.id] + pdMS_TO_TICKS(30);
                schedule(current, "deferred");
                continue;
            }
        }

        // 3. Process based on the Mode stored in the request
        if (current.payload.obd.mode == MODE_DERIVED_DATA)
        {
            xQueueSend(derivedPidQueue_, &current.payload.obd.pid, pdMS_TO_TICKS(10));
        }
        else
        {
            esp_err_t err = queryMsg(current);

            if (err == ESP_OK)
            {
                busTracker.recordTx();
                last_tx_time_per_ecu[current.id] = xTaskGetTickCount();
            }
            else if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL)
            {
                if (current.retries_left > 0)
                {
                    current.retries_left--;
                    current.nextWake = xTaskGetTickCount() + pdMS_TO_TICKS(current.interval);
                    schedule(current, "retry");
                }
                else
                {
                    current.isRecurring = false;
                }
            }

        }

        // 4. Reschedule recurring tasks
        if (!scheduled && current.isRecurring && continuousRunning)
        {
            current.nextWake = xTaskGetTickCount() + pdMS_TO_TICKS(current.interval);
            schedule(current, "recurring");
        }

        taskYIELD();
    }
}

void OBD2::receiveTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
    obd2->receiveTask();
}

void OBD2::receiveTask()
{
    TickType_t last_ping_ms      = 0;
    uint8_t    failed_pings      = 0;
    bool       awaiting_response = false;
    TickType_t response_deadline = 0;

    while (1)
    {
        if (!canDriver.isBusConnected())
        {
            xSemaphoreTake(xBusConnectionSemaphore, portMAX_DELAY);

            // Reset health-check state on (re)connect.
            last_ping_ms      = xTaskGetTickCount();
            failed_pings      = 0;
            awaiting_response = false;
            continue;
        }

        // Definition-bound derived work is produced by the polling task and
        // consumed here under the configuration lock. A replacement either
        // waits for this whole drain/evaluation or resets the queue before
        // this task can observe the new model.
        {
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            uint16_t derivedPid;

            while (xQueueReceive(derivedPidQueue_, &derivedPid, 0) == pdTRUE)
            {
                CanDriver::CanFrame f{};
                f.header.id = OBD2_FUNCTIONAL_ID;
                f.length    = 4;
                f.data[0]   = 0x03;  // DLC
                f.data[1]   = RESPONSE_MODE_DERIVED_DATA;
                f.data[2]   = (derivedPid >> 8) & 0xFF;
                f.data[3]   = derivedPid & 0xFF;
                parseRecFrame(f);
            }
        }

        CanDriver::CanFrame f{};
        esp_err_t           ret = canDriver.receive(f, pdMS_TO_TICKS(10));

        if (ret == ESP_OK)
        {
            ret = parseRecFrame(f);

            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to parse received frame: %s", esp_err_to_name(ret));
            }
        }
        else if (ret != ESP_ERR_TIMEOUT)
        {
            ESP_LOGE(TAG, "CAN receive driver error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        multiframe_watchdog();

        // Health check: periodically probe Mode 1 (supported PIDs) and clear the
        // poll queue if the ECU stops responding (e.g. ignition turned off).
        TickType_t now = xTaskGetTickCount();

        if (now - last_ping_ms >= pdMS_TO_TICKS(OBD2_HEALTHCHECK_PING_PERIOD_MS))
        {
            last_ping_ms = now;

            while (xSemaphoreTake(healthCheckSemaphore, 0) == pdTRUE)
                ;

            PollRequest r         = {};
            r.id                  = OBD2_FUNCTIONAL_ID;
            r.isRaw               = true;
            r.payload.raw.data[0] = 0x02;
            r.payload.raw.data[1] = MODE_CURRENT_DATA;
            r.payload.raw.data[2] = PID_PIDS_SUPPORTED_01_20;
            r.payload.raw.dlc     = 8;
            r.interval            = 0;
            r.nextWake            = now;
            r.priority            = 0;
            r.isRecurring         = false;
            r.retries_left        = 1;

            req(r);

            awaiting_response = true;
            response_deadline = now + pdMS_TO_TICKS(HEALTHCHECK_RESPONSE_TIMEOUT_MS);
        }

        if (awaiting_response)
        {
            if (xSemaphoreTake(healthCheckSemaphore, 0) == pdTRUE)
            {
                failed_pings      = 0;
                awaiting_response = false;
            }
            else if (now >= response_deadline)
            {
                failed_pings++;
                ESP_LOGW(TAG, "OBD Health Check timeout (%d/%d)", failed_pings, HEALTHCHECK_RETRIES);

                if (failed_pings >= HEALTHCHECK_RETRIES)
                {
                    ESP_LOGW(TAG, "OBD Health Check failed %d times! Clearing poll queue.", HEALTHCHECK_RETRIES);

                    pollQueue.clear();
                    failed_pings = 0;
                }

                awaiting_response = false;
            }
        }
    }
}

esp_err_t OBD2::parseRecFrame(const CanDriver::CanFrame& f)
{
    if (f.length < 3)
        return ESP_ERR_INVALID_SIZE;

    esp_err_t ret = ESP_OK;
    uint8_t   frame_type =
        (f.data[0] >> 4) &
        0x0F;  // High Nibble - 0 - Sigle Frame, 1 - First Frame, 2 - Consecutive Frame (3 - Flow Control)

    if (frame_type > 0)
    {
        ret = captureMultiFrame(f);
        return ret;
    }

    uint8_t mode = f.data[1];

    switch (mode)
    {
        case RESPONSE_CURRENT_DATA:
            ret = parseCurrentData(f);
            break;
        case RESPONSE_DTCS:
        {
            std::vector<CanDriver::CanFrame> frame = {f};
            ret                                    = parseDTCs(frame, mode);
            break;
        }
        case RESPONSE_CLEAR_DTCS:
            parseClearDTCsAck(f);
            break;
        case RESPONSE_PENDING_DTCS:
        {
            std::vector<CanDriver::CanFrame> frame = {f};
            ret                                    = parseDTCs(frame, mode);
            break;
        }
        case RESPONSE_VEHICLE_INFO:
            // TODO parseVehicleInfo(f);
            break;
        case RESPONSE_PERMANENT_DTCS:
        {
            std::vector<CanDriver::CanFrame> frame = {f};
            ret                                    = parseDTCs(frame, mode);
            break;
        }
        case RESPONSE_MODE_DIAGNOSTIC_SESSION_CONTROL:
            parseRMDSC(f);
            break;
        case RESPONSE_READ_DATA_BY_IDENTIFIER:
            ret = parseRDBI(f);
            break;
        case RESPONSE_MODE_DERIVED_DATA:
            ret = parseDerivedData(f);
            break;
        case RESPONSE_NEGATIVE_RESPONSE_CODE:
            ret = handleNegativeResponse(f);
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t OBD2::handleNegativeResponse(const CanDriver::CanFrame& f)
{
    if (f.length < 4)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t id   = f.header.id;
    uint8_t  mode = f.data[2];
    uint8_t  nrc  = f.data[3];

    bool found = false;
    for (uint8_t i = 0; i < nrc_list_size; i++)
    {
        if (nrc_list[i].can_id == id && nrc_list[i].mode == mode && nrc_list[i].nrc == nrc)
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        if (nrc_list_size < MAX_NRC_LIST_SIZE)
        {
            nrc_list[nrc_list_size++] = {id, mode, nrc};
        }
        else
        {
            for (uint8_t i = 1; i < MAX_NRC_LIST_SIZE; i++)
            {
                nrc_list[i - 1] = nrc_list[i];
            }
            nrc_list[MAX_NRC_LIST_SIZE - 1] = {id, mode, nrc};
        }
    }

    return ESP_OK;
}

esp_err_t OBD2::parseCurrentData(const CanDriver::CanFrame& f)
{
    uint16_t  pid = f.data[2];
    esp_err_t ret;

    switch (pid)
    {
        case PID_PIDS_SUPPORTED_01_20:
        case PID_PIDS_SUPPORTED_21_40:
        case PID_PIDS_SUPPORTED_41_60:
        case PID_PIDS_SUPPORTED_61_80:
        case PID_PIDS_SUPPORTED_81_A0:
        case PID_PIDS_SUPPORTED_A1_C0:
        case PID_PIDS_SUPPORTED_C1_E0:
            ret = parseSupportedPIDs(f);
            break;
        default:
            ret = updateData(f);
    }

    if (ret == ESP_ERR_INVALID_STATE)
        return ret;

    _setDataFieldWithLock(pid, &PIDData_t::isValid, ret == ESP_OK);

    runPidUpdateCallbacks(pid);

    return ret;
}

esp_err_t OBD2::parseRMDSC(const CanDriver::CanFrame& f)
{
    if (f.data[2] != 0x01)
    {
        ESP_LOGW(TAG, "Failed to request Default Extended Diagnostic session for ID 0x%03X", f.header.id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t OBD2::parseRDBI(const CanDriver::CanFrame& f)
{
    uint16_t  pid = (uint16_t)(f.data[2] << 8) | f.data[3];
    esp_err_t ret;

    ret = updateData(f);

    _setDataFieldWithLock(pid, &PIDData_t::isValid, ret == ESP_OK);

    runPidUpdateCallbacks(pid);

    return ret;
}

esp_err_t OBD2::parseDerivedData(const CanDriver::CanFrame& f)
{
    uint16_t pid = (uint16_t)(f.data[2] << 8) | f.data[3];

    esp_err_t ret = withPidMapLock(
        [&]() -> esp_err_t
        {
            PIDData_t* pdat = nullptr;
            esp_err_t  err  = _getData(pid, pdat);

            if (err != ESP_OK || pdat == nullptr)
            {
                _setDataField(pid, &PIDData_t::isValid, false);
                return (err == ESP_OK) ? ESP_ERR_NOT_FOUND : err;
            }

            const PIDDefinition* pdef = nullptr;
            err                       = _getDef(pid, pdef);

            if (err != ESP_OK || pdef == nullptr)
            {
                _setDataField(pid, &PIDData_t::isValid, false);
                return (err == ESP_OK) ? ESP_ERR_NOT_FOUND : err;
            }

            float result = 0.0f;
            err          = pdef->evaluate(nullptr, 0, result);

            pdat->value       = result;
            pdat->id          = OBD2_FUNCTIONAL_ID;
            pdat->lastUpdated = xTaskGetTickCount();
            pdat->isValid     = (err == ESP_OK);

            return err;
        });

    if (ret == ESP_OK)
    {
        runPidUpdateCallbacks(pid);
    }

    return ret;
}

esp_err_t OBD2::parseDTCs(std::vector<CanDriver::CanFrame>& frames, uint8_t mode)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t   dtc_rem = 0, hi_half = 0, idx = 0;
    bool      halved = false;
    esp_err_t ret    = ESP_OK;
    uint16_t  rawDTC;

    for (const auto& frame : frames)
    {
        uint8_t frame_type = (frame.data[0] >> 4) & 0x0F;
        switch (frame_type)
        {
            case 0:  // Single Frame (SF)
            {
                // Lower nibble of Byte 0 is the total payload length
                uint8_t payload_len = frame.data[0] & 0x0F;

                // Subtract 1 for the '0x43' Service ID byte, then divide by 2 bytes per DTC
                dtc_rem = (payload_len - 1) / 2;
                idx     = 2;  // data[0]=PCI, data[1]=0x43, First DTC starts at data[2]!
                break;
            }
            case 1:  // First Frame (FF) of a multi-frame stream
            {
                // 12-bit payload length across Byte 0 and Byte 1
                uint16_t payload_len = ((frame.data[0] & 0x0F) << 8) | frame.data[1];

                dtc_rem = (payload_len - 1) / 2;
                idx     = 3;  // data[0..1]=PCI_Len, data[2]=0x43, First DTC starts at data[3]!
                break;
            }
            case 2:  // Consecutive Frame (CF)
            {
                // data[0] is sequence number (0x20-0x2F), payload starts instantly at data[1]
                idx = 1;
                break;
            }
            default:
                return ESP_ERR_INVALID_RESPONSE;
        }

        while (dtc_rem > 0 && idx < frame.length)
        {
            if (halved)
            {
                rawDTC = (hi_half << 8) | frame.data[idx++];
                ERR_ACCUMULATE(ret, _setDTC(rawDTC, mode));
                dtc_rem--;
                halved = false;
            }
            if (frame.length - idx == 1)
            {
                hi_half = frame.data[idx++];
                halved  = true;
                break;
            }

            if (frame.length - idx >= 2)
            {
                uint8_t  dtc_high = frame.data[idx++];
                uint16_t rawDTC   = (dtc_high << 8) | frame.data[idx++];
                ERR_ACCUMULATE(ret, _setDTC(rawDTC, mode));
                dtc_rem--;
            }
        }

        if (dtc_rem == 0)
        {
            break;
        }
    }

    switch (mode)
    {
        case RESPONSE_DTCS:
            xSemaphoreGive(dtcData.confirmedReadySemaphore);
            break;
        case RESPONSE_PENDING_DTCS:
            xSemaphoreGive(dtcData.pendingReadySemaphore);
            break;
        case RESPONSE_PERMANENT_DTCS:
            xSemaphoreGive(dtcData.permanentReadySemaphore);
            break;
        default:
            break;
    }

    return ret;
}

esp_err_t OBD2::parseSupportedPIDs(const CanDriver::CanFrame& f)
{
    uint16_t pidGroup = f.data[2];
    if (pidGroup % 0x20 != 0)
    {
        ESP_LOGE(TAG, "Invalid PID group: 0x%02X", pidGroup);
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t groupIdx = pidGroup >> 5;
    if (groupIdx >= SUPPORTED_PIDS_GROUP_COUNT)
    {
        ESP_LOGE(TAG, "Unsupported PID group: 0x%02X", pidGroup);
        return ESP_ERR_INVALID_ARG;
    }

    if (f.length < 7)
        return ESP_OK;

    uint32_t reportedPIDs = (f.data[3] << 24) | (f.data[4] << 16) | (f.data[5] << 8) | (f.data[6]);

    MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
    if (!configurationGuard.isLocked())
        return ESP_ERR_TIMEOUT;

    if (!discoveryActive_ || discoveryExpectedGroup_ != groupIdx)
    {
        // A late response must not mutate an inactive or different discovery
        // generation. Health-check waiters may still use this valid frame.
        if (healthCheckSemaphore != nullptr)
            xSemaphoreGive(healthCheckSemaphore);
        return ESP_OK;
    }

    PollRequest recurringRequests[32];
    size_t      recurringCount = 0;
    const uint16_t firstPID    = pidGroup + 1;
    const uint16_t lastPID     = pidGroup + 32;
    const uint32_t responderID = f.header.id - RESPONSE_ID_OFFSET;
    const uint32_t mergedPIDs  = supportedPIDsGroup.pidGroup[groupIdx] | reportedPIDs;

    esp_err_t ret = withPidMapLock(
        [&]() -> esp_err_t
        {
            // Stage all desired recurring work before changing support flags.
            // The queue operation below is allocation-free and capacity
            // checked, so a failure leaves both the queue and model unchanged.
            for (const auto& [installedPID, definition] : PID_DEF)
            {
                if (definition.mode() != MODE_CURRENT_DATA || installedPID < firstPID || installedPID > lastPID)
                    continue;

                const uint8_t bit = (uint8_t)(installedPID - firstPID);
                if (continuousRunning && (mergedPIDs & (1UL << (31 - bit))) &&
                    definition.updateInterval() != UPDATE_DISABLED)
                {
                    recurringRequests[recurringCount++] =
                        makePollRequest(definition.id(), definition.mode(), installedPID, definition.len(),
                                        definition.updateInterval(), definition.priority(), true);
                }
            }

            if (!pollQueue.replaceRecurringPidRange(firstPID, lastPID, MODE_CURRENT_DATA, recurringRequests,
                                                    recurringCount))
            {
                discoveryFailed_ = true;
                return ESP_ERR_INVALID_SIZE;
            }

            for (const auto& [installedPID, definition] : PID_DEF)
            {
                if (definition.mode() != MODE_CURRENT_DATA || installedPID < firstPID || installedPID > lastPID)
                    continue;

                const bool supported = (mergedPIDs & (1UL << (31 - (installedPID - firstPID)))) != 0;
                const bool wasSupported = _getDataField(installedPID, &PIDData_t::isSupported, false);
                _setDataField(installedPID, &PIDData_t::isSupported, supported);
                _setDataField(installedPID, &PIDData_t::isValid, false);
                if (supported && !wasSupported)
                    _setDataField(installedPID, &PIDData_t::id, responderID);
                else if (!supported)
                    _setDataField(installedPID, &PIDData_t::id, definition.id());
            }

            supportedPIDsGroup.pidGroup[groupIdx] = mergedPIDs;
            discoverySeenGroups_ |= (1UL << groupIdx);
            supportedPIDsGroup.numberOfSupportedPIDs = 0;
            for (const auto& group : supportedPIDsGroup.pidGroup)
                supportedPIDsGroup.numberOfSupportedPIDs += __builtin_popcount(group);
            return ESP_OK;
        });

    if (ret != ESP_OK)
        return ret;

    xSemaphoreGive(xRequestNextPIDSemaphore);

    if (healthCheckSemaphore != nullptr)
    {
        xSemaphoreGive(healthCheckSemaphore);
    }

    return ret;
}

esp_err_t OBD2::parseClearDTCsAck(const CanDriver::CanFrame& f)
{
    if (f.length < 3 || f.data[1] != RESPONSE_CLEAR_DTCS)
    {
        ESP_LOGE(TAG, "Invalid frame for Clear DTCs ACK: length=%d, mode=0x%02X", f.length, f.data[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    clearDTC(MODE_DTCS);
    if (dtcData.clearReadySemaphore != NULL)
    {
        xSemaphoreGive(dtcData.clearReadySemaphore);
    }
    return ESP_OK;
}

void OBD2::requestDefaultDiagnosticSession(uint32_t id, bool isRecurring)
{
    PollRequest r         = {};
    r.id                  = id;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x02;
    r.payload.raw.data[1] = MODE_DIAGNOSTIC_SESSION_CONTROL;
    r.payload.raw.data[2] = 0x01;
    r.payload.raw.dlc     = 8;
    r.interval            = isRecurring ? 2000 : 0;
    r.nextWake            = 0;
    r.priority            = 0;
    r.isRecurring         = isRecurring;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;
    req(r);
}

esp_err_t OBD2::requestVIN()
{
    if (vinData.vinReadySemaphore == NULL)
    {
        ESP_LOGE(TAG, "VIN semaphore not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    PollRequest r         = {};
    r.id                  = OBD2_FUNCTIONAL_ID;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x02;
    r.payload.raw.data[1] = MODE_VEHICLE_INFO;
    r.payload.raw.data[2] = PID_VIN;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    req(r);

    xSemaphoreTake(vinData.vinReadySemaphore, 0);

    if (xSemaphoreTake(vinData.vinReadySemaphore, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t OBD2::requestDTC(uint8_t mode)
{
    SemaphoreHandle_t* sem = NULL;

    switch (mode)
    {
        case MODE_DTCS:
            sem = &dtcData.confirmedReadySemaphore;
            break;
        case MODE_PENDING_DTCS:
            sem = &dtcData.pendingReadySemaphore;
            break;
        case MODE_PERMANENT_DTCS:
            sem = &dtcData.permanentReadySemaphore;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (*sem == NULL)
    {
        ESP_LOGE(TAG, "Semaphore for mode %02x not initialized!", mode);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(clearDTC(mode), TAG, "Failed to clear DTCs");

    // Drain any stale tokens before requesting
    while (xSemaphoreTake(*sem, 0) == pdTRUE)
        ;

    PollRequest r         = {};
    r.id                  = OBD2_FUNCTIONAL_ID;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x01;
    r.payload.raw.data[1] = mode;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    req(r);

    if (xSemaphoreTake(*sem, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        ESP_LOGW(TAG, "DTC Request %s timed out", OBD2_MODE_TO_STR(mode));
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t OBD2::requestClearDTCs()
{
    if (dtcData.clearReadySemaphore == NULL)
    {
        ESP_LOGE(TAG, "Semaphore for clear DTCs not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    // Drain any stale tokens
    while (xSemaphoreTake(dtcData.clearReadySemaphore, 0) == pdTRUE)
        ;

    PollRequest r         = {};
    r.id                  = OBD2_FUNCTIONAL_ID;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x02;
    r.payload.raw.data[1] = MODE_CLEAR_DTCS;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    req(r);

    if (xSemaphoreTake(dtcData.clearReadySemaphore, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Clear DTCs Request timed out");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void OBD2::multiframe_watchdog()
{
    if (multiframe_state != 99)
    {
        if ((xTaskGetTickCount() - last_multiframe_received) > pdMS_TO_TICKS(250))
        {
            ESP_LOGE(TAG, "Multiframe session timeout");
            xSemaphoreGive(xBusArbitrationMutex);
            multiframe_state = 99;
        }
    }
}

esp_err_t OBD2::captureMultiFrame(const CanDriver::CanFrame& f)
{
    uint8_t                                 frameType;
    static uint16_t                         totalLength = 0, consecutiveFrameIndex = 0, consecutiveFramesNeeded = 0;
    constexpr uint8_t                       CONSECUTIVE_FRAME_DATA_BYTES = 7, FIRST_FRAME_DATA_BYTES = 6;
    static std::vector<CanDriver::CanFrame> multiFrameBuffer;
    esp_err_t                               ret = ESP_OK;

    last_multiframe_received = xTaskGetTickCount();

    switch (multiframe_state)
    {
        case 99:  // Idle State
        {
            frameType = (f.data[0] >> 4) & 0x0F;
            if (frameType == 1)
            {
                multiframe_state = 0;  // Go to First Frame state
            }
            else
            {
                ESP_LOGE(TAG, "Unexpected multi-frame type: %d", frameType);
                return ESP_ERR_INVALID_RESPONSE;
            }
            captureMultiFrame(f);
            break;
        }
        case 0:  // First Frame
        {
            frameType               = (f.data[0] >> 4) & 0x0F;  // High nibble = 1
            uint8_t lengthHigh      = f.data[0] & 0x0F;         // Low nibble = 0
            uint8_t lengthLow       = f.data[1];
            totalLength             = (lengthHigh << 8) | lengthLow;
            consecutiveFramesNeeded = (totalLength - FIRST_FRAME_DATA_BYTES + CONSECUTIVE_FRAME_DATA_BYTES - 1) /
                                      CONSECUTIVE_FRAME_DATA_BYTES;
            consecutiveFrameIndex   = 0;
            multiFrameBuffer.clear();
            multiFrameBuffer.push_back(f);
            xSemaphoreTake(xBusArbitrationMutex, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(10));  // Wait before sending Flow Control
            sendFlowControlFrame(f.header.id - RESPONSE_ID_OFFSET);
            multiframe_state = 1;  // Go to Consecutive Frame state
            break;
        }
        case 1:  // Consecutive Frames
        {
            consecutiveFrameIndex++;
            frameType        = (f.data[0] >> 4) & 0x0F;
            uint8_t sequence = f.data[0] & 0x0F;
            if (frameType != 2 || sequence != consecutiveFrameIndex)
            {
                ESP_LOGW(TAG, "Unexpected consecutive frame. Expected seq: %d, got: %d", consecutiveFrameIndex & 0x0F,
                         sequence);
                xSemaphoreGive(xBusArbitrationMutex);

                multiframe_state = 99;
                return ESP_ERR_INVALID_RESPONSE;
            }
            multiFrameBuffer.push_back(f);
            if (consecutiveFrameIndex >= consecutiveFramesNeeded)
            {
                xSemaphoreGive(xBusArbitrationMutex);

                multiframe_state = 99;
                ret              = parseMultiFrame(multiFrameBuffer);
            }
            break;
        }
        default:
            multiframe_state = 99;
            xSemaphoreGive(xBusArbitrationMutex);
            ret = ESP_ERR_INVALID_STATE;
            break;
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to parse multi-frame: %s", esp_err_to_name(ret));
        multiframe_state = 99;
    }
    return ret;
}

inline void OBD2::sendFlowControlFrame(uint32_t id)
{
    PollRequest r         = {};
    r.id                  = id;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x30;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    queryMsg(r);
}

esp_err_t OBD2::parseMultiFrame(std::vector<CanDriver::CanFrame>& frames)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    uint8_t mode = frames[0].data[2];

    switch (mode)
    {
        case RESPONSE_CURRENT_DATA:
            // NOT IMPLEMENTED
            break;
        case RESPONSE_DTCS:
            ret = parseDTCs(frames, mode);
            break;
        case RESPONSE_CLEAR_DTCS:
            // NOT IMPLEMENTED
            break;
        case RESPONSE_PENDING_DTCS:
            ret = parseDTCs(frames, mode);
            break;
        case RESPONSE_VEHICLE_INFO:
            ret = parseVehicleInfoMultiFrame(frames);
            break;
        case RESPONSE_PERMANENT_DTCS:
            ret = parseDTCs(frames, mode);
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t OBD2::parseVehicleInfoMultiFrame(std::vector<CanDriver::CanFrame>& frames)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    uint16_t pid = frames[0].data[3];

    switch (pid)
    {
        case PID_VIN:
            ret = parseVINMultiFrame(frames);
            break;
        default:
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }
    return ret;
}

esp_err_t OBD2::parseVINMultiFrame(std::vector<CanDriver::CanFrame>& frames)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    uint8_t vinBuffer[VIN_LENGTH] = {0};
    size_t  vinIndex              = 0;

    for (const auto& frame : frames)
    {
        uint8_t startIdx = (frame.data[0] >> 4) == 1 ? 5 : 1;  // First frame starts data at index 5
        for (uint8_t i = startIdx; i < frame.length && vinIndex < VIN_LENGTH; i++)
        {
            vinBuffer[vinIndex++] = frame.data[i];
        }
    }

    if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        xSemaphoreGive(vinData.vinReadySemaphore);
        return ESP_ERR_TIMEOUT;
    }

    if (vinIndex == VIN_LENGTH)
    {
        memcpy(vinData.vin, vinBuffer, VIN_LENGTH);
        vinData.isValid = true;
    }
    else
    {
        vinData.isValid = false;
        ret             = ESP_ERR_INVALID_SIZE;
    }

    vinData.lastUpdated = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(vinData.vinReadySemaphore);
    xSemaphoreGive(vinData.mtx_);

    return ret;
}

void OBD2::connected_subscribe(OBDIIConnectedCallback cb)
{
    bool locked = (connected_subscribers_mtx_ != nullptr) &&
                  (xSemaphoreTake(connected_subscribers_mtx_, portMAX_DELAY) == pdTRUE);
    connected_subscribers_.push_back(std::move(cb));
    if (locked)
        xSemaphoreGive(connected_subscribers_mtx_);
}

void OBD2::runOBDIIConnectedCallbacks(bool connected)
{
    xQueueSend(event_queue, &connected, 0);
}

void OBD2::callbackWorkerTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
    obd2->callbackWorkerTask();
}

void OBD2::callbackWorkerTask()
{
    bool is_connected;

    while (true)
    {
        if (xQueueReceive(event_queue, &is_connected, portMAX_DELAY) == pdTRUE)
        {
            std::vector<OBDIIConnectedCallback> callbacks;

            if (connected_subscribers_mtx_ != nullptr &&
                xSemaphoreTake(connected_subscribers_mtx_, portMAX_DELAY) == pdTRUE)
            {
                callbacks = connected_subscribers_;
                xSemaphoreGive(connected_subscribers_mtx_);
            }

            for (const auto& cb : callbacks)
            {
                cb(is_connected);
            }
        }
    }
}
