/**
 * @file obd2.hpp
 * @author bladecell (github@bum.anonaddy.com)
 * @brief
 * @version 0.1
 * @date 2025-12-15
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "can_driver.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "obd2_data_model.hpp"
#include "obd2_utils.hpp"

#define POLL_TASK_PERIOD_MS MIN_TRANSMIT_PERIOD_MS
#define ERR_ACCUMULATE(result, expr) ((result) = (result) ?: (expr))

class OBD2 : public OBD2DataModel
{
public:
    OBD2();
    ~OBD2();

    static OBD2& getInstance()
    {
        static OBD2 instance;
        return instance;
    }

    esp_err_t init();
    void      deinit();

    bool isPidInit() const;

    esp_err_t addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                     std::string desc, std::string formula, float minV, float maxV, uint8_t priority, uint16_t interval,
                     uint32_t color, std::string icon) override;

    void      startContinuousMode();
    void      stopContinuousMode();
    bool      isContinuousRunning() const;
    esp_err_t requestPID(uint16_t pid);

    void      requestSuppPids();
    void      getSupportedPids(supportedPIDsGroup_t& supportedPIDsGroup);
    esp_err_t requestVIN();
    esp_err_t requestDTC(uint8_t mode);
    esp_err_t requestClearDTCs();
    void      startPolling();
    void      pollRequestStaticPids();
    void      requestDefaultDiagnosticSession(uint32_t id, bool isRecurring = false);

    void req(uint32_t id, uint8_t mode, uint32_t pid, uint8_t len, uint32_t interval, uint8_t priority,
             bool isRecurring = false);
    void req(PollRequest& req);

    float getPollTaskUtilization() const
    {
        return pollTaskUtilization;
    }

    using OBDIIConnectedCallback = std::function<void(bool connected)>;
    void connected_subscribe(OBDIIConnectedCallback cb);

private:
    OBD2(const OBD2&)                 = delete;
    OBD2&      operator=(const OBD2&) = delete;
    CanDriver& canDriver              = CanDriver::getInstance();
    bool       continuousRunning;

    // PID Definitions and Data Storage

    TaskHandle_t ReceiveTaskHandle        = nullptr;
    TaskHandle_t PollTaskHandle           = nullptr;
    TaskHandle_t callbackWorkerTaskHandle = nullptr;
    TaskHandle_t obdHealthCheckTaskHandle = nullptr;

    std::vector<OBDIIConnectedCallback> connected_subscribers_;

    SemaphoreHandle_t connected_subscribers_mtx_ = nullptr;

    QueueHandle_t     derivedPidQueue_         = nullptr;
    SemaphoreHandle_t xPidConnectedSemaphore   = nullptr;
    SemaphoreHandle_t xBusConnectionSemaphore  = nullptr;
    SemaphoreHandle_t xBusArbitrationMutex     = nullptr;
    SemaphoreHandle_t xRequestNextPIDSemaphore = nullptr;
    SemaphoreHandle_t healthCheckSemaphore     = nullptr;

    PIDPriorityQueue pollQueue;

    uint8_t  multiframe_state         = 99;
    uint32_t last_multiframe_received = 0;

    void multiframe_watchdog();

    esp_err_t queryMsg(PollRequest& req);

    // Polling Task
    void        pollTask();
    static void pollTaskWrapper(void* param);
    float       pollTaskUtilization = 0.0f;

    // Receiving Task
    void        receiveTask();
    static void receiveTaskWrapper(void* param);

    // Health Check Task
    void        obdHealthCheckTask();
    static void obdHealthCheckTaskWrapper(void* param);

    // Callback
    bool pidsInitialized{false};

    static void onCanStateChange(void* arg, bool connected);

    // Handle connection events
    void handleCanConnected();
    void handleCanDisconnected();

    // Frame Parsing
    esp_err_t parseCurrentData(const CanDriver::CanFrame& f);
    esp_err_t parseDTCs(std::vector<CanDriver::CanFrame>& frames, uint8_t mode);

    esp_err_t   parseRecFrame(const CanDriver::CanFrame& f);
    esp_err_t   parseSupportedPIDs(const CanDriver::CanFrame& f);
    esp_err_t   parseClearDTCsAck(const CanDriver::CanFrame& f);
    esp_err_t   captureMultiFrame(const CanDriver::CanFrame& f);
    esp_err_t   parseMultiFrame(std::vector<CanDriver::CanFrame>& frames);
    esp_err_t   parseVehicleInfoMultiFrame(std::vector<CanDriver::CanFrame>& frames);
    esp_err_t   parseVINMultiFrame(std::vector<CanDriver::CanFrame>& frames);
    esp_err_t   parseRDBI(const CanDriver::CanFrame& f);
    esp_err_t   parseDerivedData(const CanDriver::CanFrame& f);
    esp_err_t   parseRMDSC(const CanDriver::CanFrame& f);
    esp_err_t   handleNegativeResponse(const CanDriver::CanFrame& f);
    inline void sendFlowControlFrame(uint32_t id);

    supportedPIDsGroup_t supportedPIDsGroup = {};

    void runOBDIIConnectedCallbacks(bool connected);

    struct CallbackTaskArgs
    {
        OBD2* instance;
        bool  connected;
    };

    void        callbackWorkerTask();
    static void callbackWorkerTaskWrapper(void* param);

    QueueHandle_t event_queue = nullptr;
};
