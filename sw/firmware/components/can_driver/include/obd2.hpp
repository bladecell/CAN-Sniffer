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

#include <math.h>
#include <sys/types.h>

#include <algorithm>
#include <vector>

#include "can_driver.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "obd2_dtb.hpp"
#include "obd2_utils.hpp"

#define POLL_TASK_PERIOD_MS MIN_TRANSMIT_PERIOD_MS
#define ERR_ACCUMULATE(result, expr) ((result) = (result) ?: (expr))

class OBD2 : public OBD2DTB
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

    bool isPidInit() const;

    esp_err_t addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                     std::string desc, std::string formula, float minV, float maxV, uint8_t priority,
                     UpdateRate interval, uint32_t color, std::string icon) override;

    void      startContinuousMode();
    void      stopContinuousMode();
    bool      isContinuousRunning() const;
    esp_err_t requestPID(uint16_t pid);

    void      requestSuppPids();
    void      getSupportedPids(supportedPIDsGroup_t& supportedPIDsGroup);
    esp_err_t requestVIN();
    void      requestConfirmedDTCs();
    void      requestPendingDTCs();
    void      requestPermanentDTCs();
    esp_err_t requestDTC(uint8_t mode);
    void      requestClearDTCs();
    esp_err_t queryMsg(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len);

    float getPollTaskUtilization() const
    {
        return pollTaskUtilization;
    }

private:
    OBD2(const OBD2&)                 = delete;
    OBD2&      operator=(const OBD2&) = delete;
    CanDriver& canDriver              = CanDriver::getInstance();
    bool       continuousRunning;

    // PID Definitions and Data Storage

    SemaphoreHandle_t xPidConnectedSemaphore = NULL;

    // Polling Task
    void              pollTask();
    void              pollStatic();
    static void       pollTaskWrapper(void* param);
    TaskHandle_t      PollTaskHandle{nullptr};
    std::atomic<bool> pollStaticGroup = false;
    void  req(uint32_t id, uint8_t mode, uint32_t pid, uint8_t len, uint8_t priority, bool isRecurring = false);
    float pollTaskUtilization = 0.0f;

    // Receiving Task
    void         receiveTask();
    static void  receiveTaskWrapper(void* param);
    TaskHandle_t ReceiveTaskHandle{nullptr};

    // Callback
    bool pidsInitialized{false};

    static void onCanStateChange(void* arg, bool connected);

    // Handle connection events
    void              handleCanConnected();
    void              handleCanDisconnected();
    SemaphoreHandle_t xBusConnectionSemaphore  = NULL;
    SemaphoreHandle_t xRequestNextPIDSemaphore = NULL;

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
    inline void sendFlowControlFrame(uint32_t id);

    supportedPIDsGroup_t supportedPIDsGroup = {};

    // Derived PIDs queue
    QueueHandle_t derivedPidQueue_ = nullptr;
};

// Get rid of OBD UTILS type definitions and include them in obd2 or obd2datamodel