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
#include <vector>
#include <algorithm>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_check.h"

#include "obd2_utils.hpp"
#include "obd2_dtb.hpp"
#include "can_driver.hpp"

#define POLL_TASK_PERIOD_MS MIN_TRANSMIT_PERIOD_MS
#define ERR_ACCUMULATE(result, expr) ((result) = (result) ?: (expr))

class OBD2 : public OBD2DTB
{
public:
    OBD2();
    ~OBD2();

    static OBD2 &getInstance()
    {
        static OBD2 instance;
        return instance;
    }

    esp_err_t init();

    bool isPidInit() const;
    esp_err_t req(uint8_t pid);

    void startContinuousMode();
    void stopContinuousMode();
    bool isContinuousRunning() const;

    void requestSuppPids();
    esp_err_t requestVIN();
    esp_err_t requestConfirmedDTCs();
    esp_err_t requestPendingDTCs();
    esp_err_t requestPermanentDTCs();
    esp_err_t requestDTC(uint8_t mode);
    esp_err_t requestClearDTCs();
    esp_err_t queryMsg(uint32_t id, uint8_t mode, uint8_t pid, uint8_t len = 0x02);

private:
    OBD2(const OBD2 &) = delete;
    OBD2 &operator=(const OBD2 &) = delete;
    CanDriver &canDriver = CanDriver::getInstance();
    bool continuousRunning;

    // PID Definitions and Data Storage

    SemaphoreHandle_t xPidConnectedSemaphore = NULL;
    esp_err_t setPidSuppStatus(uint8_t groupIndex, bool supported);

    // Polling Task
    void pollTask();
    void pollStatic();
    static void pollTaskWrapper(void *param);
    TaskHandle_t PollTaskHandle{nullptr};
    std::atomic<bool> pollStaticGroup = false;

    // Receiving Task
    void receiveTask();
    static void receiveTaskWrapper(void *param);
    TaskHandle_t ReceiveTaskHandle{nullptr};

    // Callback
    bool pidsInitialized{false};

    static void onCanStateChange(void *arg, bool connected);

    // Handle connection events
    void handleCanConnected();
    void handleCanDisconnected();
    SemaphoreHandle_t xBusConnectionSemaphore = NULL;
    SemaphoreHandle_t xRequestNextPIDSemaphore = NULL;

    // Frame Parsing
    esp_err_t parseCurrentData(const CanDriver::CanFrame &f);
    esp_err_t parseDTCs(std::vector<CanDriver::CanFrame> &frames, uint8_t mode);

    esp_err_t parseRecFrame(const CanDriver::CanFrame &f);
    esp_err_t parseSupportedPIDs(const CanDriver::CanFrame &f);
    esp_err_t captureMultiFrame(const CanDriver::CanFrame &f);
    esp_err_t parseMultiFrame(std::vector<CanDriver::CanFrame> &frames);
    esp_err_t parseVehicleInfoMultiFrame(std::vector<CanDriver::CanFrame> &frames);
    esp_err_t parseVINMultiFrame(std::vector<CanDriver::CanFrame> &frames);
    inline esp_err_t sendFlowControlFrame(uint32_t id);
};
