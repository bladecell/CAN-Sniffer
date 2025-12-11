// obd2.hpp
#pragma once

#include <math.h>
#include <vector>
#include <algorithm>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "obd2_utils.hpp"
#include "obd2_dtb.hpp"
#include "can_driver.hpp"
#include "utilities.h"

#define POLL_TASK_PERIOD_MS MIN_TRANSMIT_PERIOD_MS
#define ERR_ACCUMULATE(result, expr) ((result) = (result) ?: (expr))

class OBD2 : public OBD2DTB
{
public:
    // TODO can_driver callback for connection lost
    explicit OBD2(CanDriver &CanDriver);
    ~OBD2();

    esp_err_t init();
    void getSuppPids();
    bool isPidInit() const;
    esp_err_t req(uint8_t pid);

    void startContinuousMode();
    void stopContinuousMode();

    esp_err_t requestVIN();
    esp_err_t requestConfirmedDTCs();
    esp_err_t requestPendingDTCs();
    esp_err_t requestPermanentDTCs();

private:
    CanDriver &canDriver;
    bool continuousRunning;

    // PID Definitions and Data Storage

    SemaphoreHandle_t xPidConnectedSemaphore = NULL;
    SemaphoreHandle_t xPidRequestSemaphoreCounting = NULL;
    esp_err_t setPidSuppStatus(uint8_t groupIndex, bool supported);

    esp_err_t queryMsg(uint32_t id, uint8_t mode, uint8_t pid, uint8_t len = 0x02);

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

    // Frame Parsing
    esp_err_t parseCurrentData(const CanDriver::CanFrame &f);
    esp_err_t parseDTCs(std::vector<CanDriver::CanFrame> &frames, uint8_t mode);
    esp_err_t decodeDTC(uint8_t hi, uint8_t lo);
    esp_err_t parseRecFrame(const CanDriver::CanFrame &f);
    esp_err_t parseSupportedPIDs(const CanDriver::CanFrame &f);
    esp_err_t captureMultiFrame(const CanDriver::CanFrame &f);
    esp_err_t parseMultiFrame(std::vector<CanDriver::CanFrame> &frames);
    esp_err_t parseVehicleInfoMultiFrame(std::vector<CanDriver::CanFrame> &frames);
    esp_err_t parseVINMultiFrame(std::vector<CanDriver::CanFrame> &frames);
    inline esp_err_t sendFlowControlFrame(uint32_t id);
};
