// obd2.hpp
#pragma once

#include <math.h>
#include "obd2_utils.hpp"
#include "can_driver.hpp"
#include "esp_err.h"

class OBD2
{
public:
    // TODO can_driver callback for connection lost
    explicit OBD2(CanDriver &CanDriver);
    ~OBD2();

    esp_err_t init();
    esp_err_t getSuppPids(uint8_t pidGroup);
    bool isSup(uint8_t pid);
    bool pidExists(uint8_t pid) const;
    esp_err_t req(uint8_t pid);

    uint8_t getmode(uint8_t pid) const;
    const char *getName(uint8_t pid) const;
    const char *getUnit(uint8_t pid) const;
    const char *getDescription(uint8_t pid) const;
    float getMinValue(uint8_t pid) const;
    float getMaxValue(uint8_t pid) const;
    uint8_t getPriority(uint8_t pid) const;

    float getValue(uint8_t pid) const;
    uint32_t getLastUpdated(uint8_t pid) const;
    esp_err_t getData(uint8_t pid, uint8_t *outData) const;
    uint16_t getUpdateInterval(uint8_t pid) const;
    bool isValid(uint8_t pid) const;

    esp_err_t setUpdateInterval(uint8_t pid, uint16_t interval_ms);
    esp_err_t setValid(uint8_t pid, bool valid);
    esp_err_t setIsSupported(uint8_t pid, bool supported);

private:
    CanDriver &canDriver;
    bool continuousRunning;
    mutable SemaphoreHandle_t mtx_;

    static const std::map<uint8_t, PIDInfo_t> PID_DEF;
    std::map<uint8_t, PIDData_t> pidData;

    void initDef();

    esp_err_t queryMsg(uint8_t mode, uint8_t pid, uint8_t len, CanDriver::CanFrame &rxFrame, uint32_t timeout_ms = 1000);

    esp_err_t updateData(uint8_t pid, const CanDriver::CanFrame &frame);
    esp_err_t getData(uint8_t pid, PIDData_t &pd) const;

    // Callback
    bool pidsInitialized{false};

    static void onCanStateChange(void *arg, bool connected);

    // Handle connection events
    void handleCanConnected();
    void handleCanDisconnected();
};
