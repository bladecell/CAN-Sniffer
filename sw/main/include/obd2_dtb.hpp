// obd2_dtb.hpp
#pragma once

#include <math.h>
#include <vector>
#include <algorithm>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "obd2_utils.hpp"
#include "utilities.h"

class OBD2DTB
{
public:
    void initDef();
    bool isSup(uint8_t pid) const;
    bool pidExists(uint8_t pid) const;

    uint8_t getmode(uint8_t pid) const;
    const char *getName(uint8_t pid) const;
    const char *getUnit(uint8_t pid) const;
    const char *getDescription(uint8_t pid) const;
    float getMinValue(uint8_t pid) const;
    float getMaxValue(uint8_t pid) const;
    uint8_t getPriority(uint8_t pid) const;
    std::string getVIN() const;
    std::vector<std::string> getDTC(uint8_t mode);

    float getValue(uint8_t pid, uint32_t timeout_ms = 500) const;
    uint32_t getLastUpdated(uint8_t pid) const;
    uint32_t getId(uint8_t pid) const;
    esp_err_t getRawData(uint8_t pid, uint8_t *outData) const;
    uint16_t getUpdateInterval(uint8_t pid) const;
    bool isValid(uint8_t pid) const;

    esp_err_t setUpdateInterval(uint8_t pid, UpdateRate interval_ms);
    esp_err_t setValid(uint8_t pid, bool valid);
    esp_err_t setIsSupported(uint8_t pid, bool supported);
    esp_err_t setLastUpdated(uint8_t pid, uint32_t lastUpdated);
    esp_err_t setId(uint8_t pid, uint32_t id);
    esp_err_t setDTC(uint16_t rawDTC, uint8_t mode);
    esp_err_t clearDTC(uint8_t mode);
    std::string decodeDTC(uint16_t rawDTC);

    // PID Definitions and Data Storage
    static const std::map<uint8_t, PIDInfo_t> PID_DEF;
    std::map<uint8_t, PIDData_t> pidData;
    VINData_t vinData;
    DTCData_t dtcData;

    esp_err_t updateData(const CanDriver::CanFrame &frame);
    esp_err_t getData(uint8_t pid, PIDData_t &pd) const;

    void generatePollingGroups();
    std::vector<uint8_t> vGroupFast;
    std::vector<uint8_t> vGroupMedium;
    std::vector<uint8_t> vGroupSlow;
    std::vector<uint8_t> vGroupStatic;
};