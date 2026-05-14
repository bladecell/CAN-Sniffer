// obd2_dtb.hpp
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "can_driver.hpp"
#include "esp_err.h"
#include "obd2_utils.hpp"
#include "pid_def.hpp"
#include "pid_priority_queue.hpp"

class OBD2DataModel
{
public:
    void initDef();
    bool pidExists(uint16_t pid) const;
    // PID_DEF Getters
    uint32_t    getId_Def(uint16_t pid) const;
    uint8_t     getMode(uint16_t pid) const;
    uint8_t     getLen(uint16_t pid) const;
    std::string getName(uint16_t pid) const;
    std::string getUnit(uint16_t pid) const;
    std::string getDescription(uint16_t pid) const;
    float       getMinValue(uint16_t pid) const;
    float       getMaxValue(uint16_t pid) const;
    uint8_t     getPriority(uint16_t pid) const;
    uint16_t    getUpdateInterval(uint16_t pid) const;
    uint32_t    getColor(uint16_t pid) const;
    std::string getIcon(uint16_t pid) const;
    std::string getFormula(uint16_t pid) const;
    // PIDDefinition *getDef(uint16_t pid) const;
    esp_err_t getDef(uint16_t pid, const PIDDefinition*& outDef) const;

    // Special Getters
    std::string              getVIN() const;
    std::vector<std::string> getDTC(uint8_t mode);

    esp_err_t updateData(const CanDriver::CanFrame& frame);
    // PID Data Getters
    float     getValue(uint16_t pid, uint32_t timeout_ms = 500);
    float     getValueUnsafe(uint16_t pid, uint32_t timeout_ms = 500);
    uint32_t  getLastUpdated(uint16_t pid) const;
    uint32_t  getId(uint16_t pid) const;
    esp_err_t getRawData(uint16_t pid, uint8_t* outData) const;
    uint8_t   getRawDataByte(uint16_t pid, uint8_t idx) const;
    uint8_t   getRawDataByteUnsafe(uint16_t pid, uint8_t idx) const;
    bool      isValid(uint16_t pid) const;
    bool      isSup(uint16_t pid) const;
    esp_err_t getData(uint16_t pid, PIDData_t*& pd) const;

    esp_err_t setValid(uint16_t pid, bool valid);

    esp_err_t             clearDTC(uint8_t mode);
    std::string           decodeDTC(uint16_t rawDTC);
    std::vector<uint16_t> getPIDs() const;

    using PidUpdateCallback = std::function<void(uint16_t pid)>;
    void                           subscribe(PidUpdateCallback cb);
    std::vector<PidUpdateCallback> subscribers_;

    void runPidUpdateCallbacks(uint16_t pid);

    const VINData_t& getVinData() const
    {
        return vinData;
    }

    const DTCData_t& getDtcData() const
    {
        return dtcData;
    }

    uint32_t getPIDDataSize() const
    {
        return pidData.size();
    }

    uint32_t getPIDDEFSize() const
    {
        return PID_DEF.size();
    }

    virtual esp_err_t addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                             std::string desc, std::string formula, float minV, float maxV, uint8_t priority,
                             UpdateRate interval, uint32_t color, std::string icon);

    esp_err_t removePID(uint16_t pid);

    // PID Definitions and Data Storage
    std::map<uint16_t, PIDDefinition> PID_DEF;
    std::map<uint16_t, PIDData_t>     pidData;
    VINData_t                         vinData;
    DTCData_t                         dtcData;

    void startPolling();

    PIDPriorityQueue pollQueue;

protected:
    mutable SemaphoreHandle_t pidMapMtx = nullptr;
    bool                      _isSup(uint16_t pid) const;
    bool                      _pidExists(uint16_t pid) const;
    // PID_DEF Getters without mutex
    uint32_t    _getId_Def(uint16_t pid) const;
    uint8_t     _getMode(uint16_t pid) const;
    uint8_t     _getLen(uint16_t pid) const;
    std::string _getName(uint16_t pid) const;
    std::string _getUnit(uint16_t pid) const;
    std::string _getDescription(uint16_t pid) const;
    float       _getMinValue(uint16_t pid) const;
    float       _getMaxValue(uint16_t pid) const;
    uint8_t     _getPriority(uint16_t pid) const;
    uint16_t    _getUpdateInterval(uint16_t pid) const;
    uint32_t    _getColor(uint16_t pid) const;
    std::string _getIcon(uint16_t pid) const;
    std::string _getFormula(uint16_t pid) const;
    esp_err_t   _getDef(uint16_t pid, const PIDDefinition*& outDef) const;
    // PID Data Getters without mutex
    float     _getValue(uint16_t pid, uint32_t timeout_ms = 500);
    uint32_t  _getLastUpdated(uint16_t pid) const;
    uint32_t  _getId(uint16_t pid) const;
    esp_err_t _getRawData(uint16_t pid, uint8_t* outData) const;
    uint8_t   _getRawDataByte(uint16_t pid, uint8_t idx) const;
    bool      _isValid(uint16_t pid) const;
    esp_err_t _getData(uint16_t pid, PIDData_t*& pd) const;

    esp_err_t _setDTC(uint16_t rawDTC, uint8_t mode);

    // PID Data Setters without mutex
    esp_err_t _setUpdateInterval(uint16_t pid, UpdateRate interval_ms);
    esp_err_t _setValid(uint16_t pid, bool valid);
    esp_err_t _setIsSupported(uint16_t pid, bool supported);
    esp_err_t _setLastUpdated(uint16_t pid, uint32_t lastUpdated);
    esp_err_t _setId(uint16_t pid, uint32_t id);
};