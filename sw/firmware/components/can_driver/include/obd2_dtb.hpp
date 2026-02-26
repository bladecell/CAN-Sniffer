// obd2_dtb.hpp
#pragma once

#include <math.h>

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "esp_err.h"
#include "obd2_utils.hpp"
#include "pid_def.hpp"

class OBD2DTB
{
public:
    void initDef();
    bool isSup(uint16_t pid) const;
    bool pidExists(uint16_t pid) const;
    // PID_DEF Getters
    uint32_t    getId_Def(uint16_t pid) const;
    uint8_t     getMode(uint16_t pid) const;
    uint8_t     getLen(uint16_t pid) const;
    const char* getName(uint16_t pid) const;
    const char* getUnit(uint16_t pid) const;
    const char* getDescription(uint16_t pid) const;
    float       getMinValue(uint16_t pid) const;
    float       getMaxValue(uint16_t pid) const;
    uint8_t     getPriority(uint16_t pid) const;
    uint16_t    getUpdateInterval(uint16_t pid) const;
    uint32_t    getColor(uint16_t pid) const;
    const char* getIcon(uint16_t pid) const;
    const char* getFormula(uint16_t pid) const;
    // PIDDefinition *getDef(uint16_t pid) const;
    esp_err_t getDef(uint16_t pid, const PIDDefinition*& outDef) const;

    // Special Getters
    std::string              getVIN() const;
    std::vector<std::string> getDTC(uint8_t mode);

    // PID Data Getters
    float     getValue(uint16_t pid, uint32_t timeout_ms = 500);
    uint32_t  getLastUpdated(uint16_t pid) const;
    uint32_t  getId(uint16_t pid) const;
    esp_err_t getRawData(uint16_t pid, uint8_t* outData) const;
    uint8_t   getRawDataByte(uint16_t pid, uint8_t idx) const;
    bool      isValid(uint16_t pid) const;
    esp_err_t getData(uint16_t pid, PIDData_t& pd) const;

    // PID Data Setters
    esp_err_t             setUpdateInterval(uint16_t pid, UpdateRate interval_ms);
    esp_err_t             setValid(uint16_t pid, bool valid);
    esp_err_t             setIsSupported(uint16_t pid, bool supported);
    esp_err_t             setLastUpdated(uint16_t pid, uint32_t lastUpdated);
    esp_err_t             setId(uint16_t pid, uint32_t id);
    esp_err_t             setDTC(uint16_t rawDTC, uint8_t mode);
    esp_err_t             clearDTC(uint8_t mode);
    std::string           decodeDTC(uint16_t rawDTC);
    std::vector<uint16_t> getPIDs() const;
    esp_err_t             updateData(const CanDriver::CanFrame& frame);

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

    void addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                std::string desc, std::string formula, float minV, float maxV, uint8_t priority, UpdateRate interval,
                uint32_t color, std::string icon);

    // PID Definitions and Data Storage
    std::map<uint16_t, std::unique_ptr<PIDDefinition>> PID_DEF;
    std::map<uint16_t, PIDData_t>                      pidData;
    VINData_t                                          vinData;
    DTCData_t                                          dtcData;

    void                  generatePollingGroups();
    std::vector<uint16_t> vGroupFast;
    std::vector<uint16_t> vGroupMedium;
    std::vector<uint16_t> vGroupSlow;
    std::vector<uint16_t> vGroupStatic;
    std::set<uint32_t>    diagnosticSessionIds;
};