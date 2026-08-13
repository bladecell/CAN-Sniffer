// obd2_dtb.hpp
#pragma once

#include <math.h>

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "can_driver.hpp"
#include "esp_err.h"
#include "obd2_common.hpp"
#include "pid_def.hpp"
#include "pid_priority_queue.hpp"

struct NRCRecord
{
    uint32_t can_id;
    uint8_t  mode;
    uint8_t  nrc;
};

class OBD2DataModel
{
public:
    void initDef();
    bool pidExists(uint16_t pid) const;
    // PID_DEF Getters
    inline uint32_t getId_Def(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::id, (uint32_t)0);
    }

    inline uint8_t getMode(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::mode, (uint8_t)0);
    }

    inline uint8_t getLen(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::len, (uint8_t)0);
    }

    inline std::string getName(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::name, "Unknown");
    }

    inline std::string getUnit(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::unit, "Unknown");
    }

    inline std::string getDescription(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::description, "Unknown");
    }

    inline float getMinValue(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::minValue, NAN);
    }

    inline float getMaxValue(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::maxValue, NAN);
    }

    inline uint8_t getPriority(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::priority, (uint8_t)0);
    }

    inline uint16_t getUpdateInterval(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::updateInterval, (uint16_t)0);
    }

    inline uint32_t getColor(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::color, (uint32_t)0xFFFFFF);
    }

    inline std::string getIcon(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::icon, "Unknown");
    }

    inline std::string getFormula(uint16_t pid) const
    {
        return _getDefFieldWithLock(pid, &PIDDefinition::formula, "Unknown");
    }
    // PIDDefinition *getDef(uint16_t pid) const;
    esp_err_t getDef(uint16_t pid, PIDDefinitionData& outDef) const;

    // Special Getters
    std::string              getVIN() const;
    std::vector<std::string> getDTC(uint8_t mode) const;

    esp_err_t updateData(const CanDriver::CanFrame& frame);
    // PID Data Getters
    inline float getValue(uint16_t pid) const
    {
        return _getDataFieldWithLock(pid, &PIDData_t::value, NAN);
    }

    inline uint32_t getLastUpdated(uint16_t pid) const
    {
        return _getDataFieldWithLock(pid, &PIDData_t::lastUpdated, (uint32_t)0);
    }

    inline uint32_t getId(uint16_t pid) const
    {
        return _getDataFieldWithLock(pid, &PIDData_t::id, (uint32_t)0);
    }

    inline bool isValid(uint16_t pid) const
    {
        return _getDataFieldWithLock(pid, &PIDData_t::isValid, false);
    }

    inline bool isSup(uint16_t pid) const
    {
        return _getDataFieldWithLock(pid, &PIDData_t::isSupported, false);
    }
    uint8_t   getRawDataByte(uint16_t pid, uint8_t idx) const;
    esp_err_t getRawData(uint16_t pid, uint8_t* outData) const;
    float     getValueUnsafe(uint16_t pid) const;
    uint8_t   getRawDataByteUnsafe(uint16_t pid, uint8_t idx) const;
    esp_err_t getData(uint16_t pid, PIDData_t& pd) const;

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
                             uint16_t interval, uint32_t color, std::string icon);

    esp_err_t removePID(uint16_t pid);

    // PID Definitions and Data Storage
    std::map<uint16_t, PIDDefinition> PID_DEF;
    std::map<uint16_t, PIDData_t>     pidData;
    VINData_t                         vinData;
    DTCData_t                         dtcData;

    NRCRecord nrc_list[MAX_NRC_LIST_SIZE];
    uint8_t   nrc_list_size = 0;

    PIDPriorityQueue pollQueue;

protected:
    mutable SemaphoreHandle_t pidMapMtx = nullptr;
    mutable SemaphoreHandle_t subscribers_mtx_ = nullptr;
    bool                      _isSup(uint16_t pid) const;
    bool                      _pidExists(uint16_t pid) const;

    // PID Data Getters without mutex
    esp_err_t _getDef(uint16_t pid, const PIDDefinition*& outDef) const;
    esp_err_t _getData(uint16_t pid, PIDData_t*& pd) const;
    esp_err_t _getRawData(uint16_t pid, uint8_t* outData) const;
    uint8_t   _getRawDataByte(uint16_t pid, uint8_t idx) const;

    esp_err_t _setDTC(uint16_t rawDTC, uint8_t mode);

    template <typename F>
    esp_err_t withPidMapLock(F&& func) const
    {
        MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));
        if (!guard.isLocked())
        {
            return ESP_ERR_TIMEOUT;
        }
        return func();
    }

    template <typename T, typename D>
    auto _getDefField(uint16_t pid, T (PIDDefinition::*getter)() const, D defaultValue) const ->
        typename std::decay<T>::type
    {
        using ReturnType         = typename std::decay<T>::type;
        const PIDDefinition* def = nullptr;
        if (_getDef(pid, def) == ESP_OK && def != nullptr)
        {
            return (def->*getter)();
        }
        return static_cast<ReturnType>(defaultValue);
    }

    template <typename T>
    T _getDataField(uint16_t pid, T PIDData_t::* field, T defaultValue) const
    {
        PIDData_t* pd = nullptr;
        if (_getData(pid, pd) == ESP_OK && pd != nullptr)
            return pd->*field;
        return defaultValue;
    }

    // Locked Helpers (Wrappers)

    template <typename T, typename D>
    auto _getDefFieldWithLock(uint16_t pid, T (PIDDefinition::*getter)() const, D defaultValue) const ->
        typename std::decay<T>::type
    {
        using ReturnType = typename std::decay<T>::type;
        ReturnType value = static_cast<ReturnType>(defaultValue);

        withPidMapLock(
            [&]()
            {
                value = _getDefField(pid, getter, defaultValue);
                return ESP_OK;
            });

        return value;
    }

    template <typename T>
    T _getDataFieldWithLock(uint16_t pid, T PIDData_t::* field, T defaultValue) const
    {
        T value = defaultValue;
        withPidMapLock(
            [&]()
            {
                value = _getDataField(pid, field, defaultValue);
                return ESP_OK;
            });
        return value;
    }

    // Internal Setter (No Lock)
    template <typename T, typename V>
    esp_err_t _setDataField(uint16_t pid, T PIDData_t::* field, V value)
    {
        PIDData_t* pd  = nullptr;
        esp_err_t  ret = _getData(pid, pd);
        if (ret != ESP_OK || pd == nullptr)
            return ret;

        pd->*field = static_cast<T>(value);
        return ESP_OK;
    }

    // Thread-Safe Setter (With Lock)
    template <typename T, typename V>
    esp_err_t _setDataFieldWithLock(uint16_t pid, T PIDData_t::* field, V value)
    {
        return withPidMapLock([&]() { return _setDataField(pid, field, value); });
    }
};