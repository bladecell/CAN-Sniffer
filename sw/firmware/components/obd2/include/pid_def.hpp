#pragma once

#include <cstdint>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "obd2_common.hpp"

// Forward declaration for tinyexpr
struct te_expr;

struct PIDDefinitionData
{
    uint32_t    id;
    uint8_t     mode;
    uint16_t    pid;
    uint8_t     len;
    std::string name;
    std::string unit;
    std::string description;
    std::string formula;
    float       minValue;
    float       maxValue;
    uint16_t    updateInterval_ms;
    uint32_t    color;
    uint8_t     priority;
    std::string icon;
};

class PIDDefinition
{
public:
    uint32_t    id_;
    uint8_t     mode_;
    uint16_t    pid_;
    uint8_t     len_;
    std::string name_;
    std::string unit_;
    std::string description_;
    std::string formula_;
    float       minValue_;
    float       maxValue_;
    uint8_t     priority_;
    uint16_t    updateInterval_ms_;
    uint32_t    color_;
    std::string icon_;

    te_expr*                  compiledFormula_ = nullptr;
    mutable double            vars_storage_[4] = {0, 0, 0, 0};
    mutable SemaphoreHandle_t instance_mutex_  = nullptr;

public:
    PIDDefinition(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                  std::string desc, std::string formula, float minV, float maxV, uint8_t priority, uint16_t interval,
                  uint32_t color, std::string icon);

    ~PIDDefinition();

    // Non-copyable
    PIDDefinition(const PIDDefinition&)            = delete;
    PIDDefinition& operator=(const PIDDefinition&) = delete;

    // Movable
    PIDDefinition(PIDDefinition&& other) noexcept;

    esp_err_t evaluate(const uint8_t* frameData, uint8_t len, float& result) const;

    // Getters
    uint32_t id() const
    {
        return id_;
    }
    uint8_t mode() const
    {
        return mode_;
    }
    uint16_t pid() const
    {
        return pid_;
    }
    uint8_t len() const
    {
        return len_;
    }
    const std::string& name() const
    {
        return name_;
    }
    const std::string& unit() const
    {
        return unit_;
    }
    const std::string& description() const
    {
        return description_;
    }
    const std::string& formula() const
    {
        return formula_;
    }
    float minValue() const
    {
        return minValue_;
    }
    float maxValue() const
    {
        return maxValue_;
    }
    uint8_t priority() const
    {
        return priority_;
    }
    uint16_t updateInterval() const
    {
        return updateInterval_ms_;
    }
    uint32_t color() const
    {
        return color_;
    }
    const std::string& icon() const
    {
        return icon_;
    }
};