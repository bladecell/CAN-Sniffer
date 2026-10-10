// pid_def.cpp
#include "pid_def.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "tinyexpr.h"
#include "tinyexpr_helpers.h"  // safe here - no cycle, obd2.hpp fully visible

PIDDefinition::PIDDefinition(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                             std::string desc, std::string formula, float minV, float maxV, uint8_t priority,
                             uint16_t interval, uint32_t color, std::string icon)
    : id_(id),
      mode_(mode),
      pid_(pid),
      len_(len),
      name_(std::move(name)),
      unit_(std::move(unit)),
      description_(std::move(desc)),
      formula_(std::move(formula)),
      minValue_(minV),
      maxValue_(maxV),
      priority_(priority),
      updateInterval_ms_((uint16_t)interval),
      color_(color),
      icon_(std::move(icon)),
      instance_mutex_(xSemaphoreCreateMutex())
{
    te_variable te_vars[] = {
        {"A", &vars_storage_[0], TE_VARIABLE, 0},          {"B", &vars_storage_[1], TE_VARIABLE, 0},
        {"C", &vars_storage_[2], TE_VARIABLE, 0},          {"D", &vars_storage_[3], TE_VARIABLE, 0},
        {"getBit", (void*)get_bit, TE_FUNCTION2, 0},       {"bitMask", (void*)bit_mask, TE_FUNCTION3, 0},
        {"getPID", (void*)get_pid_value, TE_FUNCTION1, 0}, {"getPIDRaw", (void*)get_pid_raw, TE_FUNCTION2, 0},
    };

    // Scan for the single-letter variables A-D (whole identifiers only, so getBit's "B" does not count).
    for (size_t i = 0; i < formula_.size();)
    {
        if (std::isalpha((unsigned char)formula_[i]) || formula_[i] == '_')
        {
            size_t j = i + 1;
            while (j < formula_.size() && (std::isalnum((unsigned char)formula_[j]) || formula_[j] == '_'))
            {
                j++;
            }
            if (j - i == 1 && formula_[i] >= 'A' && formula_[i] <= 'D')
            {
                requiredBytes_ = std::max<uint8_t>(requiredBytes_, (uint8_t)(formula_[i] - 'A' + 1));
            }
            i = j;
        }
        else
        {
            i++;
        }
    }

    int err;
    compiledFormula_ = te_compile(formula_.c_str(), te_vars, 8, &err);
}

PIDDefinition::~PIDDefinition()
{
    if (compiledFormula_)
    {
        te_free(compiledFormula_);
    }
    if (instance_mutex_)
    {
        vSemaphoreDelete(instance_mutex_);
    }
}

PIDDefinition::PIDDefinition(PIDDefinition&& other) noexcept
    : id_(other.id_),
      mode_(other.mode_),
      pid_(other.pid_),
      len_(other.len_),
      name_(std::move(other.name_)),
      unit_(std::move(other.unit_)),
      description_(std::move(other.description_)),
      formula_(std::move(other.formula_)),
      minValue_(other.minValue_),
      maxValue_(other.maxValue_),
      priority_(other.priority_),
      updateInterval_ms_(other.updateInterval_ms_),
      color_(other.color_),
      icon_(std::move(other.icon_)),
      compiledFormula_(other.compiledFormula_),
      instance_mutex_(other.instance_mutex_)
{
    requiredBytes_         = other.requiredBytes_;
    other.compiledFormula_ = nullptr;
    other.instance_mutex_  = nullptr;
    for (int i = 0; i < 4; i++)
    {
        vars_storage_[i] = other.vars_storage_[i];
    }
}

esp_err_t PIDDefinition::evaluate(const uint8_t* frameData, uint8_t len, float& result) const
{
    if (!compiledFormula_ || !instance_mutex_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(instance_mutex_, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        // Internal state modification is safe here as it's a mutable-logical operation protected by mutex
        PIDDefinition* mutable_this = const_cast<PIDDefinition*>(this);
        for (int i = 0; i < 4; i++)
        {
            mutable_this->vars_storage_[i] = (frameData != nullptr && i < len) ? (double)frameData[i] : 0.0;
        }

        const double raw = te_eval(compiledFormula_);

        xSemaphoreGive(instance_mutex_);

        // NaN, inf, division by zero and values that overflow float mark the PID invalid; result stays untouched.
        const float value = (float)raw;
        if (!std::isfinite(raw) || !std::isfinite(value))
        {
            return ESP_ERR_INVALID_RESPONSE;
        }
        result = value;
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}