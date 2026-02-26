// pid_def.cpp
#include "pid_def.hpp"

#include "tinyexpr_helpers.h"  // safe here - no cycle, obd2.hpp fully visible

PIDDefinition::PIDDefinition(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                             std::string desc, std::string formula, float minV, float maxV, uint8_t priority,
                             UpdateRate interval, uint32_t color, std::string icon)
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
      icon_(std::move(icon))
{
    te_variable te_vars[] = {
        {"A", &vars_storage_[0], TE_VARIABLE, 0},          {"B", &vars_storage_[1], TE_VARIABLE, 0},
        {"C", &vars_storage_[2], TE_VARIABLE, 0},          {"D", &vars_storage_[3], TE_VARIABLE, 0},
        {"getBit", (void*)get_bit, TE_FUNCTION2, 0},       {"bitMask", (void*)bit_mask, TE_FUNCTION3, 0},
        {"getPID", (void*)get_pid_value, TE_FUNCTION1, 0}, {"getPIDRaw", (void*)get_pid_raw, TE_FUNCTION2, 0},
    };

    int err;
    compiledFormula_ = te_compile(formula_.c_str(), te_vars, 8, &err);
}

PIDDefinition::~PIDDefinition()
{
    if (compiledFormula_)
        te_free(compiledFormula_);
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
      compiledFormula_(other.compiledFormula_)
{
    other.compiledFormula_ = nullptr;
}

esp_err_t PIDDefinition::evaluate(const uint8_t* frameData, uint8_t len, float& result) const
{
    if (!compiledFormula_)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(eval_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (int i = 0; i < 4; i++)
        {
            vars_storage_[i] = (i < len) ? (double)frameData[i] : 0.0;
        }

        result = (float)te_eval(compiledFormula_);

        xSemaphoreGive(eval_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}