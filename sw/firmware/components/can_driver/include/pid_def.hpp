#pragma once

#include <string>
#include <utility>
#include "tinyexpr.h"
#include "obd2_utils.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class PIDDefinition
{
private:
    uint8_t mode_;
    uint8_t pid_;
    std::string name_;
    std::string unit_;
    std::string description_;
    std::string formula_;
    float minValue_;
    float maxValue_;
    uint8_t priority_;
    uint16_t updateInterval_ms_;
    uint32_t color_;
    std::string icon_;

    te_expr *compiledFormula_ = nullptr;
    static inline double raw_vars[4] = {0, 0, 0, 0};
    static inline SemaphoreHandle_t eval_mutex = xSemaphoreCreateMutex();

public:
    PIDDefinition(uint8_t mode, uint8_t pid, std::string name, std::string unit,
                  std::string desc, std::string formula, float minV, float maxV,
                  uint8_t priority, UpdateRate interval, uint32_t color, std::string icon)
        : mode_(mode), pid_(pid), name_(std::move(name)), unit_(std::move(unit)),
          description_(std::move(desc)), formula_(std::move(formula)),
          minValue_(minV), maxValue_(maxV), priority_(priority),
          updateInterval_ms_((uint16_t)interval), color_(color), icon_(std::move(icon))
    {
        te_variable te_vars[] = {
            {"A", &PIDDefinition::raw_vars[0], 0, 0},
            {"B", &PIDDefinition::raw_vars[1], 0, 0},
            {"C", &PIDDefinition::raw_vars[2], 0, 0},
            {"D", &PIDDefinition::raw_vars[3], 0, 0}};

        int err;
        compiledFormula_ = te_compile(formula_.c_str(), te_vars, 4, &err);
    }

    // Getters
    uint8_t mode() const { return mode_; }
    uint8_t pid() const { return pid_; }
    const std::string &name() const { return name_; }
    const std::string &unit() const { return unit_; }
    const std::string &description() const { return description_; }
    const std::string &formula() const { return formula_; }
    float minValue() const { return minValue_; }
    float maxValue() const { return maxValue_; }
    uint8_t priority() const { return priority_; }
    uint16_t updateInterval() const { return updateInterval_ms_; }
    uint32_t color() const { return color_; }
    const std::string &icon() const { return icon_; }

    ~PIDDefinition()
    {
        if (compiledFormula_)
            te_free(compiledFormula_);
    }

    esp_err_t evaluate(const uint8_t *frameData, uint8_t len, float &result) const
    {
        if (!compiledFormula_)
            return ESP_ERR_INVALID_STATE;

        if (xSemaphoreTake(eval_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {

            for (int i = 0; i < 4; i++)
            {
                raw_vars[i] = (i < len) ? (double)frameData[i] : 0.0;
            }

            result = (float)te_eval(compiledFormula_);

            xSemaphoreGive(eval_mutex);
            return ESP_OK;
        }
        return ESP_ERR_TIMEOUT;
    }

    PIDDefinition(const PIDDefinition &) = delete;
    PIDDefinition &operator=(const PIDDefinition &) = delete;

    PIDDefinition(PIDDefinition &&other) noexcept
        : mode_(other.mode_), pid_(other.pid_), name_(std::move(other.name_)),
          unit_(std::move(other.unit_)), description_(std::move(other.description_)),
          formula_(std::move(other.formula_)), minValue_(other.minValue_),
          maxValue_(other.maxValue_), priority_(other.priority_),
          updateInterval_ms_(other.updateInterval_ms_), color_(other.color_),
          icon_(std::move(other.icon_)),
          compiledFormula_(other.compiledFormula_)
    {
        other.compiledFormula_ = nullptr; // Clear the old one
    }
};