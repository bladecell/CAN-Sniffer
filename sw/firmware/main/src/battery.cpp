// battery.cpp
#include "battery.hpp"

#include <cmath>

#include "battery_calc.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"

#define TAG "BATT"

static constexpr float R_TOP         = 1000000.0f;  // change to 100k in next hw rev.
static constexpr float R_BOT         = 270000.0f;   // change to 27k in next hw rev.
static constexpr float DIVIDER_RATIO = (R_TOP + R_BOT) / R_BOT;
static constexpr int   NUM_SAMPLES   = 16;

static adc_oneshot_unit_handle_t adc_handle = nullptr;

esp_err_t battery_init()
{
    if (adc_handle != nullptr)
    {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_unit_handle_t handle = nullptr;
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &handle), TAG, "Failed to init ADC unit");

    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    esp_err_t              err      = adc_oneshot_config_channel(handle, ADC_CHANNEL_1, &chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(handle);  // a second battery_init() must not fail with "unit already in use"
        return err;
    }

    adc_handle = handle;
    return ESP_OK;
}

void battery_deinit()
{
    if (adc_handle != nullptr)
    {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = nullptr;
    }
}

float battery_read()
{
    if (adc_handle == nullptr)
    {
        return NAN;
    }
    return battery_calc::averageVolts(
        [](int& raw) { return adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &raw) == ESP_OK; }, NUM_SAMPLES, DIVIDER_RATIO);
}
