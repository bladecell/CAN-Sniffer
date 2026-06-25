// battery.cpp
#include "battery.hpp"

#include <cmath>

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"

#define TAG "BATT"

static constexpr float R_TOP         = 1000000.0f;  // change to 100k in next hw rev.
static constexpr float R_BOT         = 270000.0f;   // change to 27k in next hw rev.
static constexpr float DIVIDER_RATIO = (R_TOP + R_BOT) / R_BOT;
static constexpr int   NUM_SAMPLES   = 16;

static adc_oneshot_unit_handle_t adc_handle;

esp_err_t battery_init()
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &adc_handle), TAG, "Failed to init ADC unit");

    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &chan_cfg), TAG,
                        "Failed to config ADC channel");

    return ESP_OK;
}

float battery_read()
{
    int sum = 0, raw;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &raw);
        sum += raw;
    }
    float mv = (static_cast<float>(sum) / NUM_SAMPLES / 4095.0f) * 3300.0f;
    float v  = (mv / 1000.0f) * DIVIDER_RATIO;
    return roundf(v * 10.0f) / 10.0f;
}