// battery.hpp
#pragma once
#include "esp_err.h"

// Idempotent. ESP_OK once the ADC unit is ready.
esp_err_t battery_init();
// Releases the ADC unit. Safe to call when not initialised.
void battery_deinit();
// Volts at the OBD port, or NaN when the ADC is not initialised or a conversion failed ("unknown").
float battery_read();
