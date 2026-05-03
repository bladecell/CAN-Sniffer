// battery.hpp
#pragma once
#include "esp_err.h"

esp_err_t battery_init();
float     battery_read();