// obd2.hpp
#pragma once

#include <math.h>
#include "obd2_utils.hpp"
#include "can_driver.hpp"
#include "esp_log.h"

static const char *TAG = "OBD2";

class OBD2 {
    public:
        explicit OBD2(CanDriver& can_driver) : can_driver(can_driver), connected(false), continuousRunning(false) {}
        esp_err_t begin();
        esp_err_t query_supported_pids(uint8_t start_pid);
    private:
        CanDriver& can_driver;
        bool connected;
        bool continuousRunning;
}


