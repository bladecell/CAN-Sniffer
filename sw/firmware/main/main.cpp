// main.cpp
#include "esp_log.h"
#include "led_status.hpp"
#include "obd2.hpp"
#include "obd2_utils.hpp"
#include "supervisor.hpp"
#include "utilities.h"

#define PID_DERIVED_TEST_1 0xD001
#define PID_DERIVED_TEST_2 0xD002
#define PID_DERIVED_TEST_3 0xD003
#define PID_DERIVED_TEST_4 0xD004

static const char* TAG = "APP_MAIN";

// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void)
{
    esp_log_level_set("CAN_DRIVER", ESP_LOG_DEBUG);
    LedStatus& led = LedStatus::getInstance();
    led.init(LED_GPIO);
    // esp_log_level_set("ASYNC_WEB_SERVER", ESP_LOG_DEBUG);

    SUPERVISOR::getInstance().start();

    // 1. Engine Load: A * 100 / 255
    OBD2::getInstance().addPID(OBD2_FUNCTIONAL_ID, MODE_CURRENT_DATA, PID_ENGINE_LOAD, 2, "Engine Load", PERCENTAGE,
                               "Calculated engine load", "A * 100 / 255", 0.0f, 100.0f, 2, UPDATE_FAST, 0xf59e0b,
                               "gauge");

    // 2. Coolant Temp: A - 40
    OBD2::getInstance().addPID(OBD2_FUNCTIONAL_ID, MODE_CURRENT_DATA, PID_COOLANT_TEMP, 2, "Coolant Temp",
                               DEGREES_CELCIUS, "Engine coolant temperature", "A - 40", -40.0f, 215.0f, 3, UPDATE_SLOW,
                               0xef4444, "thermometer");

    // 3. Engine RPM: ((A * 256) + B) / 4
    OBD2::getInstance().addPID(OBD2_FUNCTIONAL_ID, MODE_CURRENT_DATA, PID_ENGINE_RPM, 2, "Engine RPM", RPM,
                               "Engine speed", "((A * 256) + B) / 4", 0.0f, 16383.75f, 1, UPDATE_FAST, 0x3b82f6,
                               "droplet");

    // 4. Odometer: ((A * 256) + B) * 10
    OBD2::getInstance().addPID(0x714, MODE_READ_DATA_BY_IDENTIFIER, 0x2203, 3, "Odometer", KM, "Total distance",
                               "((A * 256) + B) * 10 ", 0.0f, 999999.0f, 1, UPDATE_SLOW, 0x3498db, "gauge");

    // 5. Fuel Level: A
    OBD2::getInstance().addPID(0x714, MODE_READ_DATA_BY_IDENTIFIER, 0x2206, 3, "Fuel Amount", LITER,
                               "Fuel Tank Level (Liters)", "A", 0.0f, 50.0f, 2, UPDATE_SLOW, 0x2ecc71, "droplet");

    // 6. Cabin Temperature: ((A * 256) + B) * 0.1
    OBD2::getInstance().addPID(0x746, 0x22, 0x2613, 3, "Interior Temp", DEGREES_CELCIUS, "Cabin Temperature",
                               "((A*256)+B)*0.1", -40.0f, 85.0f, 2, UPDATE_MEDIUM, 0x3498db, "thermometer");

    // Derived: Engine Load * RPM proxy (arbitrary test formula)
    OBD2::getInstance().addPID(OBD2_FUNCTIONAL_ID, MODE_DERIVED_DATA, PID_DERIVED_TEST_1, 0, "Load x RPM", RPM,
                               "Engine load scaled by RPM", "getPID(0x04) * getPID(0x0C) / 100", 0.0f, 16383.75f, 2,
                               UPDATE_FAST, 0xa855f7, "droplet");

    // Derived: RPM to approximate speed (test only, not real)
    OBD2::getInstance().addPID(OBD2_FUNCTIONAL_ID, MODE_DERIVED_DATA, PID_DERIVED_TEST_2, 0, "RPM / Temp", PERCENTAGE,
                               "RPM divided by coolant temp sanity check", "getPID(0x0C) / getPID(0x05)", 0.0f, 1000.0f,
                               2, UPDATE_FAST, 0x22d3ee, "gauge");

    // Derived: 3rd bit of 2nd byte of engine load raw data
    OBD2::getInstance().addPID(OBD2_FUNCTIONAL_ID, MODE_DERIVED_DATA, PID_DERIVED_TEST_3, 0, "Load Bit Test",
                               PERCENTAGE, "3rd bit of 4th byte of engine load", "getBit(getPIDRaw(0x04, 3), 2)", 0.0f,
                               1.0f, 2, UPDATE_SLOW, 0xec4899, "droplet");

    // Derived: bits 2-5 of 1st byte of RPM raw data
    OBD2::getInstance().addPID(OBD2_FUNCTIONAL_ID, MODE_DERIVED_DATA, PID_DERIVED_TEST_4, 0, "RPM BitMask Test", RPM,
                               "bits 2 to 5 of 4th byte of RPM", "bitMask(getPIDRaw(0x0C, 3), 2, 4)", 0.0f, 15.0f, 2,
                               UPDATE_MEDIUM, 0xf97316, "gauge");

    // xTaskCreate(t_request_sample, "request_sample", 4096, NULL, 5, NULL);
}