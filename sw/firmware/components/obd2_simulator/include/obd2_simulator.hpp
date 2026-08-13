#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// Forward Declaration: This tells the compiler that CanDriver exists,
// allowing us to use pointers/references without circular includes.
class CanDriver;

// Global Task Handle: Must be defined as 'extern' in the header.
// Initialize in the implementation file.
extern TaskHandle_t xDataSimTaskHandle;

// Function Declarations
void dataSimTask(CanDriver &canDriver);
void dataSimTaskWrapper(void *param);
void start_sim_task(CanDriver *driver);
void stop_sim_task();
void sendVIN(bool multiframe_in_progress);
uint32_t sinDataSim(uint32_t t_ms, float frequency, uint32_t size);

// ISR-safe notify hook, called from CanDriver's RX ISR in debug mode.
void IRAM_ATTR sim_notify(uint32_t data, BaseType_t* woken);