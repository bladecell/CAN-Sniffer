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

// Frames the ISR hook dropped because the simulator queue was full.
extern volatile uint32_t simDroppedFrames;

// Function Declarations
void dataSimTask(CanDriver &canDriver);
void dataSimTaskWrapper(void *param);
bool start_sim_task(CanDriver *driver);
// true when the task is gone (or never ran); false if it did not leave in time.
bool stop_sim_task();
void sendVIN(bool multiframe_in_progress);
uint32_t sinDataSim(uint32_t t_ms, float frequency, uint32_t size);

// ISR-safe notify hook, called from CanDriver's RX ISR in debug mode.
void sim_notify(uint32_t id, const uint8_t* data, uint8_t length, BaseType_t* woken);