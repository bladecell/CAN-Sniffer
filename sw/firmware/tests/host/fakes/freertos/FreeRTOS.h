#pragma once
#include <cstdint>
using TickType_t = uint32_t;
using TaskHandle_t = void*;
using BaseType_t = int;
using SemaphoreHandle_t = void*;
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
TickType_t xTaskGetTickCount();
void xTaskNotifyGive(TaskHandle_t task);
