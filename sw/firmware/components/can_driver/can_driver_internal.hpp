#pragma once

// Private to the can_driver component: shared by the can_driver*.cpp files.
#include "can_driver.hpp"

#include <cstring>
#include <new>

#include "esp_log.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "hal/twai_types.h"

static const char* const TAG = "CAN_DRIVER";
extern SemaphoreHandle_t canConnectedSemaphore;

#define LOG_CAN_FRAME(LOG_TAG, DIR, CAN_ID, DATA_PTR, LEN)                                                         \
    ESP_LOGD(LOG_TAG, "%s %X %02X %02X %02X %02X %02X %02X %02X %02X", DIR, CAN_ID, (LEN) > 0 ? (DATA_PTR)[0] : 0, \
             (LEN) > 1 ? (DATA_PTR)[1] : 0, (LEN) > 2 ? (DATA_PTR)[2] : 0, (LEN) > 3 ? (DATA_PTR)[3] : 0,          \
             (LEN) > 4 ? (DATA_PTR)[4] : 0, (LEN) > 5 ? (DATA_PTR)[5] : 0, (LEN) > 6 ? (DATA_PTR)[6] : 0,          \
             (LEN) > 7 ? (DATA_PTR)[7] : 0)
