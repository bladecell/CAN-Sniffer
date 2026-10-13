/**
 * @file obd2_internal.hpp
 * @brief Private to the obd2 component: shared includes, macros and helpers for the obd2*.cpp translation units.
 * Not a public API header; do not include from outside components/obd2.
 */
#pragma once

#include "obd2.hpp"
#include "obd2_frame.hpp"

#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "can_driver.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "obd2_common.hpp"
#include "obd2_data_model.hpp"

#define HEALTHCHECK_RETRIES 3
#define OBD2_HEALTHCHECK_PING_PERIOD_MS                                                                              \
    (((NO_MESSAGE_SENT_HEALTHCHECK_PING_PERIOD_MS - 1000) > 1000) ? (NO_MESSAGE_SENT_HEALTHCHECK_PING_PERIOD_MS - 1000) \
                                                                    : 1000)
#define HEALTHCHECK_RESPONSE_TIMEOUT_MS 3000
#define SUPPORTED_PID_RESPONSE_TIMEOUT_MS 500
#define SUPPORTED_PID_RESPONSE_WINDOW_MS 50

inline constexpr const char* TAG = "OBD2";
