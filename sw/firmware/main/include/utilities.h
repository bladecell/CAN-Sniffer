// utilities.h
#pragma once

#define DEBUG_MODE 1

#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 2
#define APP_VERSION_PATCH 4

// REST/WebSocket contract version (docs/api.md, Versioning). Bump on any breaking change, never for additive ones.
#define API_VERSION 1

#define STR_(x) #x
#define STR(x) STR_(x)

#define APP_VERSION_STRING STR(APP_VERSION_MAJOR) "." STR(APP_VERSION_MINOR) "." STR(APP_VERSION_PATCH)

#include "sdkconfig.h"

// Pin configuration
#if defined(CONFIG_SNIFFER_HW_REV_V1_1)
#define CAN_RX_GPIO GPIO_NUM_4
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_LBK_GPIO GPIO_NUM_6
#define CAN_RS_GPIO GPIO_NUM_7
#define LED_GPIO GPIO_NUM_47
#define MISO GPIO_NUM_10
#define MOSI GPIO_NUM_12
#define SCLK GPIO_NUM_11
#define CS GPIO_NUM_13
#define SD_DETECT GPIO_NUM_9
#elif defined(CONFIG_SNIFFER_HW_REV_V1_2)
#define SD_DETECT GPIO_NUM_13
#define CAN_RX_GPIO GPIO_NUM_4
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RS_GPIO GPIO_NUM_6
#define CAN_LBK_GPIO GPIO_NUM_7
#define LED_GPIO GPIO_NUM_47
#define MISO GPIO_NUM_12
#define MOSI GPIO_NUM_10
#define SCLK GPIO_NUM_11
#define CS GPIO_NUM_9
#elif defined(CONFIG_SNIFFER_HW_REV_V2)
#error "HW v2 pin map is not defined yet (see docs/hardware.md)"
#else
#error "No hardware revision selected (CONFIG_SNIFFER_HW_REV_*)"
#endif
