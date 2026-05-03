// utilities.h
#pragma once

#define DEBUG_MODE 1
#define APP_VERSION_MAJOR 0
#define APP_VERION_MINOR 3

#define VERSION_1_1
// #define VERSION_1_2

// Pin configuration
#ifdef VERSION_1_1
#define CAN_RX_GPIO GPIO_NUM_4
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_LBK_GPIO GPIO_NUM_6
#define CAN_RS_GPIO GPIO_NUM_7
#define LED_GPIO GPIO_NUM_47
#elif defined(VERSION_1_2)
#define CAN_RX_GPIO GPIO_NUM_4
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RS_GPIO GPIO_NUM_6
#define CAN_LBK_GPIO GPIO_NUM_7
#define LED_GPIO GPIO_NUM_47
#endif