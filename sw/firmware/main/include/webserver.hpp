// webserver.hpp
#pragma once

#include <stddef.h>

#include "esp_err.h"

// ESP_ERR_TIMEOUT: a request handler did not finish in time; the server is still running, call again later.
esp_err_t stop_web_server();
#include "esp_err.h"

// Forward declarations
struct cJSON;
struct httpd_req;
typedef struct httpd_req httpd_req_t;

#define WS_START_PID_STREAM 0xA0
#define WS_STOP_PID_STREAM 0xA1
#define WS_PING_REQUEST_STREAM 0xA2
#define WS_PING_RESPONSE_STREAM 0xA3

esp_err_t setup_web_server();
