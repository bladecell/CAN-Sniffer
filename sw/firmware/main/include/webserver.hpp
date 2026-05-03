// webserver.hpp
#pragma once

#include <stddef.h>
#include "esp_err.h"

// Forward declarations
struct cJSON;
struct httpd_req;
typedef struct httpd_req httpd_req_t;

#define WS_START_PID_STREAM 0xA0
#define WS_STOP_PID_STREAM 0xA1

esp_err_t setup_web_server();

void ws_send_can_status();

esp_err_t send_json_response(httpd_req_t* req, cJSON* root);
bool      get_query_str(httpd_req_t* req, const char* key, char* out_val, size_t val_len);
esp_err_t get_query_int(httpd_req_t* req, const char* key, int* value);
esp_err_t get_query_bool(httpd_req_t* req, const char* key, bool* value);

// Web server request handlers
esp_err_t index_handler(httpd_req_t* req, void* arg);
esp_err_t g_pid_def_index_handler(httpd_req_t* req, void* arg);
esp_err_t g_pid_data_index_handler(httpd_req_t* req, void* arg);
esp_err_t g_can_bus_index_handler(httpd_req_t* req, void* arg);
esp_err_t g_obdii_index_handler(httpd_req_t* req, void* arg);
esp_err_t g_request_data_index_handler(httpd_req_t* req, void* arg);
esp_err_t g_dtc_index_handler(httpd_req_t* req, void* arg);
esp_err_t g_vin_index_handler(httpd_req_t* req, void* arg);
esp_err_t p_pid_poll_data_index_handler(httpd_req_t* req, void* arg);
esp_err_t p_clear_dtc_data_index_handler(httpd_req_t* req, void* arg);
esp_err_t p_dtc_index_handler(httpd_req_t* req, void* arg);
esp_err_t p_vin_index_handler(httpd_req_t* req, void* arg);
esp_err_t p_clear_dtc_index_handler(httpd_req_t* req, void* arg);
esp_err_t ws_socket_handler(httpd_req_t* req);
esp_err_t p_system_index_handler(httpd_req_t* req, void* arg);

void pid_stream_callback(uint16_t pid);
void enable_pid_stream(bool enable);