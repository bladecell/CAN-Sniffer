// middleware.hpp
#pragma once
#include <cstdint>

#include "cJSON.h"
#include "esp_err.h"

// Forward declarations to minimize includes
class PIDDefinition;
struct PIDData_t;

#define MSG_TYPE_LOG 0x01
#define MSG_TYPE_PID 0x02
#define MSG_TYPE_CAN_STATUS 0x03

#define PID_STREAM_PACKET_SIZE 20
#define CAN_STATUS_PACKET_SIZE 12

// The HTTP outcome is explicit and is never inferred from the JSON body.
struct MiddlewareJsonResult
{
    cJSON* body;
    int    http_status;
};

// Middlewares
MiddlewareJsonResult m_pid_def_get(int filter_id);
MiddlewareJsonResult m_pid_data_get(int filter_id);
MiddlewareJsonResult m_pid_def_delete(int filter_id);
MiddlewareJsonResult m_pid_def_post(cJSON* data);
// Returns an error JSON tree on failure and nullptr after a committed PUT.
// The caller owns and must delete a returned tree. A successful call reports
// HTTP 204 through http_status and transfers no JSON ownership.
cJSON* m_pid_def_put(cJSON* data, int* http_status);
void   m_pid_poll_set_running(bool running);
MiddlewareJsonResult m_can_bus_get();
MiddlewareJsonResult m_obdii_get();
MiddlewareJsonResult m_obdii_set(cJSON* payload);
MiddlewareJsonResult m_system_get();
MiddlewareJsonResult m_system_reboot();
MiddlewareJsonResult m_system_copy_file(cJSON* payload);
MiddlewareJsonResult m_sdcard_info_get();
MiddlewareJsonResult m_sdcard_format_post();
MiddlewareJsonResult m_sdcard_file_tree_get(const char* path);
MiddlewareJsonResult m_sdcard_file_delete_delete(const char* path);
MiddlewareJsonResult m_vin_get();
MiddlewareJsonResult m_dtc_get(int mode);
MiddlewareJsonResult m_dtc_description_get(const char* target_codes[], size_t count);
MiddlewareJsonResult m_vin_request();
MiddlewareJsonResult m_dtc_request(int mode);
MiddlewareJsonResult m_clear_dtc_request();
cJSON* m_static_pid_request();

MiddlewareJsonResult m_pid_def_set(cJSON* payload);
MiddlewareJsonResult m_pid_def_save(cJSON* payload);
MiddlewareJsonResult m_pid_def_load(cJSON* payload);
MiddlewareJsonResult m_settings_get();
MiddlewareJsonResult m_settings_set(cJSON* payload);

cJSON* single_pid_def_get(uint16_t pid, esp_err_t* operation_error = nullptr);
cJSON* single_pid_data_get(uint16_t pid, esp_err_t* operation_error = nullptr);

esp_err_t pid_stream_packet_get(uint16_t pid, uint8_t* out_packet);
esp_err_t can_status_packet_get(uint8_t* out_packet);
