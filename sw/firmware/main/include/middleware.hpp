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

// Middlewares
cJSON* m_pid_def_get(int filter_id);
cJSON* m_pid_data_get(int filter_id);
cJSON* m_pid_def_delete(int filter_id);
cJSON* m_pid_def_post(cJSON* data);
void   m_pid_poll_set_running(bool running);
cJSON* m_can_bus_get();
cJSON* m_obdii_get();
cJSON* m_obdii_set(cJSON* payload);
cJSON* m_system_get();
cJSON* m_system_reboot();
cJSON* m_system_copy_file(cJSON* payload);
cJSON* m_sdcard_info_get();
cJSON* m_sdcard_format_post();
cJSON* m_sdcard_file_tree_get(const char* path);
cJSON* m_sdcard_file_delete_delete(const char* path);
cJSON* m_vin_get();
cJSON* m_dtc_get(int mode);
cJSON* m_dtc_description_get(const char* target_codes[], size_t count);
cJSON* m_vin_request();
cJSON* m_dtc_request(int mode);
cJSON* m_clear_dtc_request();
cJSON* m_static_pid_request();

cJSON* m_pid_def_set(cJSON* payload);
cJSON* m_pid_def_save(cJSON* payload);
cJSON* m_pid_def_load(cJSON* payload);
cJSON* m_settings_get();
cJSON* m_settings_set(cJSON* payload);

cJSON* single_pid_def_get(uint16_t pid);
cJSON* single_pid_data_get(uint16_t pid);

esp_err_t pid_stream_packet_get(uint16_t pid, uint8_t* out_packet);
esp_err_t can_status_packet_get(uint8_t* out_packet);
