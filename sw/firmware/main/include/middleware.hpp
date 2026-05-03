// middleware.hpp
#pragma once
#include <cstdint>
#include "esp_err.h"
#include "cJSON.h"

// Forward declarations to minimize includes
class PIDDefinition;
struct PIDData_t;

#define MSG_TYPE_LOG 0x01
#define MSG_TYPE_PID 0x02
#define MSG_TYPE_CAN_STATUS 0x03

#define PID_STREAM_PACKET_SIZE 20
#define CAN_STATUS_PACKET_SIZE 8

// Middlewares
cJSON* m_pid_def_json(int filter_id);
cJSON* m_pid_data_json(int filter_id);
void   m_pid_poll_set_running(bool running);
cJSON* m_can_bus_json();
cJSON* m_obdii_json();
cJSON* m_system_json();
cJSON* m_vin_json();
cJSON* m_dtc_json(int mode);
cJSON* m_vin_request();
cJSON* m_dtc_request(int mode);
cJSON* m_clear_dtc_request();

cJSON* get_single_pid_def_json(const PIDDefinition* def);
cJSON* get_single_pid_data_json(uint16_t pid, const PIDData_t& pd);

esp_err_t get_pid_stream_packet(uint16_t pid, uint8_t* out_packet);
esp_err_t get_can_status_packet(uint8_t* out_packet);
