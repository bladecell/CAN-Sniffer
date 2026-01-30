#include "cJSON.h"
#include "obd2.hpp"
#include "esp_http_server.h"

#define MSG_TYPE_LOG 0x01
#define MSG_TYPE_PID 0x02

#pragma pack(push, 1)
struct PidWirePacket
{
    uint8_t type;
    uint32_t pid_id;
    float value;
    uint32_t lastUpdated;
    uint32_t interval;
    uint8_t isSupported;
    uint8_t isValid;
};
#pragma pack(pop)

// Middlewares
cJSON *m_pid_def_json(int filter_id);
cJSON *m_pid_data_json(int filter_id);
void m_pid_poll_set_running(bool running);
cJSON *m_can_bus_json();
cJSON *m_obdii_json();
cJSON *m_vin_json();
cJSON *m_dtc_json(int mode);
cJSON *m_vin_request();
cJSON *m_dtc_request(int mode);
cJSON *m_clear_dtc_request();

cJSON *get_single_pid_def_json(const PIDDef_t &pi);
cJSON *get_single_pid_data_json(uint16_t pid, const PIDData_t &pd);

esp_err_t get_pid_stream_packet(uint16_t pid, uint8_t *out_packet);
