#include "cJSON.h"
#include "obd2.hpp"
#include "esp_http_server.h"

// Middlewares
cJSON *m_pid_def_json(int filter_id);
cJSON *m_pid_data_json(int filter_id);
void m_pid_poll_set_running(bool running);
cJSON *m_can_bus_json();
cJSON *m_obdii_json();
cJSON *m_vin_json();
cJSON *m_dtc_json(int mode);
void m_vin_request();
void m_dtc_request(int mode);

cJSON *get_single_pid_def_json(const PIDDef_t &pi);
cJSON *get_single_pid_data_json(uint8_t pid, const PIDData_t &pd);
