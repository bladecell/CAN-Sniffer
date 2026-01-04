#include "cJSON.h"
#include "obd2.hpp"
#include "esp_http_server.h"

// Middlewares
cJSON *m_pid_data_json(int filter_id);
cJSON *m_pid_poll_json();
void m_pid_poll_set_running(bool running);
cJSON *m_can_bus_json();

cJSON *get_single_pid_json(uint8_t pid);
