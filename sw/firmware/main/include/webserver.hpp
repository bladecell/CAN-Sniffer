#include "async_web_server.hpp"
#include "web_assets.h"
#include "wifi.hpp"
#include "middleware.hpp"

esp_err_t setup_web_server();

esp_err_t send_json_response(httpd_req_t *req, cJSON *root);
bool get_query_bool(httpd_req_t *req, const char *key, bool default_val = false);

// Web server request handlers
esp_err_t index_handler(httpd_req_t *req, void *arg);
esp_err_t pid_def_index_handler(httpd_req_t *req, void *arg);
esp_err_t pid_data_index_handler(httpd_req_t *req, void *arg);
esp_err_t pid_poll_index_handler(httpd_req_t *req, void *arg);
esp_err_t set_pid_poll_index_handler(httpd_req_t *req, void *arg);
esp_err_t can_bus_index_handler(httpd_req_t *req, void *arg);