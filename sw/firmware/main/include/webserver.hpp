#include "async_web_server.hpp"
#include "web_assets.h"
#include "wifi.hpp"

esp_err_t setup_web_server();

// Web server request handlers
esp_err_t index_handler(httpd_req_t *req, void *arg);
