#include "webserver.hpp"
#include "cJSON.h"

esp_err_t setup_web_server()
{
    AsyncWebServer::Config server_config;
    server_config.async_worker_task_num = 2;
    server_config.async_worker_task_priority = 5;
    server_config.async_worker_stack_size = 8192;

    esp_err_t ret = AsyncWebServer::getInstance().start(server_config);
    AsyncWebServer::getInstance().registerRoute("/", HTTP_GET, index_handler, NULL);

    return ret;
}

esp_err_t index_handler(httpd_req_t *req, void *arg)
{
    httpd_resp_set_type(req, "text/html");

    // Check the boolean generated in your header
    if (INDEX_HTML_IS_GZ)
    {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    }

    // Use the names that exist in your web_assets.h
    return httpd_resp_send(req, (const char *)INDEX_HTML, INDEX_HTML_LEN);
}