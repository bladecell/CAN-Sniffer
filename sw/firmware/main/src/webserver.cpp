#include "webserver.hpp"

static const char *TAG = "WEB_SERVER";

esp_err_t setup_web_server()
{
    AsyncWebServer::Config server_config;
    server_config.async_worker_task_num = 2;
    server_config.async_worker_task_priority = 5;
    server_config.async_worker_stack_size = 8192;
    server_config.httpd_config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t ret = AsyncWebServer::getInstance().start(server_config);
    AsyncWebServer::getInstance().registerRoute("/", HTTP_GET, index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def/*", HTTP_GET, pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def", HTTP_GET, pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_poll", HTTP_GET, pid_poll_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_poll*", HTTP_POST, set_pid_poll_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/can_bus", HTTP_GET, can_bus_index_handler, NULL);

    return ret;
}

esp_err_t send_json_response(httpd_req_t *req, cJSON *root)
{
    const char *json_str = NULL;

    json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to print JSON");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON Error");
    }

    httpd_resp_set_type(req, "application/json");

    esp_err_t ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    free((void *)json_str);
    cJSON_Delete(root);

    return ret;
}

bool get_query_bool(httpd_req_t *req, const char *key, bool default_val)
{
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len <= 1)
        return default_val;

    char *buf = (char *)malloc(len);
    if (!buf)
        return default_val;

    httpd_req_get_url_query_str(req, buf, len);

    char value[16] = {0};
    bool result = default_val;

    if (httpd_query_key_value(buf, key, value, sizeof(value)) == ESP_OK)
    {
        result = (strcasecmp(value, "true") == 0 || strcmp(value, "1") == 0);
    }

    free(buf);
    return result;
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

esp_err_t pid_def_index_handler(httpd_req_t *req, void *arg)
{
    int target_pid = -1;

    if (sscanf(req->uri, "/api/v1/pid_def/%d", &target_pid) == 1)
    {
        if (target_pid < 0 || target_pid > 255)
        {
            ESP_LOGW(TAG, "Invalid PID requested: %d", target_pid);
            return httpd_resp_send_404(req);
        }
    }

    cJSON *root = m_pid_data_json(target_pid);

    return send_json_response(req, root);
}

esp_err_t pid_poll_index_handler(httpd_req_t *req, void *arg)
{
    cJSON *root = m_pid_poll_json();

    return send_json_response(req, root);
}

esp_err_t set_pid_poll_index_handler(httpd_req_t *req, void *arg)
{
    bool running = get_query_bool(req, "running");

    m_pid_poll_set_running(running);

    httpd_resp_set_status(req, "201 Created");
    esp_err_t ret = httpd_resp_send(req, NULL, 0);

    return ret;
}

esp_err_t can_bus_index_handler(httpd_req_t *req, void *arg)
{
    cJSON *root = m_can_bus_json();

    return send_json_response(req, root);
}