#include "webserver.hpp"

static const char *TAG = "WEB_SERVER";

esp_err_t setup_web_server()
{
    AsyncWebServer::Config server_config;
    server_config.async_worker_task_num = 2;
    server_config.async_worker_task_priority = 5;
    server_config.async_worker_stack_size = 8192;
    server_config.httpd_config.uri_match_fn = httpd_uri_match_wildcard;
    server_config.httpd_config.max_uri_handlers = 20;

    esp_err_t ret = AsyncWebServer::getInstance().start(server_config);
    AsyncWebServer::getInstance().registerRoute("/", HTTP_GET, index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def/*", HTTP_GET, g_pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def", HTTP_GET, g_pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/can_bus", HTTP_GET, g_can_bus_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/obd2", HTTP_GET, g_obdii_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_data/*", HTTP_GET, g_pid_data_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_data", HTTP_GET, g_pid_data_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/pid_poll*", HTTP_POST, p_pid_poll_data_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/vin", HTTP_GET, g_vin_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/dtc*", HTTP_GET, g_dtc_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/vin", HTTP_POST, p_vin_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/dtc*", HTTP_POST, p_dtc_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/clear_dtc", HTTP_POST, p_clear_dtc_index_handler, NULL);

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

bool get_query_str(httpd_req_t *req, const char *key, char *out_val, size_t val_len)
{
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len <= 1)
        return false;

    char *buf = (char *)malloc(len);
    if (!buf)
        return false;

    httpd_req_get_url_query_str(req, buf, len);
    esp_err_t err = httpd_query_key_value(buf, key, out_val, val_len);
    free(buf);

    return (err == ESP_OK);
}

esp_err_t get_query_int(httpd_req_t *req, const char *key, int *value)
{
    char val[32];
    if (get_query_str(req, key, val, sizeof(val)))
    {
        *value = atoi(val);
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t get_query_bool(httpd_req_t *req, const char *key, bool *value)
{
    char val[16];
    if (get_query_str(req, key, val, sizeof(val)))
    {
        *value = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
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

esp_err_t g_pid_def_index_handler(httpd_req_t *req, void *arg)
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

    cJSON *root = m_pid_def_json(target_pid);

    return send_json_response(req, root);
}

esp_err_t g_pid_data_index_handler(httpd_req_t *req, void *arg)
{
    int target_pid = -1;

    if (sscanf(req->uri, "/api/v1/pid_data/%d", &target_pid) == 1)
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

esp_err_t p_pid_poll_data_index_handler(httpd_req_t *req, void *arg)
{
    bool *running = (bool *)malloc(sizeof(bool));
    if (get_query_bool(req, "running", running) != ESP_OK)
    {
        free(running);
        esp_err_t ret = httpd_resp_send_404(req);
        return ret;
    }

    m_pid_poll_set_running(running);

    httpd_resp_set_status(req, "201 Created");
    esp_err_t ret = httpd_resp_send(req, NULL, 0);
    free(running);

    return ret;
}

esp_err_t g_can_bus_index_handler(httpd_req_t *req, void *arg)
{
    cJSON *root = m_can_bus_json();

    return send_json_response(req, root);
}

esp_err_t g_obdii_index_handler(httpd_req_t *req, void *arg)
{
    cJSON *root = m_obdii_json();

    return send_json_response(req, root);
}

esp_err_t g_vin_index_handler(httpd_req_t *req, void *arg)
{
    cJSON *root = m_vin_json();

    return send_json_response(req, root);
}

esp_err_t g_dtc_index_handler(httpd_req_t *req, void *arg)
{
    int mode = -1;
    if (get_query_int(req, "mode", &mode) != ESP_OK && (httpd_req_get_url_query_len(req) > 0))
    {
        esp_err_t ret = httpd_resp_send_404(req);
        return ret;
    }

    cJSON *root = m_dtc_json(mode);

    return send_json_response(req, root);
}

esp_err_t p_vin_index_handler(httpd_req_t *req, void *arg)
{
    cJSON *root = m_vin_request();

    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, root);
}

esp_err_t p_dtc_index_handler(httpd_req_t *req, void *arg)
{
    int mode = -1;
    if (get_query_int(req, "mode", &mode) != ESP_OK && (httpd_req_get_url_query_len(req) > 0))
    {
        esp_err_t ret = httpd_resp_send_404(req);
        return ret;
    }

    cJSON *root = m_dtc_request(mode);

    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, root);
}

esp_err_t p_clear_dtc_index_handler(httpd_req_t *req, void *arg)
{
    cJSON *root = m_clear_dtc_request();

    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, root);
}