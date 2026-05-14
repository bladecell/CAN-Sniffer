#include "webserver.hpp"

#include <atomic>
#include <cstring>

#include "async_web_server.hpp"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "middleware.hpp"
#include "obd2.hpp"
#include "web_assets.h"
#include "wifi.hpp"

#define WS_DAT_STREAM_TASK_STACK_SIZE 2048
#define WS_DAT_STREAM_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define WS_DAT_STREAM_TASK_CORE_ID 1
#define WS_DAT_STREAM_PERIOD 1000

static const char* TAG = "WEB_SERVER";

static TaskHandle_t      xWSDataStreamTaskHandle = nullptr;
static SemaphoreHandle_t WSDataStreamSemaphore   = nullptr;
static std::atomic<bool> wsStreamingEnabled      = false;

static void WSDataStreamTask(void* pvParameters)
{
    if (WSDataStreamSemaphore == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create WSDataStreamSemaphore");
        vTaskDelete(xWSDataStreamTaskHandle);
        return;
    }

    TickType_t       xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod       = pdMS_TO_TICKS(WS_DAT_STREAM_PERIOD);

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        if (wsStreamingEnabled.load())
        {
            ws_send_can_status();
        }
        else
        {
            xSemaphoreTake(WSDataStreamSemaphore, portMAX_DELAY);
        }
    }
}

esp_err_t setup_web_server()
{
    AsyncWebServer::Config server_config;
    server_config.async_worker_task_num         = 5;
    server_config.max_open_sockets              = 7;
    server_config.async_worker_task_priority    = 5;
    server_config.async_worker_stack_size       = 8192;
    server_config.httpd_config.uri_match_fn     = httpd_uri_match_wildcard;
    server_config.httpd_config.max_uri_handlers = 24;

    esp_err_t ret = AsyncWebServer::getInstance().start(server_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start web server");
        return ret;
    }

    AsyncWebServer::getInstance().registerRoute("/", HTTP_GET, index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def/*", HTTP_GET, g_pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def/*", HTTP_DELETE, d_pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def", HTTP_GET, g_pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_def", HTTP_POST, p_pid_def_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/can_bus", HTTP_GET, g_can_bus_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/obd2", HTTP_GET, g_obdii_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_data/*", HTTP_GET, g_pid_data_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/pid_data", HTTP_GET, g_pid_data_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/pid_poll*", HTTP_POST, p_pid_poll_data_index_handler,
                                                NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/vin", HTTP_GET, g_vin_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/dtc*", HTTP_GET, g_dtc_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/vin", HTTP_POST, p_vin_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/dtc*", HTTP_POST, p_dtc_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/req/clear_dtc", HTTP_POST, p_clear_dtc_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/system", HTTP_GET, g_system_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/system/reboot", HTTP_POST, p_system_reboot_index_handler,
                                                NULL);

    AsyncWebServer::getInstance().registerRoute("/api/v1/settings/wifi", HTTP_GET, g_settings_wifi_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/settings/can", HTTP_GET, g_settings_can_index_handler, NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/settings/wifi", HTTP_POST, p_settings_wifi_index_handler,
                                                NULL);
    AsyncWebServer::getInstance().registerRoute("/api/v1/settings/can", HTTP_POST, p_settings_can_index_handler, NULL);

    AsyncWebServer::getInstance().registerSocketRoute("/ws", ws_socket_handler, NULL);

    OBD2::getInstance().subscribe(pid_stream_callback);

    WSDataStreamSemaphore = xSemaphoreCreateBinary();

    BaseType_t result =
        xTaskCreatePinnedToCore(WSDataStreamTask, "WSDataStreamTask", WS_DAT_STREAM_TASK_STACK_SIZE, NULL,
                                WS_DAT_STREAM_TASK_PRIORITY, &xWSDataStreamTaskHandle, WS_DAT_STREAM_TASK_CORE_ID);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create WSDataStreamTask!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t send_json_response(httpd_req_t* req, cJSON* root)
{
    const char* json_str = NULL;

    json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to print JSON");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON Error");
    }

    httpd_resp_set_type(req, "application/json");

    esp_err_t ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    free((void*)json_str);
    cJSON_Delete(root);

    return ret;
}

bool get_query_str(httpd_req_t* req, const char* key, char* out_val, size_t val_len)
{
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len <= 1)
        return false;

    char* buf = (char*)malloc(len);
    if (!buf)
        return false;

    httpd_req_get_url_query_str(req, buf, len);
    esp_err_t err = httpd_query_key_value(buf, key, out_val, val_len);
    free(buf);

    return (err == ESP_OK);
}

esp_err_t get_query_int(httpd_req_t* req, const char* key, int* value)
{
    char val[32];
    if (get_query_str(req, key, val, sizeof(val)))
    {
        *value = atoi(val);
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t get_query_bool(httpd_req_t* req, const char* key, bool* value)
{
    char val[16];
    if (get_query_str(req, key, val, sizeof(val)))
    {
        *value = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t index_handler(httpd_req_t* req, void* arg)
{
    httpd_resp_set_type(req, "text/html");

    // Check the boolean generated in your header
    if (INDEX_HTML_IS_GZ)
    {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    }

    // Use the names that exist in your web_assets.h
    return httpd_resp_send(req, (const char*)INDEX_HTML, INDEX_HTML_LEN);
}

esp_err_t g_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;

    if (sscanf(req->uri, "/api/v1/pid_def/%d", &target_pid) == 1)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            ESP_LOGW(TAG, "Invalid PID requested: %d", target_pid);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
        }
    }

    cJSON* root = m_pid_def_get(target_pid);

    return send_json_response(req, root);
}

esp_err_t g_pid_data_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;

    if (sscanf(req->uri, "/api/v1/pid_data/%d", &target_pid) == 1)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            ESP_LOGW(TAG, "Invalid PID requested: %d", target_pid);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
        }
    }

    cJSON* root = m_pid_data_get(target_pid);

    return send_json_response(req, root);
}

esp_err_t p_pid_poll_data_index_handler(httpd_req_t* req, void* arg)
{
    bool running = false;
    if (get_query_bool(req, "running", &running) != ESP_OK)
    {
        esp_err_t ret = httpd_resp_send_500(req);
        return ret;
    }

    m_pid_poll_set_running(running);

    httpd_resp_set_status(req, "201 Created");
    esp_err_t ret = httpd_resp_send(req, NULL, 0);

    return ret;
}

esp_err_t g_can_bus_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = m_can_bus_get();

    return send_json_response(req, root);
}

esp_err_t g_obdii_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = m_obdii_get();

    return send_json_response(req, root);
}

esp_err_t g_vin_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = m_vin_get();

    return send_json_response(req, root);
}

esp_err_t g_dtc_index_handler(httpd_req_t* req, void* arg)
{
    int mode = -1;
    if (get_query_int(req, "mode", &mode) != ESP_OK && (httpd_req_get_url_query_len(req) > 0))
    {
        esp_err_t ret = httpd_resp_send_404(req);
        return ret;
    }

    cJSON* root = m_dtc_get(mode);

    return send_json_response(req, root);
}

esp_err_t p_vin_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = m_vin_request();

    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, root);
}

esp_err_t p_dtc_index_handler(httpd_req_t* req, void* arg)
{
    int mode = -1;
    if (get_query_int(req, "mode", &mode) != ESP_OK && (httpd_req_get_url_query_len(req) > 0))
    {
        esp_err_t ret = httpd_resp_send_404(req);
        return ret;
    }

    cJSON* root = m_dtc_request(mode);

    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, root);
}

esp_err_t p_clear_dtc_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = m_clear_dtc_request();

    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, root);
}

esp_err_t ws_socket_handler(httpd_req_t* req)
{
    int sockfd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "WS Connect: Client #%d", sockfd);
        wsStreamingEnabled.store(true);
        xSemaphoreGive(WSDataStreamSemaphore);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);

    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "WS Client #%d Read Error (Likely Disconnected)", sockfd);
        return ret;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
    {
        ESP_LOGI(TAG, "WS Client #%d Sent Close Frame", sockfd);
        uint32_t wcClientCount = AsyncWebServer::getInstance().getActiveWSClientCount();
        if (wcClientCount <= 1)
        {
            enable_pid_stream(false);
            wsStreamingEnabled.store(false);
        }
        return httpd_ws_send_frame(req, &ws_pkt);
    }

    if (ws_pkt.len > 0)
    {
        uint8_t* buf = (uint8_t*)malloc(ws_pkt.len + 1);
        if (!buf)
            return ESP_ERR_NO_MEM;

        ws_pkt.payload = buf;
        ret            = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);

        if (ret == ESP_OK)
        {
            if (ws_pkt.type == HTTPD_WS_TYPE_TEXT)
            {
                buf[ws_pkt.len] = 0;
                ESP_LOGD(TAG, "WS Text: %s", (char*)buf);
            }
            else if (ws_pkt.type == HTTPD_WS_TYPE_BINARY)
            {
                uint8_t command = buf[0];
                switch (command)
                {
                    case WS_START_PID_STREAM:
                        enable_pid_stream(true);
                        ESP_LOGI(TAG, "Starting PID Stream");
                        break;
                    case WS_STOP_PID_STREAM:
                        enable_pid_stream(false);
                        ESP_LOGI(TAG, "Stopping PID Stream");
                        break;
                    default:
                        ESP_LOGW(TAG, "Unknown WS Command: 0x%02X", command);
                }
            }
        }
        free(buf);
    }
    return ret;
}

static bool b_pid_stream_enabled = false;

inline void enable_pid_stream(bool enable)
{
    b_pid_stream_enabled = enable;
}

void pid_stream_callback(uint16_t pid)
{
    if (!b_pid_stream_enabled)
        return;

    uint8_t packet[PID_STREAM_PACKET_SIZE];

    esp_err_t err = pid_stream_packet_get(pid, packet);

    if (err == ESP_OK)
    {
        httpd_ws_frame_t ws_frame;
        memset(&ws_frame, 0, sizeof(httpd_ws_frame_t));
        ws_frame.payload = packet;
        ws_frame.len     = PID_STREAM_PACKET_SIZE;
        ws_frame.type    = HTTPD_WS_TYPE_BINARY;

        AsyncWebServer::getInstance().wsBroadcast(&ws_frame);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to get PID stream packet for PID 0x%02X - %s", pid, esp_err_to_name(err));
    }
}

esp_err_t g_system_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = m_system_get();

    return send_json_response(req, root);
}

void ws_send_can_status()
{
    // Use a static buffer because httpd_ws_send_frame_async does not copy the payload.
    static uint8_t packet[CAN_STATUS_PACKET_SIZE];

    esp_err_t err = can_status_packet_get(packet);

    if (err == ESP_OK)
    {
        httpd_ws_frame_t ws_frame;
        memset(&ws_frame, 0, sizeof(httpd_ws_frame_t));
        ws_frame.payload = packet;
        ws_frame.len     = CAN_STATUS_PACKET_SIZE;
        ws_frame.type    = HTTPD_WS_TYPE_BINARY;

        AsyncWebServer::getInstance().wsBroadcast(&ws_frame);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to get Can Status stream packet - %s", esp_err_to_name(err));
    }
}

cJSON* get_validated_json_payload(httpd_req_t* req, size_t max_size)
{
    size_t total_len = req->content_len;

    if (total_len <= 0)
    {
        ESP_LOGW(TAG, "Empty content length");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length required");
        return NULL;
    }

    if (total_len > max_size)
    {
        ESP_LOGW(TAG, "Payload too large: %d bytes", total_len);
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "JSON too large");
        return NULL;
    }

    char* buf = (char*)malloc(total_len + 1);
    if (buf == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server OOM");
        return NULL;
    }

    int ret = httpd_req_recv(req, buf, total_len);
    if (ret <= 0)
    {
        free(buf);
        return NULL;
    }
    buf[ret] = '\0';

    cJSON* root = cJSON_Parse(buf);
    free(buf);

    if (root == NULL)
    {
        ESP_LOGW(TAG, "Invalid JSON format");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return NULL;
    }

    return root;
}

esp_err_t p_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 1024);

    if (root == NULL)
    {
        return ESP_FAIL;
    }
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t d_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;

    if (sscanf(req->uri, "/api/v1/pid_def/%d", &target_pid) == 1)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            ESP_LOGW(TAG, "Invalid PID requested: %d", target_pid);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
        }
    }

    cJSON* root = m_pid_def_delete(target_pid);

    return send_json_response(req, root);
}

esp_err_t g_settings_wifi_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t g_settings_can_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t p_settings_wifi_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t p_settings_can_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t p_system_reboot_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}