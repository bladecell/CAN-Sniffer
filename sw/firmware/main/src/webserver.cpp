#include "webserver.hpp"

#include <sys/param.h>

#include <atomic>
#include <cstring>

#include "async_web_server.hpp"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "http_parser.h"
#include "middleware.hpp"
#include "obd2.hpp"
#include "web_assets.h"
#include "wifi.hpp"

struct RouteDef
{
    const char*                  uri;
    httpd_method_t               method;
    AsyncWebServer::AsyncHandler handler;
};

#define WS_DAT_STREAM_TASK_STACK_SIZE 4096
#define WS_DAT_STREAM_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define WS_DAT_STREAM_TASK_CORE_ID 1
#define WS_DAT_STREAM_PERIOD 1000
#define MAX_DTC_CODES_QUERY 30

static const char* TAG = "WEB_SERVER";

static TaskHandle_t      xWSDataStreamTaskHandle = nullptr;
static SemaphoreHandle_t WSDataStreamSemaphore   = nullptr;
static std::atomic<bool> wsStreamingEnabled{false};
static bool              b_pid_stream_enabled = false;

namespace
{

// ============================================================================
// 1. HTTP & PARSING UTILITIES
// ============================================================================

esp_err_t send_json_response(httpd_req_t* req, cJSON* root)
{
    const char* json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to print JSON");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON Error");
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    cJSON_free((void*)json_str);
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

esp_err_t get_query_list(httpd_req_t* req, const char* key, char* scratch_buf, size_t scratch_len,
                         const char* out_ptrs[], size_t max_ptrs, size_t* out_count)
{
    *out_count = 0;

    if (!get_query_str(req, key, scratch_buf, scratch_len))
    {
        return ESP_ERR_NOT_FOUND;
    }

    char* save_ptr = nullptr;
    char* token    = strtok_r(scratch_buf, ",", &save_ptr);

    while (token != nullptr && (*out_count) < max_ptrs)
    {
        out_ptrs[*out_count] = token;
        (*out_count)++;
        token = strtok_r(nullptr, ",", &save_ptr);
    }

    return (*out_count > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
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

cJSON* get_validated_json_payload(httpd_req_t* req, size_t max_size)
{
    size_t total_len = req->content_len;

    if (total_len <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length required");
        return NULL;
    }

    if (total_len > max_size)
    {
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "JSON too large");
        return NULL;
    }

    char* buf = (char*)malloc(total_len + 1);
    if (buf == NULL)
    {
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
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");

    return root;
}

static void set_content_type_from_file(httpd_req_t* req, const char* filepath)
{
    if (strstr(filepath, ".html"))
        httpd_resp_set_type(req, "text/html");
    else if (strstr(filepath, ".css"))
        httpd_resp_set_type(req, "text/css");
    else if (strstr(filepath, ".js"))
        httpd_resp_set_type(req, "application/javascript");
    else if (strstr(filepath, ".png"))
        httpd_resp_set_type(req, "image/png");
    else if (strstr(filepath, ".jpg"))
        httpd_resp_set_type(req, "image/jpeg");
    else if (strstr(filepath, ".csv"))
        httpd_resp_set_type(req, "text/csv");
    else if (strstr(filepath, ".json"))
        httpd_resp_set_type(req, "application/json");
    else if (strstr(filepath, ".txt"))
        httpd_resp_set_type(req, "text/plain");
    else
        httpd_resp_set_type(req, "application/octet-stream");
}

// ============================================================================
// 2. ROUTE HANDLERS
// ============================================================================

esp_err_t index_handler(httpd_req_t* req, void* arg)
{
    httpd_resp_set_type(req, "text/html");
    if (INDEX_HTML_IS_GZ)
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    return httpd_resp_send(req, (const char*)INDEX_HTML, INDEX_HTML_LEN);
}

esp_err_t g_system_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_system_get());
}

esp_err_t p_system_reboot_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t g_can_bus_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_can_bus_get());
}

esp_err_t g_obdii_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_obdii_get());
}

esp_err_t g_vin_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_vin_get());
}

esp_err_t p_vin_index_handler(httpd_req_t* req, void* arg)
{
    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, m_vin_request());
}

esp_err_t g_dtc_index_handler(httpd_req_t* req, void* arg)
{
    int mode = -1;
    if (httpd_req_get_url_query_len(req) == 0)
    {
        return send_json_response(req, m_dtc_get(mode));
    }

    if (get_query_int(req, "mode", &mode) == ESP_OK)
    {
        cJSON* data = m_dtc_get(mode);
        return send_json_response(req, data);
    }

    char        scratch[(MAX_DTC_CODES_QUERY * 6) + 2];
    const char* codes[MAX_DTC_CODES_QUERY];
    size_t      count = 0;

    if (get_query_list(req, "codes", scratch, sizeof(scratch), codes, MAX_DTC_CODES_QUERY, &count) == ESP_OK)
    {
        cJSON* data = m_dtc_description_get(codes, count);
        return send_json_response(req, data);
    }

    return httpd_resp_send_404(req);
}

esp_err_t p_dtc_index_handler(httpd_req_t* req, void* arg)
{
    int mode = -1;
    if (get_query_int(req, "mode", &mode) != ESP_OK && (httpd_req_get_url_query_len(req) > 0))
    {
        return httpd_resp_send_404(req);
    }
    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, m_dtc_request(mode));
}

esp_err_t p_clear_dtc_index_handler(httpd_req_t* req, void* arg)
{
    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, m_clear_dtc_request());
}

esp_err_t g_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;
    if (sscanf(req->uri, "/api/v1/pid_def/%d", &target_pid) == 1)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
        }
    }
    return send_json_response(req, m_pid_def_get(target_pid));
}

esp_err_t p_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 1024);
    if (root == NULL)
        return ESP_FAIL;

    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t d_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;
    if (sscanf(req->uri, "/api/v1/pid_def/%d", &target_pid) == 1)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
        }
    }
    return send_json_response(req, m_pid_def_delete(target_pid));
}

esp_err_t g_pid_data_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;
    if (sscanf(req->uri, "/api/v1/pid_data/%d", &target_pid) == 1)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
        }
    }
    return send_json_response(req, m_pid_data_get(target_pid));
}

esp_err_t p_pid_poll_data_index_handler(httpd_req_t* req, void* arg)
{
    bool running = false;
    if (get_query_bool(req, "running", &running) != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid query parameter");
    }

    m_pid_poll_set_running(running);
    httpd_resp_set_status(req, "201 Created");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t p_static_pid_index_handler(httpd_req_t* req, void* arg)
{
    httpd_resp_set_status(req, "200 OK");
    return send_json_response(req, m_static_pid_request());
}

esp_err_t g_settings_wifi_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}
esp_err_t p_settings_wifi_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}
esp_err_t g_settings_can_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}
esp_err_t p_settings_can_index_handler(httpd_req_t* req, void* arg)
{
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not Implemented");
}

esp_err_t g_sd_card_info_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_sdcard_info_get());
}

esp_err_t g_sd_card_file_tree_handler(httpd_req_t* req, void* arg)
{
    const char* api_route     = "/api/v1/sd_card/tree";
    const char* relative_path = req->uri + strlen(api_route);

    return send_json_response(req, m_sdcard_file_tree_get(relative_path));
}

esp_err_t g_sd_card_file_read_handler(httpd_req_t* req, void* arg)
{
    const char* api_route = "/api/v1/sd_card/file";

    bool download = false;
    get_query_bool(req, "download", &download);

    const char* relative_path = req->uri + strlen(api_route);

    char clean_path[256];
    strlcpy(clean_path, relative_path, sizeof(clean_path));
    char* query_ptr = strchr(clean_path, '?');
    if (query_ptr)
        *query_ptr = '\0';

    if (strstr(clean_path, "..") || strlen(clean_path) <= 1)
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Invalid path");

    FILE*     fd  = nullptr;
    esp_err_t err = m_sdcard_file_open_file(clean_path, "r", fd);

    if (err != ESP_OK || !fd)
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    }

    set_content_type_from_file(req, clean_path);

    if (download)
    {
        const char* filename = strrchr(clean_path, '/');
        filename             = (filename != nullptr) ? (filename + 1) : clean_path;

        char disp_header[128];
        snprintf(disp_header, sizeof(disp_header), "attachment; filename=\"%s\"", filename);
        httpd_resp_set_hdr(req, "Content-Disposition", disp_header);
    }
    else
    {
        httpd_resp_set_hdr(req, "Content-Disposition", "inline");
        httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600");
    }

    char   chunk[1024];
    size_t read_bytes;

    while ((read_bytes = m_sdcard_file_read_chunk(fd, chunk, sizeof(chunk))) > 0)
    {
        if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK)
        {
            m_sdcard_file_close_file(fd);
            return ESP_FAIL;
        }
    }

    httpd_resp_send_chunk(req, NULL, 0);
    m_sdcard_file_close_file(fd);

    return ESP_OK;
}

esp_err_t p_file_upload_handler(httpd_req_t* req, void* arg)
{
    const char* api_route     = "/api/v1/sd_card/file";
    const char* relative_path = req->uri + strlen(api_route);

    FILE*     fd  = nullptr;
    esp_err_t err = m_sdcard_file_open_file(relative_path, "w", fd);

    if (err != ESP_OK || !fd)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", relative_path);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Storage error");
    }

    int  remaining = req->content_len;
    int  received;
    char chunk[1024];

    while (remaining > 0)
    {
        if ((received = httpd_req_recv(req, chunk, MIN(remaining, sizeof(chunk)))) <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
                continue;

            m_sdcard_file_close_file(fd);
            m_sdcard_file_delete_delete(relative_path);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload timeout");
        }

        if (m_sdcard_file_write_chunk(fd, chunk, received) != ESP_OK)
        {
            m_sdcard_file_close_file(fd);
            m_sdcard_file_delete_delete(relative_path);
            ESP_LOGE(TAG, "Disk write failed!");
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write error");
        }

        remaining -= received;
    }

    m_sdcard_file_close_file(fd);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "success");

    return send_json_response(req, root);
}

esp_err_t d_file_delete_handler(httpd_req_t* req, void* arg)
{
    const char* api_route     = "/api/v1/sd_card/file";  // FIX: mapped to /file
    const char* relative_path = req->uri + strlen(api_route);

    return send_json_response(req, m_sdcard_file_delete_delete(relative_path));
}

// ============================================================================
// 3. WEBSOCKETS & LIVE STREAMING
// ============================================================================

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
        return ret;

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
    {
        ESP_LOGI(TAG, "WS Client #%d Closed", sockfd);
        if (AsyncWebServer::getInstance().getActiveWSClientCount() <= 1)
        {
            b_pid_stream_enabled = false;
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
                switch (buf[0])
                {
                    case WS_START_PID_STREAM:
                        b_pid_stream_enabled = true;
                        break;
                    case WS_STOP_PID_STREAM:
                        b_pid_stream_enabled = false;
                        break;
                }
            }
        }
        free(buf);
    }
    return ret;
}

void pid_stream_callback(uint16_t pid)
{
    if (!b_pid_stream_enabled)
        return;

    static uint8_t packet[PID_STREAM_PACKET_SIZE];
    esp_err_t      err = pid_stream_packet_get(pid, packet);

    if (err == ESP_OK)
    {
        httpd_ws_frame_t ws_frame = {.final      = true,
                                     .fragmented = false,
                                     .type       = HTTPD_WS_TYPE_BINARY,
                                     .payload    = packet,
                                     .len        = PID_STREAM_PACKET_SIZE};
        AsyncWebServer::getInstance().wsBroadcast(&ws_frame);
    }
}

void ws_send_can_status()
{
    static uint8_t packet[CAN_STATUS_PACKET_SIZE];
    esp_err_t      err = can_status_packet_get(packet);

    if (err == ESP_OK)
    {
        httpd_ws_frame_t ws_frame = {.final      = true,
                                     .fragmented = false,
                                     .type       = HTTPD_WS_TYPE_BINARY,
                                     .payload    = packet,
                                     .len        = CAN_STATUS_PACKET_SIZE};
        AsyncWebServer::getInstance().wsBroadcast(&ws_frame);
    }
}

static void WSDataStreamTask(void* pvParameters)
{
    if (WSDataStreamSemaphore == nullptr)
    {
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

// ============================================================================
// 4. ROUTE TABLE & REGISTRATION
// ============================================================================

const RouteDef api_routes[] = {{"/", HTTP_GET, index_handler},

                               {"/api/v1/pid_def/*", HTTP_GET, g_pid_def_index_handler},
                               {"/api/v1/pid_def/*", HTTP_DELETE, d_pid_def_index_handler},
                               {"/api/v1/pid_def", HTTP_GET, g_pid_def_index_handler},
                               {"/api/v1/pid_def", HTTP_POST, p_pid_def_index_handler},

                               {"/api/v1/can_bus", HTTP_GET, g_can_bus_index_handler},
                               {"/api/v1/obd2", HTTP_GET, g_obdii_index_handler},

                               {"/api/v1/pid_data/*", HTTP_GET, g_pid_data_index_handler},
                               {"/api/v1/pid_data", HTTP_GET, g_pid_data_index_handler},
                               {"/api/v1/req/pid_poll*", HTTP_POST, p_pid_poll_data_index_handler},
                               {"/api/v1/req/static_pid", HTTP_POST, p_static_pid_index_handler},

                               {"/api/v1/vin", HTTP_GET, g_vin_index_handler},
                               {"/api/v1/req/vin", HTTP_POST, p_vin_index_handler},
                               {"/api/v1/dtc*", HTTP_GET, g_dtc_index_handler},
                               {"/api/v1/req/dtc*", HTTP_POST, p_dtc_index_handler},
                               {"/api/v1/req/clear_dtc", HTTP_POST, p_clear_dtc_index_handler},

                               {"/api/v1/system", HTTP_GET, g_system_index_handler},
                               {"/api/v1/system/reboot", HTTP_POST, p_system_reboot_index_handler},

                               {"/api/v1/sd_card/info", HTTP_GET, g_sd_card_info_handler},
                               {"/api/v1/sd_card/tree", HTTP_GET, g_sd_card_file_tree_handler},
                               {"/api/v1/sd_card/tree/*", HTTP_GET, g_sd_card_file_tree_handler},
                               {"/api/v1/sd_card/file/*", HTTP_POST, p_file_upload_handler},
                               {"/api/v1/sd_card/file/*", HTTP_DELETE, d_file_delete_handler},
                               {"/api/v1/sd_card/file/*", HTTP_GET, g_sd_card_file_read_handler},

                               {"/api/v1/settings/wifi", HTTP_GET, g_settings_wifi_index_handler},
                               {"/api/v1/settings/wifi", HTTP_POST, p_settings_wifi_index_handler},
                               {"/api/v1/settings/can", HTTP_GET, g_settings_can_index_handler},
                               {"/api/v1/settings/can", HTTP_POST, p_settings_can_index_handler}};

static void register_routes()
{
    AsyncWebServer& server     = AsyncWebServer::getInstance();
    size_t          num_routes = sizeof(api_routes) / sizeof(api_routes[0]);

    for (size_t i = 0; i < num_routes; i++)
    {
        server.registerRoute(api_routes[i].uri, api_routes[i].method, api_routes[i].handler, NULL);
    }
    server.registerSocketRoute("/ws", ws_socket_handler, NULL);
}

}  // namespace

// ============================================================================
// 5. PUBLIC BOOTSTRAPPER (Sits at the bottom looking up at the namespace)
// ============================================================================

esp_err_t setup_web_server()
{
    AsyncWebServer::Config server_config;
    server_config.async_worker_task_num         = 4;
    server_config.max_open_sockets              = 7;
    server_config.async_worker_task_priority    = 5;
    server_config.async_worker_stack_size       = 8192;
    server_config.httpd_config.uri_match_fn     = httpd_uri_match_wildcard;
    server_config.httpd_config.max_uri_handlers = 32;

    esp_err_t ret = AsyncWebServer::getInstance().start(server_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start web server");
        return ret;
    }

    register_routes();

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