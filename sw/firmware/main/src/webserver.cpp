#include "webserver.hpp"

#include <sys/param.h>
#include <sys/stat.h>

#include <atomic>
#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "async_web_server.hpp"
#include "cJSON.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "http_parser.h"
#include "middleware.hpp"
#include "obd2.hpp"
#include "sd_card.hpp"

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

static constexpr int64_t JSON_PAYLOAD_RECEIVE_DEADLINE_US = 5 * 1000 * 1000;
static constexpr size_t  MAX_UPLOAD_CONTENT_LENGTH        = 16 * 1024 * 1024;
static constexpr int64_t UPLOAD_NO_PROGRESS_DEADLINE_US   = 5 * 1000 * 1000;
static constexpr int64_t UPLOAD_ABSOLUTE_DEADLINE_US      = 60 * 1000 * 1000;
static constexpr TickType_t UPLOAD_OPERATION_TIMEOUT       = pdMS_TO_TICKS(250);
static constexpr TickType_t SD_FILE_READ_OPERATION_TIMEOUT = pdMS_TO_TICKS(250);

static TaskHandle_t      xWSDataStreamTaskHandle = nullptr;
static SemaphoreHandle_t WSDataStreamSemaphore   = nullptr;
static std::atomic<bool> wsStreamingEnabled{false};
static std::atomic<bool> b_pid_stream_enabled{false};

namespace
{

// ============================================================================
// 1. HTTP & PARSING UTILITIES
// ============================================================================

// Extracts, percent-decodes and validates an SD-card route suffix. Separators
// must be literal slashes so an encoded delimiter cannot change path structure.
static esp_err_t get_sd_rest_path(httpd_req_t* req, const char* api_route, char* out, size_t out_size,
                                  bool allow_root)
{
    if (req == nullptr || api_route == nullptr || out == nullptr || out_size == 0)
        return ESP_ERR_INVALID_ARG;

    const size_t route_len = strlen(api_route);
    if (strncmp(req->uri, api_route, route_len) != 0)
        return ESP_ERR_INVALID_ARG;

    const char* read = req->uri + route_len;
    if (*read != '\0' && *read != '/' && *read != '?')
        return ESP_ERR_INVALID_ARG;

    char*       write = out;
    char*       end   = out + out_size - 1;

    while (*read != '\0' && *read != '?')
    {
        unsigned char decoded;
        if (*read == '%')
        {
            if (read[1] == '\0' || read[2] == '\0' || !std::isxdigit((unsigned char)read[1]) ||
                !std::isxdigit((unsigned char)read[2]))
                return ESP_ERR_INVALID_ARG;

            char hex[3] = {read[1], read[2], '\0'};
            decoded     = (unsigned char)std::strtol(hex, NULL, 16);
            read += 3;

            if (decoded == '/' || decoded == '\\' || decoded == '?' || decoded == '#')
                return ESP_ERR_INVALID_ARG;
        }
        else
        {
            decoded = (unsigned char)*read++;
        }

        if (decoded == 0 || decoded < 0x20 || decoded == 0x7f || decoded == '\\' || decoded == '#')
            return ESP_ERR_INVALID_ARG;
        if (write == end)
            return ESP_ERR_NO_MEM;
        *write++ = (char)decoded;
    }

    // URI fragments are not valid in HTTP requests. Reject them even after a
    // legitimate query delimiter rather than allowing parser disagreement.
    if (strchr(read, '#') != nullptr)
        return ESP_ERR_INVALID_ARG;

    *write = '\0';
    return SDCard::validate_relative_path(out, allow_root);
}

static const char* http_status_text(int status)
{
    switch (status)
    {
        case 200: return "200 OK";
        case 201: return "201 Created";
        case 204: return "204 No Content";
        case 400: return "400 Bad Request";
        case 401: return "401 Unauthorized";
        case 403: return "403 Forbidden";
        case 404: return "404 Not Found";
        case 408: return "408 Request Timeout";
        case 409: return "409 Conflict";
        case 413: return "413 Content Too Large";
        case 422: return "422 Unprocessable Entity";
        case 429: return "429 Too Many Requests";
        case 500: return "500 Internal Server Error";
        case 503: return "503 Service Unavailable";
        case 504: return "504 Gateway Timeout";
        default: return nullptr;
    }
}

static const char* json_error_body(int status)
{
    switch (status)
    {
        case 400: return "{\"status\":\"error\",\"reason\":\"Bad request\"}";
        case 404: return "{\"status\":\"error\",\"reason\":\"Not found\"}";
        case 408: return "{\"status\":\"error\",\"reason\":\"Request timeout\"}";
        case 413: return "{\"status\":\"error\",\"reason\":\"Content too large\"}";
        case 422: return "{\"status\":\"error\",\"reason\":\"Unprocessable entity\"}";
        case 503: return "{\"status\":\"error\",\"reason\":\"Service unavailable\"}";
        case 504: return "{\"status\":\"error\",\"reason\":\"Gateway timeout\"}";
        default: return "{\"status\":\"error\",\"reason\":\"Internal server error\"}";
    }
}

static esp_err_t send_json_error_response(httpd_req_t* req, int status)
{
    if (http_status_text(status) == nullptr)
        status = 500;

    esp_err_t ret = httpd_resp_set_status(req, http_status_text(status));
    if (ret != ESP_OK)
        return ret;
    ret = httpd_resp_set_type(req, "application/json");
    if (ret != ESP_OK)
        return ret;
    return httpd_resp_send(req, json_error_body(status), HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_json_response(httpd_req_t* req, MiddlewareJsonResult result)
{
    cJSON* root = result.body;
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "Failed to allocate JSON response");
        // A null body can accompany a meaningful middleware status. Preserve
        // mapped non-2xx statuses, but treat a successful/null result as OOM.
        const int fallback_status = (result.http_status >= 300 && result.http_status < 600 &&
                                     http_status_text(result.http_status) != nullptr)
                                        ? result.http_status
                                        : 500;
        return send_json_error_response(req, fallback_status);
    }

    const char* json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to print JSON");
        return send_json_error_response(req, 500);
    }

    if (result.http_status != 200)
    {
        const char* status_text = http_status_text(result.http_status);
        httpd_resp_set_status(req, status_text != nullptr ? status_text : http_status_text(500));
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    cJSON_free((void*)json_str);
    cJSON_Delete(root);
    return ret;
}

static MiddlewareJsonResult make_upload_result(const char* status, const char* reason, int http_status)
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
        return {nullptr, 500};
    cJSON_AddStringToObject(root, "status", status);
    if (reason != nullptr)
        cJSON_AddStringToObject(root, "reason", reason);
    return {root, http_status};
}

// req->content_len is zero both when Content-Length is absent and when the
// request explicitly declares an empty body. SD uploads need to distinguish
// those cases before acquiring an SD operation or changing a target path.
static bool get_validated_upload_content_length(httpd_req_t* req, size_t* content_length)
{
    if (req == nullptr || content_length == nullptr)
        return false;

    const size_t header_length = httpd_req_get_hdr_value_len(req, "Content-Length");
    if (header_length == 0 || header_length >= CONFIG_HTTPD_MAX_REQ_HDR_LEN)
        return false;

    char header_value[CONFIG_HTTPD_MAX_REQ_HDR_LEN];
    if (httpd_req_get_hdr_value_str(req, "Content-Length", header_value, sizeof(header_value)) != ESP_OK)
        return false;

    bool   saw_digit          = false;
    bool   trailing_whitespace = false;
    bool   exceeds_size_limit  = false;
    size_t parsed_length       = 0;
    for (const char* value = header_value; *value != '\0'; ++value)
    {
        if (*value >= '0' && *value <= '9')
        {
            if (trailing_whitespace)
                return false;

            saw_digit = true;
            const size_t digit = (size_t)(*value - '0');
            if (!exceeds_size_limit)
            {
                if (parsed_length > (MAX_UPLOAD_CONTENT_LENGTH - digit) / 10)
                    exceeds_size_limit = true;
                else
                    parsed_length = (parsed_length * 10) + digit;
            }
        }
        else if (*value == ' ' || *value == '\t')
        {
            if (saw_digit)
                trailing_whitespace = true;
        }
        else
        {
            return false;
        }
    }

    if (!saw_digit)
        return false;

    if (exceeds_size_limit)
    {
        *content_length = MAX_UPLOAD_CONTENT_LENGTH + 1;
        return true;
    }

    // In addition to syntactic validation, require the parsed header to agree
    // with the HTTP server's framing value before trusting it for the upload.
    if (parsed_length != req->content_len)
        return false;

    *content_length = parsed_length;
    return true;
}

static bool get_query_str(httpd_req_t* req, const char* key, char* out_val, size_t val_len)
{
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len <= 1)
        return false;

    if (len > CONFIG_HTTPD_MAX_URI_LEN + 1)
        return false;

    char buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    httpd_req_get_url_query_str(req, buf, len);
    esp_err_t err = httpd_query_key_value(buf, key, out_val, val_len);

    return (err == ESP_OK);
}

static esp_err_t get_query_int(httpd_req_t* req, const char* key, int* value)
{
    char val[32];
    if (get_query_str(req, key, val, sizeof(val)))
    {
        char* end = nullptr;
        errno     = 0;
        long parsed = std::strtol(val, &end, 10);
        if (end == val || *end != '\0' || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
            return ESP_ERR_INVALID_ARG;
        *value = static_cast<int>(parsed);
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

static int hex_nibble(char value)
{
    if (value >= '0' && value <= '9')
        return value - '0';
    if (value >= 'A' && value <= 'F')
        return value - 'A' + 10;
    if (value >= 'a' && value <= 'f')
        return value - 'a' + 10;
    return -1;
}

static bool percent_decode_dtc_codes(const char* encoded, char* decoded, size_t decoded_size)
{
    if (encoded == nullptr || decoded == nullptr || decoded_size == 0)
        return false;

    char*       write = decoded;
    const char* read  = encoded;
    char* const end   = decoded + decoded_size - 1;
    while (*read != '\0')
    {
        unsigned char value;
        if (*read == '%')
        {
            if (read[1] == '\0' || read[2] == '\0')
                return false;

            const int high_nibble = hex_nibble(read[1]);
            const int low_nibble  = hex_nibble(read[2]);
            if (high_nibble < 0 || low_nibble < 0)
                return false;

            value = (unsigned char)((high_nibble << 4) | low_nibble);
            read += 3;
        }
        else
        {
            value = (unsigned char)*read++;
        }

        if (value == '\0' || write == end)
            return false;
        *write++ = (char)value;
    }

    *write = '\0';
    return true;
}

static esp_err_t get_dtc_code_list(httpd_req_t* req, char* scratch_buf, size_t scratch_len,
                                   const char* out_ptrs[], size_t max_ptrs, size_t* out_count)
{
    *out_count = 0;

    char encoded_codes[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (!get_query_str(req, "codes", encoded_codes, sizeof(encoded_codes)))
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (!percent_decode_dtc_codes(encoded_codes, scratch_buf, scratch_len))
        return ESP_ERR_INVALID_ARG;

    char* token = scratch_buf;
    while (true)
    {
        if (*token == '\0')
            return ESP_ERR_INVALID_ARG;
        if (*out_count == max_ptrs)
            return ESP_ERR_INVALID_SIZE;

        out_ptrs[*out_count] = token;
        (*out_count)++;

        char* separator = strchr(token, ',');
        if (separator == nullptr)
            return ESP_OK;

        *separator = '\0';
        token      = separator + 1;
    }
}

static bool is_valid_dtc_code(const char* code)
{
    if (strlen(code) != 5)
        return false;

    const char system = code[0];
    if (system != 'P' && system != 'B' && system != 'C' && system != 'U')
        return false;
    if (code[1] < '0' || code[1] > '3')
        return false;

    for (size_t i = 2; i < 5; ++i)
    {
        if (!((code[i] >= '0' && code[i] <= '9') || (code[i] >= 'A' && code[i] <= 'F')))
            return false;
    }

    return true;
}

static esp_err_t get_path_pid(httpd_req_t* req, const char* route, int* value)
{
    const size_t route_len = strlen(route);
    if (strncmp(req->uri, route, route_len) != 0)
        return ESP_ERR_NOT_FOUND;

    const char* suffix = req->uri + route_len;
    if (*suffix == '\0' || *suffix == '?')
        return ESP_ERR_INVALID_ARG;

    char* end = nullptr;
    errno     = 0;
    long parsed = std::strtol(suffix, &end, 10);
    if (end == suffix || (*end != '\0' && *end != '?') || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
        return ESP_ERR_INVALID_ARG;

    *value = static_cast<int>(parsed);
    return ESP_OK;
}

static esp_err_t get_query_bool(httpd_req_t* req, const char* key, bool* value)
{
    char val[16];
    if (get_query_str(req, key, val, sizeof(val)))
    {
        *value = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t get_download_query(httpd_req_t* req, bool* download)
{
    if (req == nullptr || download == nullptr)
        return ESP_ERR_INVALID_ARG;

    const size_t query_length = httpd_req_get_url_query_len(req);
    const char*  query_start  = strchr(req->uri, '?');
    if (query_start == nullptr)
        return ESP_ERR_NOT_FOUND;
    if (query_length == 0 || query_length > CONFIG_HTTPD_MAX_URI_LEN)
        return ESP_ERR_INVALID_ARG;

    char query[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
        return ESP_ERR_INVALID_ARG;

    // This endpoint accepts no query or one literal download=value component.
    // Do not use httpd_query_key_value here: it accepts unrelated parameters,
    // duplicate keys, and decoded aliases that make the request ambiguous.
    if (strchr(query, '&') != nullptr)
        return ESP_ERR_INVALID_ARG;

    static constexpr char download_key[] = "download=";
    constexpr size_t      download_key_length = sizeof(download_key) - 1;
    if (strncmp(query, download_key, download_key_length) != 0)
        return ESP_ERR_INVALID_ARG;

    const char* value = query + download_key_length;
    if (strcmp(value, "true") == 0 || strcmp(value, "1") == 0)
    {
        *download = true;
        return ESP_OK;
    }
    if (strcmp(value, "false") == 0 || strcmp(value, "0") == 0)
    {
        *download = false;
        return ESP_OK;
    }

    return ESP_ERR_INVALID_ARG;
}

cJSON* get_validated_json_payload(httpd_req_t* req, size_t max_size)
{
    size_t total_len = req->content_len;

    if (total_len <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length required");
        return nullptr;
    }

    if (total_len > max_size)
    {
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "JSON too large");
        return nullptr;
    }

    char* buf = (char*)malloc(total_len + 1);
    if (buf == nullptr)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server OOM");
        return nullptr;
    }

    size_t received_total = 0;
    const int64_t deadline = esp_timer_get_time() + JSON_PAYLOAD_RECEIVE_DEADLINE_US;
    while (received_total < total_len)
    {
        if (esp_timer_get_time() >= deadline)
        {
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request body");
            return nullptr;
        }

        int ret = httpd_req_recv(req, buf + received_total, total_len - received_total);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            if (esp_timer_get_time() < deadline)
                continue;

            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request body");
            return nullptr;
        }
        if (ret <= 0 || (size_t)ret > (total_len - received_total))
        {
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request body");
            return nullptr;
        }
        received_total += (size_t)ret;
    }
    buf[total_len] = '\0';

    const char* parse_end = nullptr;
    cJSON*      root      = cJSON_ParseWithLengthOpts(buf, total_len, &parse_end, 0);
    if (root != nullptr && parse_end != nullptr && parse_end >= buf && parse_end <= (buf + total_len))
    {
        while (parse_end < (buf + total_len) && std::isspace((unsigned char)*parse_end))
            ++parse_end;

        if (parse_end != (buf + total_len))
        {
            cJSON_Delete(root);
            root = nullptr;
        }
    }
    else if (root != nullptr)
    {
        cJSON_Delete(root);
        root = nullptr;
    }
    free(buf);

    if (root == nullptr)
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");

    return root;
}

static cJSON* get_full_json_payload(httpd_req_t* req, size_t max_size)
{
    if (req->content_len <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length required");
        return nullptr;
    }

    const size_t totalLength = (size_t)req->content_len;
    if (totalLength > max_size)
    {
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "JSON too large");
        return nullptr;
    }

    char* buffer = (char*)malloc(totalLength + 1);
    if (buffer == nullptr)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server OOM");
        return nullptr;
    }

    size_t receivedTotal = 0;
    while (receivedTotal < totalLength)
    {
        int received = httpd_req_recv(req, buffer + receivedTotal, totalLength - receivedTotal);
        if (received == HTTPD_SOCK_ERR_TIMEOUT)
            continue;
        if (received <= 0)
        {
            free(buffer);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request body");
            return nullptr;
        }
        receivedTotal += (size_t)received;
    }
    buffer[totalLength] = '\0';

    const char* parseEnd = nullptr;
    cJSON* root = cJSON_ParseWithLengthOpts(buffer, totalLength, &parseEnd, 0);
    if (root != nullptr && parseEnd != nullptr)
    {
        while (*parseEnd != '\0' && std::isspace((unsigned char)*parseEnd))
            ++parseEnd;
        if (*parseEnd != '\0')
        {
            cJSON_Delete(root);
            root = nullptr;
        }
    }
    free(buffer);

    if (root == nullptr)
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return root;
}

static char ascii_to_lower(char value)
{
    return (value >= 'A' && value <= 'Z') ? (char)(value + ('a' - 'A')) : value;
}

static bool has_ascii_case_insensitive_extension(const char* basename, const char* extension)
{
    const char* actual_extension = strrchr(basename, '.');
    if (actual_extension == nullptr)
        return false;

    while (*actual_extension != '\0' && *extension != '\0')
    {
        if (ascii_to_lower(*actual_extension) != ascii_to_lower(*extension))
            return false;
        ++actual_extension;
        ++extension;
    }

    return *actual_extension == '\0' && *extension == '\0';
}

static const char* sd_file_content_type(const char* basename, bool* inline_allowed)
{
    *inline_allowed = true;
    if (has_ascii_case_insensitive_extension(basename, ".txt"))
        return "text/plain";
    if (has_ascii_case_insensitive_extension(basename, ".csv"))
        return "text/csv";
    if (has_ascii_case_insensitive_extension(basename, ".json"))
        return "application/json";
    if (has_ascii_case_insensitive_extension(basename, ".png"))
        return "image/png";
    if (has_ascii_case_insensitive_extension(basename, ".jpg") ||
        has_ascii_case_insensitive_extension(basename, ".jpeg"))
        return "image/jpeg";

    *inline_allowed = false;
    return "application/octet-stream";
}

static void make_safe_download_filename(const char* basename, char* output, size_t output_size)
{
    if (output == nullptr || output_size == 0)
        return;

    size_t written = 0;
    if (basename != nullptr)
    {
        for (const unsigned char* current = (const unsigned char*)basename;
             *current != '\0' && written + 1 < output_size; ++current)
        {
            const unsigned char value = *current;
            const bool allowed = (value >= 'A' && value <= 'Z') || (value >= 'a' && value <= 'z') ||
                                 (value >= '0' && value <= '9') || value == '.' || value == '_' || value == '-';
            output[written++] = allowed ? (char)value : '_';
        }
    }

    if (written == 0)
    {
        static constexpr char fallback[] = "download";
        const size_t fallback_length = MIN(sizeof(fallback) - 1, output_size - 1);
        memcpy(output, fallback, fallback_length);
        written = fallback_length;
    }
    output[written] = '\0';
}

static esp_err_t set_sd_file_response_headers(httpd_req_t* req, const char* content_type,
                                              const char* content_disposition)
{
    esp_err_t err = httpd_resp_set_type(req, content_type);
    if (err != ESP_OK)
        return err;

    return httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);
}

// ============================================================================
// 2. ROUTE HANDLERS
// ============================================================================

esp_err_t index_handler(httpd_req_t* req, void* arg)
{
    FILE* f = fopen("/www/index.html.gz", "rb");
    if (f == nullptr)
    {
        ESP_LOGE("Web", "Failed to open /www/index.html.gz");

        return httpd_resp_send_404(req);
    }
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    char   chunk[1024];
    size_t chunksize = 0;

    while ((chunksize = fread(chunk, 1, sizeof(chunk), f)) > 0)
    {
        if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
        {
            fclose(f);
            return ESP_FAIL;
        }
    }

    fclose(f);

    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t g_system_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_system_get());
}

esp_err_t p_system_reboot_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_system_reboot());
}

esp_err_t p_system_copy_file_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 2048);
    if (root == nullptr)
        return ESP_OK;

    MiddlewareJsonResult resp = m_system_copy_file(root);
    cJSON_Delete(root);
    return send_json_response(req, resp);
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
    return send_json_response(req, m_vin_request());
}

esp_err_t g_dtc_index_handler(httpd_req_t* req, void* arg)
{
    int mode = -1;
    if (httpd_req_get_url_query_len(req) == 0)
    {
        return send_json_response(req, m_dtc_get(mode));
    }

    char mode_text[32];
    if (get_query_str(req, "mode", mode_text, sizeof(mode_text)))
    {
        if (get_query_int(req, "mode", &mode) != ESP_OK)
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid DTC mode");
        MiddlewareJsonResult data = m_dtc_get(mode);
        return send_json_response(req, data);
    }

    char        scratch[(MAX_DTC_CODES_QUERY * 6) + 2];
    const char* codes[MAX_DTC_CODES_QUERY];
    size_t      count = 0;

    if (get_dtc_code_list(req, scratch, sizeof(scratch), codes, MAX_DTC_CODES_QUERY, &count) == ESP_OK)
    {
        for (size_t i = 0; i < count; ++i)
        {
            if (!is_valid_dtc_code(codes[i]))
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid DTC code");
        }
        MiddlewareJsonResult data = m_dtc_description_get(codes, count);
        return send_json_response(req, data);
    }

    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid DTC query");
}

esp_err_t p_dtc_index_handler(httpd_req_t* req, void* arg)
{
    int mode = -1;
    if (get_query_int(req, "mode", &mode) != ESP_OK && (httpd_req_get_url_query_len(req) > 0))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid DTC mode");
    }
    return send_json_response(req, m_dtc_request(mode));
}

esp_err_t p_clear_dtc_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_clear_dtc_request());
}

esp_err_t g_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;
    esp_err_t pid_path = get_path_pid(req, "/api/v1/pid_def/", &target_pid);
    if (pid_path == ESP_ERR_INVALID_ARG)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
    if (pid_path == ESP_OK)
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
    cJSON* root = get_validated_json_payload(req, 2048);
    if (root == nullptr)
        return ESP_OK;  // response already sent

    MiddlewareJsonResult resp = m_pid_def_post(root);
    cJSON_Delete(root);
    return send_json_response(req, resp);
}

esp_err_t put_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_full_json_payload(req, 64 * 1024);
    if (root == nullptr)
        return ESP_OK;

    int status = 200;
    cJSON* resp = m_pid_def_put(root, &status);
    cJSON_Delete(root);

    if (resp == nullptr && status == 204)
    {
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, nullptr, 0);
    }

    return send_json_response(req, {resp, status});
}

esp_err_t p_pid_def_save_index_handler(httpd_req_t* req, void* arg)
{
    if (req->content_len > 0)
    {
        cJSON* root = get_validated_json_payload(req, 256);
        if (root == nullptr)
            return ESP_OK;

        MiddlewareJsonResult resp = m_pid_def_save(root);
        cJSON_Delete(root);
        return send_json_response(req, resp);
    }
    else
    {
        MiddlewareJsonResult resp = m_pid_def_save(nullptr);
        return send_json_response(req, resp);
    }
}

esp_err_t p_pid_def_load_index_handler(httpd_req_t* req, void* arg)
{
    if (req->content_len > 0)
    {
        cJSON* root = get_validated_json_payload(req, 256);
        if (root == nullptr)
            return ESP_OK;

        MiddlewareJsonResult resp = m_pid_def_load(root);
        cJSON_Delete(root);
        return send_json_response(req, resp);
    }
    else
    {
        MiddlewareJsonResult resp = m_pid_def_load(nullptr);
        return send_json_response(req, resp);
    }
}

esp_err_t d_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;
    esp_err_t pid_path = get_path_pid(req, "/api/v1/pid_def/", &target_pid);
    if (pid_path == ESP_ERR_INVALID_ARG)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
    if (pid_path == ESP_OK)
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
    esp_err_t pid_path = get_path_pid(req, "/api/v1/pid_data/", &target_pid);
    if (pid_path == ESP_ERR_INVALID_ARG)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid PID requested");
    if (pid_path == ESP_OK)
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
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid query parameter");
    }

    m_pid_poll_set_running(running);
    httpd_resp_set_status(req, "201 Created");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t p_static_pid_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, {m_static_pid_request(), 200});
}

esp_err_t g_settings_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_settings_get());
}

esp_err_t p_settings_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 2048);
    if (root == nullptr)
        return ESP_OK;

    MiddlewareJsonResult resp = m_settings_set(root);
    cJSON_Delete(root);
    return send_json_response(req, resp);
}

esp_err_t g_sd_card_info_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_sdcard_info_get());
}

esp_err_t p_sd_card_format_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_sdcard_format_post());
}

esp_err_t g_sd_card_file_tree_handler(httpd_req_t* req, void* arg)
{
    const char* api_route = "/api/v1/sd_card/tree";
    char        path_buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (get_sd_rest_path(req, api_route, path_buf, sizeof(path_buf), true) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid SD card path");

    return send_json_response(req, m_sdcard_file_tree_get(path_buf));
}

esp_err_t g_sd_card_file_read_handler(httpd_req_t* req, void* arg)
{
    const char* api_route = "/api/v1/sd_card/file";

    bool download = false;
    if (httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff") != ESP_OK ||
        httpd_resp_set_hdr(req, "Cache-Control", "no-store") != ESP_OK)
        return ESP_FAIL;

    const esp_err_t download_query = get_download_query(req, &download);
    if (download_query != ESP_OK && download_query != ESP_ERR_NOT_FOUND)
        return send_json_error_response(req, 400);

    char path_buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (get_sd_rest_path(req, api_route, path_buf, sizeof(path_buf), false) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid SD card path");

    // Keep this bounded lease through stat, open, every read/send, and close.
    // SDCard's lower-level calls take the same recursive mutex, so another SD
    // request cannot unmount or mutate the file while its response is active.
    SDCard::Operation operation = SDCard::getInstance().acquire_operation(SD_FILE_READ_OPERATION_TIMEOUT, true);
    if (!operation)
        return send_json_error_response(req, 503);

    struct stat file_stat;
    const esp_err_t stat_err = SDCard::getInstance().get_file_stat(path_buf, &file_stat);
    if (stat_err != ESP_OK)
    {
        if (stat_err == ESP_ERR_INVALID_STATE)
            return send_json_error_response(req, 503);
        if (stat_err == ESP_ERR_NOT_FOUND)
            return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return send_json_error_response(req, 500);
    }
    if (!S_ISREG(file_stat.st_mode))
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");

    FILE*           fd       = nullptr;
    const esp_err_t open_err = SDCard::getInstance().open_file(path_buf, "rb", fd);
    if (open_err != ESP_OK || fd == nullptr)
    {
        if (fd != nullptr)
            SDCard::getInstance().close_file(fd);
        if (open_err == ESP_ERR_INVALID_STATE)
            return send_json_error_response(req, 503);
        if (open_err == ESP_ERR_NOT_FOUND)
            return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return send_json_error_response(req, 500);
    }

    const char* basename = strrchr(path_buf, '/');
    basename             = (basename != nullptr) ? basename + 1 : path_buf;
    bool        inline_allowed;
    const char* content_type = sd_file_content_type(basename, &inline_allowed);
    const bool  attachment   = download || !inline_allowed;

    // httpd_resp_set_hdr() retains this pointer until the response is sent.
    // Keep the bounded, sanitized value in this handler frame through every
    // streamed response send, including the final zero-length chunk.
    static constexpr size_t SAFE_FILENAME_SIZE = 96;
    static constexpr char   disposition_prefix[] = "attachment; filename=\"";
    char                    safe_filename[SAFE_FILENAME_SIZE];
    char                    disposition[sizeof(disposition_prefix) + SAFE_FILENAME_SIZE + 1];
    const char*             content_disposition = "inline";
    if (attachment)
    {
        make_safe_download_filename(basename, safe_filename, sizeof(safe_filename));
        const size_t filename_length = strlen(safe_filename);
        memcpy(disposition, disposition_prefix, sizeof(disposition_prefix) - 1);
        memcpy(disposition + sizeof(disposition_prefix) - 1, safe_filename, filename_length);
        disposition[sizeof(disposition_prefix) - 1 + filename_length]     = '\"';
        disposition[sizeof(disposition_prefix) - 1 + filename_length + 1] = '\0';
        content_disposition = disposition;
    }

    bool response_headers_set = false;
    bool bytes_sent           = false;
    char chunk[1024];
    while (true)
    {
        const size_t read_bytes = SDCard::getInstance().file_read_chunk(fd, chunk, sizeof(chunk));
        if (ferror(fd) != 0)
        {
            const esp_err_t close_err = SDCard::getInstance().close_file(fd);
            fd                        = nullptr;
            if (close_err != ESP_OK)
                ESP_LOGE(TAG, "Failed to close SD file after read error: %s", esp_err_to_name(close_err));
            return bytes_sent ? ESP_FAIL : send_json_error_response(req, 500);
        }
        if (read_bytes == 0)
            break;

        if (!response_headers_set)
        {
            if (set_sd_file_response_headers(req, content_type, content_disposition) != ESP_OK)
            {
                SDCard::getInstance().close_file(fd);
                return send_json_error_response(req, 500);
            }
            response_headers_set = true;
        }

        if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK)
        {
            SDCard::getInstance().close_file(fd);
            return ESP_FAIL;
        }
        bytes_sent = true;
    }

    // Empty regular files still need their successful stream response headers.
    if (!response_headers_set && set_sd_file_response_headers(req, content_type, content_disposition) != ESP_OK)
    {
        SDCard::getInstance().close_file(fd);
        return send_json_error_response(req, 500);
    }

    const esp_err_t send_err  = httpd_resp_send_chunk(req, nullptr, 0);
    const esp_err_t close_err = SDCard::getInstance().close_file(fd);
    fd                        = nullptr;
    if (close_err != ESP_OK)
        ESP_LOGE(TAG, "Failed to close SD file: %s", esp_err_to_name(close_err));

    // The response is committed once its stream terminator is attempted. Do
    // not try to append a JSON error after any response bytes were sent.
    return (send_err == ESP_OK && close_err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t p_file_upload_handler(httpd_req_t* req, void* arg)
{
    size_t content_length = 0;
    if (!get_validated_upload_content_length(req, &content_length))
        return send_json_response(req, make_upload_result("error", "Invalid Content-Length", 400));

    const char* api_route = "/api/v1/sd_card/file";
    char        path_buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (get_sd_rest_path(req, api_route, path_buf, sizeof(path_buf), false) != ESP_OK)
        return send_json_response(req, make_upload_result("error", "Invalid SD card path", 400));

    const char* relative_path = path_buf;
    size_t      path_len      = strlen(relative_path);

    if (content_length > MAX_UPLOAD_CONTENT_LENGTH)
        return send_json_response(req, make_upload_result("error", "Content too large", 413));

    // If the path ends with '/', treat it as a directory creation request
    if (path_len > 0 && relative_path[path_len - 1] == '/')
    {
        if (content_length != 0)
            return send_json_response(req, make_upload_result("error", "Directory upload body must be empty", 400));

        int         response_status = 200;
        const char* response_reason = nullptr;
        {
            SDCard::Operation operation = SDCard::getInstance().acquire_operation(UPLOAD_OPERATION_TIMEOUT, true);
            if (!operation)
            {
                response_status = 503;
                response_reason = operation.status() == ESP_ERR_TIMEOUT ? "SD card busy" : "SD Card not mounted";
            }
            else if (SDCard::getInstance().create_directory(relative_path) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to create directory: %s", relative_path);
                response_status = 500;
                response_reason = "Failed to create directory";
            }
        }

        if (response_status != 200)
        {
            return send_json_response(req, make_upload_result("error", response_reason, response_status));
        }
        return send_json_response(req, make_upload_result("success", nullptr, 200));
    }

    const int64_t upload_started_at = esp_timer_get_time();
    int           response_status   = 200;
    const char*   response_reason   = nullptr;
    bool          send_response     = true;

    {
        // Keep this lease for the entire transaction. The file APIs take the
        // same recursive mutex internally, so no other SD request can observe
        // a partially written newly-created target.
        SDCard::Operation operation = SDCard::getInstance().acquire_operation(UPLOAD_OPERATION_TIMEOUT, true);
        if (!operation)
        {
            response_status = 503;
            response_reason = operation.status() == ESP_ERR_TIMEOUT ? "SD card busy" : "SD Card not mounted";
        }
        else
        {
            struct stat file_stat;
            const esp_err_t stat_err = SDCard::getInstance().get_file_stat(relative_path, &file_stat);
            const bool      target_existed = stat_err == ESP_OK;
            FILE*           fd             = nullptr;

            if (stat_err != ESP_OK && stat_err != ESP_ERR_NOT_FOUND)
            {
                response_status = 500;
                response_reason = "Failed to inspect upload target";
            }
            else
            {
                // Deliberately retain the selected legacy overwrite behavior:
                // an existing target is truncated in place, not staged.
                const esp_err_t open_err = SDCard::getInstance().open_file(relative_path, "w", fd);
                if (open_err != ESP_OK || fd == nullptr)
                {
                    ESP_LOGE(TAG, "Failed to open file for writing: %s", relative_path);
                    if (fd != nullptr)
                    {
                        const esp_err_t close_err = SDCard::getInstance().close_file(fd);
                        if (close_err != ESP_OK)
                            ESP_LOGE(TAG, "Failed to close upload target: %s", esp_err_to_name(close_err));
                    }
                    if (!target_existed)
                    {
                        const esp_err_t delete_err = SDCard::getInstance().delete_file(relative_path);
                        if (delete_err != ESP_OK && delete_err != ESP_ERR_NOT_FOUND)
                            ESP_LOGE(TAG, "Failed to remove failed upload %s: %s", relative_path,
                                     esp_err_to_name(delete_err));
                    }
                    response_status = open_err == ESP_ERR_INVALID_STATE ? 503 : 500;
                    response_reason = "Storage error or file already exists";
                }
                else
                {
                    bool   upload_failed = false;
                    size_t remaining     = content_length;
                    char   chunk[1024];
                    int64_t no_progress_deadline = esp_timer_get_time() + UPLOAD_NO_PROGRESS_DEADLINE_US;
                    const int64_t absolute_deadline = upload_started_at + UPLOAD_ABSOLUTE_DEADLINE_US;

                    if (esp_timer_get_time() >= absolute_deadline)
                    {
                        upload_failed  = true;
                        response_status = 408;
                        response_reason = "Upload timed out";
                    }

                    while (!upload_failed && remaining > 0)
                    {
                        const int64_t now = esp_timer_get_time();
                        if (now >= no_progress_deadline || now >= absolute_deadline)
                        {
                            upload_failed  = true;
                            response_status = 408;
                            response_reason = "Upload timed out";
                            break;
                        }

                        const int received = httpd_req_recv(req, chunk, MIN(remaining, sizeof(chunk)));
                        if (received == HTTPD_SOCK_ERR_TIMEOUT)
                            continue;

                        if (received <= 0 || (size_t)received > remaining)
                        {
                            // The peer has disconnected. Cleanup is still
                            // required, but attempting another response is not.
                            upload_failed = true;
                            send_response = false;
                            break;
                        }

                        if (SDCard::getInstance().file_write_chunk(fd, chunk, (size_t)received) != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Disk write failed");
                            upload_failed  = true;
                            response_status = 500;
                            response_reason = "Disk write failed";
                            break;
                        }

                        remaining -= (size_t)received;
                        no_progress_deadline = esp_timer_get_time() + UPLOAD_NO_PROGRESS_DEADLINE_US;
                    }

                    if (!upload_failed && esp_timer_get_time() >= absolute_deadline)
                    {
                        upload_failed  = true;
                        response_status = 408;
                        response_reason = "Upload timed out";
                    }

                    if (!upload_failed && (fflush(fd) != 0 || ferror(fd) != 0))
                    {
                        ESP_LOGE(TAG, "Failed to flush upload target");
                        upload_failed  = true;
                        response_status = 500;
                        response_reason = "Disk write failed";
                    }

                    // close_file is called exactly once on every successful
                    // open path. ferror() covers buffered write failures and
                    // close_file() reports any close failure exposed by the
                    // SD-card implementation.
                    const esp_err_t close_err = SDCard::getInstance().close_file(fd);
                    fd                        = nullptr;
                    if (close_err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to close upload target: %s", esp_err_to_name(close_err));
                        upload_failed  = true;
                        response_status = 500;
                        response_reason = "Failed to close upload target";
                    }

                    if (upload_failed && !target_existed)
                    {
                        const esp_err_t delete_err = SDCard::getInstance().delete_file(relative_path);
                        if (delete_err != ESP_OK && delete_err != ESP_ERR_NOT_FOUND)
                            ESP_LOGE(TAG, "Failed to remove partial upload %s: %s", relative_path,
                                     esp_err_to_name(delete_err));
                    }
                }
            }
        }
    }

    if (!send_response)
        return ESP_FAIL;

    if (response_status != 200)
        return send_json_response(req, make_upload_result("error", response_reason, response_status));
    return send_json_response(req, make_upload_result("success", nullptr, 200));
}

esp_err_t d_file_delete_handler(httpd_req_t* req, void* arg)
{
    const char* api_route = "/api/v1/sd_card/file";
    char        path_buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (get_sd_rest_path(req, api_route, path_buf, sizeof(path_buf), false) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid SD card path");

    return send_json_response(req, m_sdcard_file_delete_delete(path_buf));
}

// ============================================================================
// 3. WEBSOCKETS & LIVE STREAMING
// ============================================================================

void ws_send_byte(uint8_t byte)
{
    static uint8_t packet[1];
    packet[0] = byte;

    httpd_ws_frame_t ws_frame = {
        .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_BINARY, .payload = packet, .len = 1};

    AsyncWebServer::getInstance().wsBroadcast(&ws_frame);
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
        return ret;

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
    {
        ESP_LOGI(TAG, "WS Client #%d Closed", sockfd);
        if (AsyncWebServer::getInstance().getActiveWSClientCount() <= 1)
        {
            b_pid_stream_enabled.store(false);
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
                        b_pid_stream_enabled.store(true);
                        break;
                    case WS_STOP_PID_STREAM:
                        b_pid_stream_enabled.store(false);
                        break;
                    case WS_PING_REQUEST_STREAM:
                        ESP_LOGD(TAG, "WS Ping received");
                        ws_send_byte(WS_PING_RESPONSE_STREAM);
                        break;
                    default:
                        ESP_LOGW(TAG, "WS Unknown command: 0x%02X", buf[0]);
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
    if (!b_pid_stream_enabled.load())
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
                               {"/api/v1/pid_def", HTTP_PUT, put_pid_def_index_handler},
                               {"/api/v1/pid_def/save", HTTP_POST, p_pid_def_save_index_handler},
                               {"/api/v1/pid_def/load", HTTP_POST, p_pid_def_load_index_handler},

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
                               {"/api/v1/system/copy_file", HTTP_POST, p_system_copy_file_index_handler},

                               {"/api/v1/sd_card/info", HTTP_GET, g_sd_card_info_handler},
                               {"/api/v1/sd_card/format", HTTP_POST, p_sd_card_format_handler},
                               {"/api/v1/sd_card/tree", HTTP_GET, g_sd_card_file_tree_handler},
                               {"/api/v1/sd_card/tree/*", HTTP_GET, g_sd_card_file_tree_handler},
                               {"/api/v1/sd_card/file/*", HTTP_POST, p_file_upload_handler},
                               {"/api/v1/sd_card/file/*", HTTP_DELETE, d_file_delete_handler},
                               {"/api/v1/sd_card/file/*", HTTP_GET, g_sd_card_file_read_handler},

                               {"/api/v1/settings", HTTP_GET, g_settings_index_handler},
                               {"/api/v1/settings", HTTP_POST, p_settings_index_handler}};

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
    AsyncWebServer::Config server_config;  // TODO: Load from nvs storage
    server_config.async_worker_task_num         = 6;
    server_config.max_open_sockets              = 7;
    server_config.max_requests_per_sec          = 50;
    server_config.async_worker_task_priority    = 5;
    server_config.async_worker_stack_size       = 8192;
    server_config.httpd_config.uri_match_fn     = httpd_uri_match_wildcard;
    server_config.httpd_config.max_uri_handlers = 48;
    // Keep receive waits below the upload no-progress deadline so an upload
    // worker can enforce it rather than remaining blocked in httpd_req_recv.
    server_config.httpd_config.recv_wait_timeout = 1;

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

void stop_web_server()
{
    if (xWSDataStreamTaskHandle)
    {
        vTaskDelete(xWSDataStreamTaskHandle);
        xWSDataStreamTaskHandle = nullptr;
    }
    if (WSDataStreamSemaphore)
    {
        vSemaphoreDelete(WSDataStreamSemaphore);
        WSDataStreamSemaphore = nullptr;
    }
    AsyncWebServer::getInstance().stop();
}

// TODO add nrc endpoint and maybe callback with ws frame
