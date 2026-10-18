#include "webserver.hpp"

#include <sys/param.h>
#include <sys/stat.h>

#include <atomic>
#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "api_routes.hpp"
#include "auth.hpp"
#include "auth_http.hpp"
#include "async_web_server.hpp"
#include "body_reader.hpp"
#include "button.h"
#include "cJSON.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host_guard.hpp"
#include "http_error.hpp"
#include "http_parse.hpp"
#include "http_parser.h"
#include "input_limits.hpp"
#include "json_budget.hpp"
#include "middleware.hpp"
#include "obd2_patch.hpp"
#include "obd2.hpp"
#include "sd_card.hpp"
#include "wifi.hpp"
#include "wifi_scan_http.hpp"

struct RouteDef
{
    const char*                  uri;
    httpd_method_t               method;
    AsyncWebServer::AsyncHandler handler;
    bool                         gated;  // needs a valid Bearer session (P2.10c)
};

#define WS_DAT_STREAM_TASK_STACK_SIZE 4096
#define WS_DAT_STREAM_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define WS_DAT_STREAM_TASK_CORE_ID 1
#define WS_DAT_STREAM_PERIOD 1000
#define MAX_DTC_CODES_QUERY 30

static const char* TAG = "WEB_SERVER";

static constexpr int64_t JSON_PAYLOAD_RECEIVE_DEADLINE_US = 5LL * 1000 * 1000;
// PUT /pid_def takes up to 64 KiB, so it gets more time; it is still absolute (ASYNC-1).
static constexpr int64_t PID_DEF_PUT_RECEIVE_DEADLINE_US = 20LL * 1000 * 1000;
static constexpr size_t  MAX_UPLOAD_CONTENT_LENGTH        = input_limits::UPLOAD_MAX;
static constexpr int64_t UPLOAD_NO_PROGRESS_DEADLINE_US   = 5LL * 1000 * 1000;
static constexpr int64_t UPLOAD_ABSOLUTE_DEADLINE_US      = 60LL * 1000 * 1000;
// The download keeps the SD lease until the last byte is sent (SD-2), so it needs an end as well.
static constexpr int64_t DOWNLOAD_ABSOLUTE_DEADLINE_US    = 5LL * 60 * 1000 * 1000;
static constexpr TickType_t UPLOAD_OPERATION_TIMEOUT       = pdMS_TO_TICKS(250);
static constexpr TickType_t SD_FILE_READ_OPERATION_TIMEOUT = pdMS_TO_TICKS(250);

static TaskHandle_t      xWSDataStreamTaskHandle = nullptr;
static SemaphoreHandle_t WSDataStreamSemaphore   = nullptr;
static std::atomic<bool> wsStreamingEnabled{false};
static std::atomic<bool> wsTaskStopRequested{false};  // WSDataStreamTask leaves its loop on its own
static std::atomic<bool> wsTaskRunning{false};
static std::atomic<bool> b_pid_stream_enabled{false};

namespace
{

// ============================================================================
// 1. HTTP & PARSING UTILITIES
// ============================================================================

static esp_err_t to_esp_err(http_parse::Result r)
{
    switch (r)
    {
        case http_parse::Result::Ok: return ESP_OK;
        case http_parse::Result::NoMem: return ESP_ERR_NO_MEM;
        case http_parse::Result::InvalidSize: return ESP_ERR_INVALID_SIZE;
        default: return ESP_ERR_INVALID_ARG;
    }
}

// Extracts, percent-decodes and validates an SD-card route suffix (http_parse::sdRestPath).
static esp_err_t get_sd_rest_path(httpd_req_t* req, const char* api_route, char* out, size_t out_size,
                                  bool allow_root)
{
    if (req == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return to_esp_err(http_parse::sdRestPath(req->uri, api_route, out, out_size, allow_root));
}

// Every error leaves through AsyncWebServer::sendError (http_error.hpp): one JSON shape, one Content-Type.
static esp_err_t send_json_error_response(httpd_req_t* req, int status)
{
    return AsyncWebServer::sendError(req, status);
}

// ============================================================================
// Host/Origin guard and Content-Type policy (P2.10b). Runs in the httpd task, before the rate limit and the queue.
// ============================================================================

// Reads one request header into buf. present=false when it is absent or empty. False when it does not fit (hostile).
static bool read_header(httpd_req_t* req, const char* name, char* buf, size_t cap, bool* present)
{
    buf[0]        = '\0';
    *present      = false;
    const size_t len = httpd_req_get_hdr_value_len(req, name);
    if (len == 0)
    {
        return true;
    }
    *present = true;
    return len < cap && httpd_req_get_hdr_value_str(req, name, buf, cap) == ESP_OK;
}

// True when Host (and Origin, if sent) name this device. Otherwise the request was answered with 403.
static bool pass_host_origin(httpd_req_t* req)
{
    char host[host_guard::kHostMax];
    char origin[host_guard::kOriginMax];
    bool hostPresent   = false;
    bool originPresent = false;
    const bool hostFits   = read_header(req, "Host", host, sizeof(host), &hostPresent);
    const bool originFits = read_header(req, "Origin", origin, sizeof(origin), &originPresent);

    const std::string ap  = WIFI::getInstance().getApIP();
    const std::string sta = WIFI::getInstance().getStaIP();
    const host_guard::Ips ips{ap.c_str(), sta.c_str()};

    host_guard::Verdict verdict = host_guard::check(hostPresent && hostFits ? host : nullptr,
                                                    originPresent && originFits ? origin : nullptr, ips);
    if (verdict == host_guard::Verdict::Ok && originPresent && !originFits)
    {
        verdict = host_guard::Verdict::BadOrigin;
    }
    if (verdict == host_guard::Verdict::BadHost)
    {
        (void)AsyncWebServer::sendError(req, 403, "Host not allowed", http_error::kBadHost);
        return false;
    }
    if (verdict == host_guard::Verdict::BadOrigin)
    {
        (void)AsyncWebServer::sendError(req, 403, "Origin not allowed", http_error::kBadOrigin);
        return false;
    }
    return true;
}

// A request that carries a body must say it is JSON, or an octet stream on the upload route.
static bool pass_content_type(httpd_req_t* req, const RouteDef* route)
{
    if ((req->method != HTTP_POST && req->method != HTTP_PUT && req->method != HTTP_PATCH) || req->content_len == 0)
    {
        return true;
    }
    const bool upload = route != nullptr && strcmp(route->uri, "/api/v1/sd_card/file/*") == 0;
    char       ct[host_guard::kContentTypeMax];
    bool       present = false;
    const bool fits    = read_header(req, "Content-Type", ct, sizeof(ct), &present);
    if (present && fits && host_guard::contentTypeOk(ct, upload ? host_guard::Body::Octet : host_guard::Body::Json))
    {
        return true;
    }
    (void)AsyncWebServer::sendError(req, 415, upload ? "Content-Type must be application/octet-stream"
                                                      : "Content-Type must be application/json",
                                    http_error::kBadContentType);
    return false;
}

// Gated routes need a valid Bearer session. Same answer for a missing, malformed, unknown or expired token.
static bool pass_auth(httpd_req_t* req, const RouteDef* route)
{
    if (route == nullptr || !route->gated)
    {
        return true;
    }
    char       header[auth::kAuthHeaderMax];
    char       token[auth::kAuthHeaderMax];
    bool       present = false;
    const bool fits    = read_header(req, "Authorization", header, sizeof(header), &present);
    const bool have    = present && fits && auth::bearerToken(header, token, sizeof(token));
    const auth::Result r = auth_service::requireAdmin(have ? token : nullptr);
    if (r == auth::Result::Ok)
    {
        return true;
    }
    const auth::Http h = auth::httpFor(r);
    (void)AsyncWebServer::sendError(req, h.status, h.reason, h.code);
    return false;
}

// Order of checks (docs/decisions.md): Host/Origin guard, Content-Type, auth; the global rate limit and the handler
// come after, in AsyncWebServer.
static bool check_request(httpd_req_t* req, const RouteDef* route)
{
    return pass_host_origin(req) && pass_content_type(req, route) && pass_auth(req, route);
}

// Bodies larger than this are judged in the worker: httpd reads and discards an unread body after the handler returns,
// and that must not happen in the HTTP task for a hostile 4 MiB upload.
static constexpr size_t INLINE_REJECT_MAX_BODY = 1024;

// AsyncWebServer::Config::request_filter, in the httpd task.
static bool request_filter(httpd_req_t* req, void* routeArg)
{
    if (req->content_len > INLINE_REJECT_MAX_BODY)
    {
        return true;
    }
    return check_request(req, static_cast<const RouteDef*>(routeArg));
}

// What every REST route is registered with: re-checks the requests the filter let through unjudged, then the handler.
static esp_err_t route_shim(httpd_req_t* req, void* arg)
{
    const RouteDef* route = static_cast<const RouteDef*>(arg);
    if (req->content_len > INLINE_REJECT_MAX_BODY && !check_request(req, route))
    {
        return ESP_OK;
    }
    return route->handler(req, nullptr);
}

// Error bodies built by the middleware (or make_upload_result) get the stable `code` here, so no call site has to
// remember it. Bodies that are not errors (the 409 confirm_required challenge) are left alone.
static void ensure_error_fields(cJSON* root, int http_status)
{
    const cJSON* status = cJSON_GetObjectItemCaseSensitive(root, "status");
    if (http_status < 400 || !cJSON_IsString(status) || strcmp(status->valuestring, "error") != 0)
    {
        return;
    }
    if (cJSON_GetObjectItemCaseSensitive(root, "reason") == nullptr)
    {
        cJSON_AddStringToObject(root, "reason", http_error::defaultReason(http_status));
    }
    if (cJSON_GetObjectItemCaseSensitive(root, "code") == nullptr)
    {
        cJSON_AddStringToObject(root, "code", http_error::defaultCode(http_status));
    }
}

static esp_err_t send_json_response(httpd_req_t* req, MiddlewareJsonResult result)
{
    cJSON* root = result.body;
    if (root == nullptr && result.http_status == 204)
    {
        httpd_resp_set_status(req, http_error::statusLine(204));
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, nullptr, 0);
    }
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "Failed to allocate JSON response");
        // A null body can accompany a meaningful middleware status. Preserve
        // mapped non-2xx statuses, but treat a successful/null result as OOM.
        return send_json_error_response(req, http_error::errorStatus(result.http_status));
    }

    ensure_error_fields(root, result.http_status);
    const char* json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to print JSON");
        return send_json_error_response(req, 500);
    }

    if (result.http_status != 200)
    {
        const char* status_text = http_error::statusLine(result.http_status);
        httpd_resp_set_status(req, status_text != nullptr ? status_text : http_error::statusLine(500));
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char retry_after[12];
    if (result.http_status >= 400 && http_error::retryAfterS(result.http_status) != 0)
    {
        (void)snprintf(retry_after, sizeof(retry_after), "%u",
                 result.retry_after_s ? result.retry_after_s : static_cast<unsigned>(http_error::retryAfterS(result.http_status)));
        httpd_resp_set_hdr(req, "Retry-After", retry_after);  // lives until the send below
    }
    esp_err_t ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    cJSON_free((void*)json_str);
    cJSON_Delete(root);
    return ret;
}

static MiddlewareJsonResult make_upload_result(const char* status, const char* reason, int http_status)
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return {nullptr, 500};
    }
    cJSON_AddStringToObject(root, "status", status);
    if (reason != nullptr)
    {
        cJSON_AddStringToObject(root, "reason", reason);
    }
    return {root, http_status};
}

// req->content_len is zero both when Content-Length is absent and when the
// request explicitly declares an empty body. SD uploads need to distinguish
// those cases before acquiring an SD operation or changing a target path.
static bool get_validated_upload_content_length(httpd_req_t* req, size_t* content_length)
{
    if (req == nullptr || content_length == nullptr)
    {
        return false;
    }

    const size_t header_length = httpd_req_get_hdr_value_len(req, "Content-Length");
    if (header_length == 0 || header_length >= CONFIG_HTTPD_MAX_REQ_HDR_LEN)
    {
        return false;
    }

    char header_value[CONFIG_HTTPD_MAX_REQ_HDR_LEN];
    if (httpd_req_get_hdr_value_str(req, "Content-Length", header_value, sizeof(header_value)) != ESP_OK)
    {
        return false;
    }

    bool   saw_digit          = false;
    bool   trailing_whitespace = false;
    bool   exceeds_size_limit  = false;
    size_t parsed_length       = 0;
    for (const char* value = header_value; *value != '\0'; ++value)
    {
        if (*value >= '0' && *value <= '9')
        {
            if (trailing_whitespace)
            {
                return false;
            }

            saw_digit = true;
            const size_t digit = (size_t)(*value - '0');
            if (!exceeds_size_limit)
            {
                if (parsed_length > (MAX_UPLOAD_CONTENT_LENGTH - digit) / 10)
                {
                    exceeds_size_limit = true;
                }
                else
                {
                    parsed_length = (parsed_length * 10) + digit;
                }
            }
        }
        else if (*value == ' ' || *value == '\t')
        {
            if (saw_digit)
            {
                trailing_whitespace = true;
            }
        }
        else
        {
            return false;
        }
    }

    if (!saw_digit)
    {
        return false;
    }

    if (exceeds_size_limit)
    {
        *content_length = MAX_UPLOAD_CONTENT_LENGTH + 1;
        return true;
    }

    // In addition to syntactic validation, require the parsed header to agree
    // with the HTTP server's framing value before trusting it for the upload.
    if (parsed_length != req->content_len)
    {
        return false;
    }

    *content_length = parsed_length;
    return true;
}

static bool get_query_str(httpd_req_t* req, const char* key, char* out_val, size_t val_len)
{
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len <= 1)
    {
        return false;
    }

    if (len > CONFIG_HTTPD_MAX_URI_LEN + 1)
    {
        return false;
    }

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
        return to_esp_err(http_parse::strictInt(val, value));
    }
    return ESP_ERR_NOT_FOUND;
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
    return to_esp_err(http_parse::dtcCodeList(encoded_codes, scratch_buf, scratch_len, out_ptrs, max_ptrs, out_count));
}

static bool is_valid_dtc_code(const char* code)
{
    return http_parse::isValidDtcCode(code);
}

static esp_err_t get_path_pid(httpd_req_t* req, const char* route, int* value)
{
    const size_t route_len = strlen(route);
    if (strncmp(req->uri, route, route_len) != 0)
    {
        return ESP_ERR_NOT_FOUND;
    }
    return to_esp_err(http_parse::pathInt(req->uri, route, value));
}

// Checks the request's query keys against a whitelist (http_parse::queryKeys). `present` has bit i set when allowed[i]
// occurs. ESP_ERR_INVALID_ARG for an unknown or repeated key.
static esp_err_t check_query_keys(httpd_req_t* req, const char* const* allowed, size_t n, unsigned* present)
{
    return to_esp_err(http_parse::queryKeys(req->uri, allowed, n, present));
}

// Optional `mode` query of the DTC routes: -1 (absent) means all modes. A present value must be a non-negative
// integer; a negative one used to be accepted and meant "all" by accident.
static esp_err_t get_dtc_mode_query(httpd_req_t* req, int* mode)
{
    *mode = -1;
    int       parsed = 0;
    esp_err_t err    = get_query_int(req, "mode", &parsed);
    if (err != ESP_OK || parsed < 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    *mode = parsed;
    return ESP_OK;
}

static esp_err_t get_download_query(httpd_req_t* req, bool* download)
{
    if (req == nullptr || download == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t query_length = httpd_req_get_url_query_len(req);
    const char*  query_start  = strchr(req->uri, '?');
    if (query_start == nullptr)
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (query_length == 0 || query_length > CONFIG_HTTPD_MAX_URI_LEN)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char query[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // This endpoint accepts no query or one literal download=value component.
    // Do not use httpd_query_key_value here: it accepts unrelated parameters,
    // duplicate keys, and decoded aliases that make the request ambiguous.
    if (strchr(query, '&') != nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    static constexpr char download_key[] = "download=";
    constexpr size_t      download_key_length = sizeof(download_key) - 1;
    if (strncmp(query, download_key, download_key_length) != 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

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

cJSON* get_validated_json_payload(httpd_req_t* req, size_t max_size, int64_t deadline_us = JSON_PAYLOAD_RECEIVE_DEADLINE_US)
{
    size_t total_len = req->content_len;

    if (total_len <= 0)
    {
        AsyncWebServer::sendError(req, 400, "Content-Length required");
        return nullptr;
    }

    if (total_len > max_size)
    {
        AsyncWebServer::sendError(req, 413, "JSON too large");
        return nullptr;
    }

    char* buf = (char*)malloc(total_len + 1);
    if (buf == nullptr)
    {
        AsyncWebServer::sendError(req, 500, "Server OOM");
        return nullptr;
    }

    const body_reader::Result received = body_reader::readBody(
        [req](char* dst, size_t len) { return httpd_req_recv(req, dst, len); }, esp_timer_get_time, buf, total_len,
        HTTPD_SOCK_ERR_TIMEOUT, esp_timer_get_time() + deadline_us);
    if (received != body_reader::Result::Ok)
    {
        free(buf);
        AsyncWebServer::sendError(req, 400, "Incomplete request body");
        return nullptr;
    }
    buf[total_len] = '\0';

    // A byte cap does not bound cJSON's node allocations (HEAP-1): refuse before parsing.
    const json_budget::Verdict budget = json_budget::check(buf, total_len, heap_caps_get_free_size(MALLOC_CAP_8BIT));
    if (budget != json_budget::Verdict::Ok)
    {
        free(buf);
        if (budget == json_budget::Verdict::NoMemory)
        {
            AsyncWebServer::sendError(req, 503, json_budget::reason(budget));
        }
        else
        {
            AsyncWebServer::sendError(req, 400, json_budget::reason(budget), http_error::kBadJson);
        }
        return nullptr;
    }

    const char* parse_end = nullptr;
    cJSON*      root      = cJSON_ParseWithLengthOpts(buf, total_len, &parse_end, 0);
    if (root != nullptr && parse_end != nullptr && parse_end >= buf && parse_end <= (buf + total_len))
    {
        while (parse_end < (buf + total_len) && std::isspace((unsigned char)*parse_end))
        {
            ++parse_end;
        }

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
    {
        AsyncWebServer::sendError(req, 400, "Invalid JSON", http_error::kBadJson);
    }

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
    {
        return false;
    }

    while (*actual_extension != '\0' && *extension != '\0')
    {
        if (ascii_to_lower(*actual_extension) != ascii_to_lower(*extension))
        {
            return false;
        }
        ++actual_extension;
        ++extension;
    }

    return *actual_extension == '\0' && *extension == '\0';
}

static const char* sd_file_content_type(const char* basename, bool* inline_allowed)
{
    *inline_allowed = true;
    if (has_ascii_case_insensitive_extension(basename, ".txt"))
    {
        return "text/plain";
    }
    if (has_ascii_case_insensitive_extension(basename, ".csv"))
    {
        return "text/csv";
    }
    if (has_ascii_case_insensitive_extension(basename, ".json"))
    {
        return "application/json";
    }
    if (has_ascii_case_insensitive_extension(basename, ".png"))
    {
        return "image/png";
    }
    if (has_ascii_case_insensitive_extension(basename, ".jpg") ||
        has_ascii_case_insensitive_extension(basename, ".jpeg"))
    {
        return "image/jpeg";
    }

    *inline_allowed = false;
    return "application/octet-stream";
}

static void make_safe_download_filename(const char* basename, char* output, size_t output_size)
{
    if (output == nullptr || output_size == 0)
    {
        return;
    }

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
    {
        return err;
    }

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
            (void)fclose(f);
            return ESP_FAIL;
        }
    }

    (void)fclose(f);

    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t g_system_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_system_get());
}

esp_err_t g_wifi_scan_handler(httpd_req_t* req, void* arg)
{
    if (!wifi_scan_http::accepts(req->uri, req->content_len, httpd_req_get_hdr_value_len(req, "Transfer-Encoding") != 0))
    {
        return AsyncWebServer::sendError(req, 400, "Wi-Fi scan accepts no body or query");
    }
    return send_json_response(req, m_wifi_scan_get());
}

esp_err_t p_wifi_scan_handler(httpd_req_t* req, void* arg)
{
    if (!wifi_scan_http::accepts(req->uri, req->content_len, httpd_req_get_hdr_value_len(req, "Transfer-Encoding") != 0))
    {
        return AsyncWebServer::sendError(req, 400, "Wi-Fi scan accepts no body or query");
    }
    return send_json_response(req, m_wifi_scan_start());
}

esp_err_t p_system_reboot_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_system_reboot());
}

esp_err_t p_sd_card_copy_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 2048);
    if (root == nullptr)
    {
        return ESP_OK;
    }

    MiddlewareJsonResult resp = m_sd_card_copy(root);
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
    static const char* const keys[] = {"mode"};
    unsigned                 present = 0;
    // One query form per call: nothing or ?mode=. Extra or repeated keys are rejected.
    if (check_query_keys(req, keys, 1, &present) != ESP_OK)
    {
        return AsyncWebServer::sendError(req, 400, "Invalid DTC query");
    }
    int mode = -1;
    if (present != 0 && get_dtc_mode_query(req, &mode) != ESP_OK)
    {
        return AsyncWebServer::sendError(req, 400, "Invalid DTC mode");
    }
    return send_json_response(req, m_dtc_get(mode));
}

esp_err_t g_dtc_descriptions_handler(httpd_req_t* req, void* arg)
{
    static const char* const keys[] = {"codes"};
    unsigned                 present = 0;
    if (check_query_keys(req, keys, 1, &present) != ESP_OK || present != 1)
    {
        return AsyncWebServer::sendError(req, 400, "Invalid DTC query");
    }

    char        scratch[(MAX_DTC_CODES_QUERY * 6) + 2];
    const char* codes[MAX_DTC_CODES_QUERY];
    size_t      count = 0;

    if (get_dtc_code_list(req, scratch, sizeof(scratch), codes, MAX_DTC_CODES_QUERY, &count) == ESP_OK)
    {
        for (size_t i = 0; i < count; ++i)
        {
            if (!is_valid_dtc_code(codes[i]))
            {
                return AsyncWebServer::sendError(req, 400, "Invalid DTC code");
            }
        }
        MiddlewareJsonResult data = m_dtc_description_get(codes, count);
        return send_json_response(req, data);
    }

    return AsyncWebServer::sendError(req, 400, "Invalid DTC query");
}

esp_err_t p_dtc_index_handler(httpd_req_t* req, void* arg)
{
    static const char* const keys[] = {"mode"};
    unsigned                 present = 0;
    int                      mode    = -1;
    if (check_query_keys(req, keys, 1, &present) != ESP_OK ||
        (present != 0 && get_dtc_mode_query(req, &mode) != ESP_OK))
    {
        return AsyncWebServer::sendError(req, 400, "Invalid DTC mode");
    }
    return send_json_response(req, m_dtc_request(mode));
}

esp_err_t p_clear_dtc_index_handler(httpd_req_t* req, void* arg)
{
    // Gated by the session check before this runs. A leftover ?token= from the old confirm flow is ignored.
    return send_json_response(req, m_clear_dtc_request());
}

esp_err_t g_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;
    esp_err_t pid_path = get_path_pid(req, "/api/v1/pid_def/", &target_pid);
    if (pid_path == ESP_ERR_INVALID_ARG)
    {
        return AsyncWebServer::sendError(req, 400, "Invalid PID requested");
    }
    if (pid_path == ESP_OK)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            return AsyncWebServer::sendError(req, 400, "Invalid PID requested");
        }
    }
    return send_json_response(req, m_pid_def_get(target_pid));
}

esp_err_t p_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 2048);
    if (root == nullptr)
    {
        return ESP_OK;  // response already sent
    }

    MiddlewareJsonResult resp = m_pid_def_post(root);
    cJSON_Delete(root);
    return send_json_response(req, resp);
}

esp_err_t put_pid_def_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 64 * 1024, PID_DEF_PUT_RECEIVE_DEADLINE_US);
    if (root == nullptr)
    {
        return ESP_OK;
    }

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
        {
            return ESP_OK;
        }

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
        {
            return ESP_OK;
        }

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
    {
        return AsyncWebServer::sendError(req, 400, "Invalid PID requested");
    }
    if (pid_path == ESP_OK)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            return AsyncWebServer::sendError(req, 400, "Invalid PID requested");
        }
    }
    return send_json_response(req, m_pid_def_delete(target_pid));
}

esp_err_t g_pid_data_index_handler(httpd_req_t* req, void* arg)
{
    int target_pid = -1;
    esp_err_t pid_path = get_path_pid(req, "/api/v1/pid_data/", &target_pid);
    if (pid_path == ESP_ERR_INVALID_ARG)
    {
        return AsyncWebServer::sendError(req, 400, "Invalid PID requested");
    }
    if (pid_path == ESP_OK)
    {
        if (target_pid < 0 || target_pid > 0xFFFF)
        {
            return AsyncWebServer::sendError(req, 400, "Invalid PID requested");
        }
    }
    return send_json_response(req, m_pid_data_get(target_pid));
}

esp_err_t patch_obd2_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 256);
    if (root == nullptr)
    {
        return ESP_OK;
    }
    bool       on = false;
    const bool ok = obd2_patch::parseAutoPoll(root, on);
    cJSON_Delete(root);
    if (!ok)
    {
        return AsyncWebServer::sendError(req, 400, "Body must be {\"auto_poll\": true|false}");
    }

    return send_json_response(req, m_auto_poll_set(on));
}

esp_err_t p_static_pid_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, {m_static_pid_request(), 200});
}

// ============================================================================
// Access control routes (P2.10c). The rules are in auth_core.hpp; auth.cpp is the device side.
// ============================================================================

// The token of the Authorization header, or nullptr. `out` must hold auth::kAuthHeaderMax bytes.
static const char* bearer_of(httpd_req_t* req, char* out)
{
    char       header[auth::kAuthHeaderMax];
    bool       present = false;
    const bool fits    = read_header(req, "Authorization", header, sizeof(header), &present);
    return (present && fits && auth::bearerToken(header, out, auth::kAuthHeaderMax)) ? out : nullptr;
}

static esp_err_t send_auth_error(httpd_req_t* req, auth::Result r, uint32_t retryS = 0)
{
    const auth::Http h = auth::httpFor(r);
    return AsyncWebServer::sendError(req, h.status, h.reason, h.code, retryS);
}

// Reads a string member. False when it is missing or not a string.
static bool json_string(const cJSON* root, const char* key, const char** value, size_t* len)
{
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsString(item) || item->valuestring == nullptr)
    {
        return false;
    }
    *value = item->valuestring;
    *len   = strlen(item->valuestring);
    return true;
}

static void add_session_fields(cJSON* obj, const char* token)
{
    cJSON_AddStringToObject(obj, "token", token);
    cJSON_AddNumberToObject(obj, "idle_timeout_s", static_cast<double>(auth::SessionTable::kIdleMs) / 1000.0);
    cJSON_AddNumberToObject(obj, "max_age_s", static_cast<double>(auth::SessionTable::kMaxAgeMs) / 1000.0);
}

static esp_err_t send_session(httpd_req_t* req, int status, const char* token)
{
    cJSON* root = cJSON_CreateObject();
    if (root != nullptr)
    {
        cJSON_AddStringToObject(root, "status", "ok");
        add_session_fields(root, token);
    }
    return send_json_response(req, {root, status});
}

esp_err_t g_auth_index_handler(httpd_req_t* req, void* arg)
{
    char                           token[auth::kAuthHeaderMax];
    const auth_service::StatusView v    = auth_service::status(bearer_of(req, token));
    cJSON*                         root = cJSON_CreateObject();
    if (root != nullptr)
    {
        const char* state = v.state == auth::State::Claimed ? "claimed"
                            : v.state == auth::State::Unavailable ? "unavailable"
                            : v.pending                           ? "pending"
                                                                  : "unclaimed";
        cJSON_AddStringToObject(root, "state", state);
        cJSON_AddBoolToObject(root, "authenticated", v.authenticated);
        cJSON_AddNumberToObject(root, "retry_after_s", v.retryAfterS);
        cJSON_AddNumberToObject(root, "claim_expires_in_s", v.pendingExpiresS);
        cJSON_AddBoolToObject(root, "ap_open", WIFI::getInstance().isApOpen());
        if (auth_dev::kBypass)
        {
            cJSON_AddBoolToObject(root, "dev_bypass", true);  // absent in every normal build
        }

        // The client that posted a claim polls with its id and learns the outcome here.
        char claimId[auth::kClaimIdLen + 8];
        if (get_query_str(req, "claim_id", claimId, sizeof(claimId)))
        {
            const auth::ClaimView c = auth_service::claimStatus(claimId);
            static const char* const names[] = {"pending", "committed", "expired", "conflict", "failed", "unknown"};
            cJSON* claim = cJSON_AddObjectToObject(root, "claim");
            if (claim != nullptr)
            {
                cJSON_AddStringToObject(claim, "result", names[static_cast<size_t>(c.status)]);
                if (c.status == auth::ClaimStatus::Pending)
                {
                    cJSON_AddNumberToObject(claim, "expires_in_s", c.expiresInS);
                }
                if (c.hasToken)
                {
                    add_session_fields(claim, c.token);
                }
            }
        }
    }
    return send_json_response(req, {root, 200});
}

esp_err_t p_auth_claim_handler(httpd_req_t* req, void* arg)
{
    cJSON* body = get_validated_json_payload(req, 256);
    if (body == nullptr)
    {
        return ESP_OK;
    }
    const char* pin    = nullptr;
    size_t      pinLen = 0;
    char        id[auth::kClaimIdLen + 1];
    auth::Result r = json_string(body, "pin", &pin, &pinLen) ? auth_service::claimPost(pin, pinLen, id)
                                                             : auth::Result::BadFormat;
    cJSON_Delete(body);
    if (r != auth::Result::Ok)
    {
        return send_auth_error(req, r);
    }
    cJSON* root = cJSON_CreateObject();
    if (root != nullptr)
    {
        cJSON_AddStringToObject(root, "status", "pending");
        cJSON_AddStringToObject(root, "claim_id", id);
        cJSON_AddNumberToObject(root, "expires_in_s", static_cast<double>(auth::kClaimWindowUs) / 1e6);
    }
    return send_json_response(req, {root, 202});
}

esp_err_t p_auth_login_handler(httpd_req_t* req, void* arg)
{
    cJSON* body = get_validated_json_payload(req, 256);
    if (body == nullptr)
    {
        return ESP_OK;
    }
    const char* pin    = nullptr;
    size_t      pinLen = 0;
    char        token[auth::SessionTable::kTokenLen + 1];
    uint32_t    retryS = 0;
    auth::Result r = json_string(body, "pin", &pin, &pinLen) ? auth_service::login(pin, pinLen, token, &retryS)
                                                             : auth::Result::BadFormat;
    cJSON_Delete(body);
    return r == auth::Result::Ok ? send_session(req, 200, token) : send_auth_error(req, r, retryS);
}

esp_err_t p_auth_logout_handler(httpd_req_t* req, void* arg)
{
    char               token[auth::kAuthHeaderMax];
    const auth::Result r = auth_service::logout(bearer_of(req, token));
    if (r != auth::Result::Ok)
    {
        return send_auth_error(req, r);
    }
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t put_auth_pin_handler(httpd_req_t* req, void* arg)
{
    cJSON* body = get_validated_json_payload(req, 256);
    if (body == nullptr)
    {
        return ESP_OK;
    }
    const char* oldPin = nullptr;
    const char* newPin = nullptr;
    size_t      oldLen = 0;
    size_t      newLen = 0;
    uint32_t    retryS = 0;
    auth::Result r = (json_string(body, "old_pin", &oldPin, &oldLen) && json_string(body, "new_pin", &newPin, &newLen))
                         ? auth_service::changePin(oldPin, oldLen, newPin, newLen, &retryS)
                         : auth::Result::BadFormat;
    cJSON_Delete(body);
    if (r != auth::Result::Ok)
    {
        return send_auth_error(req, r, retryS);
    }
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t g_settings_index_handler(httpd_req_t* req, void* arg)
{
    return send_json_response(req, m_settings_get());
}

esp_err_t patch_settings_index_handler(httpd_req_t* req, void* arg)
{
    cJSON* root = get_validated_json_payload(req, 2048);
    if (root == nullptr)
    {
        return ESP_OK;
    }

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
    {
        return AsyncWebServer::sendError(req, 400, "Invalid SD card path", http_error::kBadPath);
    }

    sd_tree::Limits limits;
    if (to_esp_err(http_parse::treeLimits(req->uri, &limits)) != ESP_OK)
    {
        return AsyncWebServer::sendError(req, 400, "depth must be 0..5, limit 1..512, offset 0..100000");
    }

    return send_json_response(req, m_sdcard_file_tree_get(path_buf, limits));
}

esp_err_t g_sd_card_file_read_handler(httpd_req_t* req, void* arg)
{
    const char* api_route = "/api/v1/sd_card/file";

    bool download = false;
    if (httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff") != ESP_OK ||
        httpd_resp_set_hdr(req, "Cache-Control", "no-store") != ESP_OK)
    {
        return ESP_FAIL;
    }

    const esp_err_t download_query = get_download_query(req, &download);
    if (download_query != ESP_OK && download_query != ESP_ERR_NOT_FOUND)
    {
        return send_json_error_response(req, 400);
    }

    char path_buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (get_sd_rest_path(req, api_route, path_buf, sizeof(path_buf), false) != ESP_OK)
    {
        return AsyncWebServer::sendError(req, 400, "Invalid SD card path", http_error::kBadPath);
    }

    // Keep this bounded lease through stat, open, every read/send, and close.
    // SDCard's lower-level calls take the same recursive mutex, so another SD
    // request cannot unmount or mutate the file while its response is active.
    SDCard::Operation operation = SDCard::getInstance().acquire_operation(SD_FILE_READ_OPERATION_TIMEOUT, true);
    if (!operation)
    {
        return send_json_error_response(req, 503);
    }

    struct stat file_stat;
    const esp_err_t stat_err = SDCard::getInstance().get_file_stat(path_buf, &file_stat);
    if (stat_err != ESP_OK)
    {
        if (stat_err == ESP_ERR_INVALID_STATE)
        {
            return send_json_error_response(req, 503);
        }
        if (stat_err == ESP_ERR_NOT_FOUND)
        {
            return AsyncWebServer::sendError(req, 404, "File not found");
        }
        return send_json_error_response(req, 500);
    }
    if (!S_ISREG(file_stat.st_mode))
    {
        return AsyncWebServer::sendError(req, 404, "File not found");
    }

    FILE*           fd       = nullptr;
    const esp_err_t open_err = SDCard::getInstance().open_file(path_buf, "rb", fd);
    if (open_err != ESP_OK || fd == nullptr)
    {
        if (fd != nullptr)
        {
            SDCard::getInstance().close_file(fd);
        }
        if (open_err == ESP_ERR_INVALID_STATE)
        {
            return send_json_error_response(req, 503);
        }
        if (open_err == ESP_ERR_NOT_FOUND)
        {
            return AsyncWebServer::sendError(req, 404, "File not found");
        }
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
    const int64_t download_deadline = esp_timer_get_time() + DOWNLOAD_ABSOLUTE_DEADLINE_US;
    while (true)
    {
        if (esp_timer_get_time() >= download_deadline)
        {
            ESP_LOGW(TAG, "SD download exceeded %d s, aborting to release the SD lease",
                     static_cast<int>(DOWNLOAD_ABSOLUTE_DEADLINE_US / 1000000));
            SDCard::getInstance().close_file(fd);
            return bytes_sent ? ESP_FAIL : send_json_error_response(req, 504);
        }
        const size_t read_bytes = SDCard::getInstance().file_read_chunk(fd, chunk, sizeof(chunk));
        if (ferror(fd) != 0)
        {
            const esp_err_t close_err = SDCard::getInstance().close_file(fd);
            fd                        = nullptr;
            if (close_err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to close SD file after read error: %s", esp_err_to_name(close_err));
            }
            return bytes_sent ? ESP_FAIL : send_json_error_response(req, 500);
        }
        if (read_bytes == 0)
        {
            break;
        }

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
    {
        ESP_LOGE(TAG, "Failed to close SD file: %s", esp_err_to_name(close_err));
    }

    // The response is committed once its stream terminator is attempted. Do
    // not try to append a JSON error after any response bytes were sent.
    return (send_err == ESP_OK && close_err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t put_file_upload_handler(httpd_req_t* req, void* arg)
{
    size_t content_length = 0;
    if (!get_validated_upload_content_length(req, &content_length))
    {
        return send_json_response(req, make_upload_result("error", "Invalid Content-Length", 400));
    }

    const char* api_route = "/api/v1/sd_card/file";
    char        path_buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (get_sd_rest_path(req, api_route, path_buf, sizeof(path_buf), false) != ESP_OK)
    {
        return send_json_response(req, make_upload_result("error", "Invalid SD card path", 400));
    }

    const char* relative_path = path_buf;
    size_t      path_len      = strlen(relative_path);

    if (content_length > MAX_UPLOAD_CONTENT_LENGTH)
    {
        return send_json_response(req, make_upload_result("error", "Content too large", 413));
    }

    // If the path ends with '/', treat it as a directory creation request
    if (path_len > 0 && relative_path[path_len - 1] == '/')
    {
        if (content_length != 0)
        {
            return send_json_response(req, make_upload_result("error", "Directory upload body must be empty", 400));
        }

        int         response_status = 200;
        const char* response_reason = nullptr;
        bool        created         = false;
        {
            SDCard::Operation operation = SDCard::getInstance().acquire_operation(UPLOAD_OPERATION_TIMEOUT, true);
            if (!operation)
            {
                response_status = 503;
                response_reason = operation.status() == ESP_ERR_TIMEOUT ? "SD card busy" : "SD Card not mounted";
            }
            else
            {
                struct stat dir_stat;
                created = SDCard::getInstance().get_file_stat(relative_path, &dir_stat) != ESP_OK || !S_ISDIR(dir_stat.st_mode);
                if (SDCard::getInstance().create_directory(relative_path) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to create directory: %s", relative_path);
                    response_status = 500;
                    response_reason = "Failed to create directory";
                }
            }
        }

        if (response_status != 200)
        {
            return send_json_response(req, make_upload_result("error", response_reason, response_status));
        }
        return send_json_response(req, make_upload_result("success", nullptr, created ? 201 : 200));
    }

    const int64_t upload_started_at = esp_timer_get_time();
    int           response_status   = 200;
    const char*   response_reason   = nullptr;
    bool          send_response     = true;
    bool          created           = false;

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
            created                        = !target_existed;
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
                        {
                            ESP_LOGE(TAG, "Failed to close upload target: %s", esp_err_to_name(close_err));
                        }
                    }
                    if (!target_existed)
                    {
                        const esp_err_t delete_err = SDCard::getInstance().delete_file(relative_path);
                        if (delete_err != ESP_OK && delete_err != ESP_ERR_NOT_FOUND)
                        {
                            ESP_LOGE(TAG, "Failed to remove failed upload %s: %s", relative_path,
                                     esp_err_to_name(delete_err));
                        }
                    }
                    response_status = open_err == ESP_ERR_INVALID_STATE ? 503 : 500;
                    response_reason = "Storage error";
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
                        {
                            continue;
                        }

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
                        {
                            ESP_LOGE(TAG, "Failed to remove partial upload %s: %s", relative_path,
                                     esp_err_to_name(delete_err));
                        }
                    }
                }
            }
        }
    }

    if (!send_response)
    {
        return ESP_FAIL;
    }

    if (response_status != 200)
    {
        return send_json_response(req, make_upload_result("error", response_reason, response_status));
    }
    return send_json_response(req, make_upload_result("success", nullptr, created ? 201 : 200));
}

esp_err_t d_file_delete_handler(httpd_req_t* req, void* arg)
{
    const char* api_route = "/api/v1/sd_card/file";
    char        path_buf[CONFIG_HTTPD_MAX_URI_LEN + 1];
    if (get_sd_rest_path(req, api_route, path_buf, sizeof(path_buf), false) != ESP_OK)
    {
        return AsyncWebServer::sendError(req, 400, "Invalid SD card path", http_error::kBadPath);
    }

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
        .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_BINARY, .payload = packet, .len = 1, .left_len = 0};

    AsyncWebServer::getInstance().wsBroadcast(&ws_frame);
}

// IDF does not call ws_socket_handler for the upgrade GET, so the Origin check and the stream start live in handshake
// hooks. Without them an evil Origin was accepted and the 1 Hz CAN status frame never started.
#if !defined(CONFIG_HTTPD_WS_PRE_HANDSHAKE_CB_SUPPORT) || !defined(CONFIG_HTTPD_WS_POST_HANDSHAKE_CB_SUPPORT)
#error "WebSocket handshake callbacks are off: set CONFIG_HTTPD_WS_PRE_HANDSHAKE_CB_SUPPORT and CONFIG_HTTPD_WS_POST_HANDSHAKE_CB_SUPPORT"
#endif

// Browsers allow cross-site WebSockets, so the upgrade request is the only chance to look at Origin.
static esp_err_t ws_pre_handshake(httpd_req_t* req)
{
    return pass_host_origin(req) ? ESP_OK : ESP_FAIL;  // pass_host_origin already sent the 403
}

static esp_err_t ws_post_handshake(httpd_req_t* req)
{
    ESP_LOGI(TAG, "WS Connect: Client #%d", httpd_req_to_sockfd(req));
    wsStreamingEnabled.store(true);
    if (WSDataStreamSemaphore != nullptr)
    {
        xSemaphoreGive(WSDataStreamSemaphore);
    }
    return ESP_OK;
}

esp_err_t ws_socket_handler(httpd_req_t* req)
{
    int sockfd = httpd_req_to_sockfd(req);

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        return ret;
    }

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

    if (!input_limits::wsFrameLenOk(ws_pkt.len))
    {
        ESP_LOGW(TAG, "WS Client #%d frame of %zu bytes too large, closing", sockfd, ws_pkt.len);
        return ESP_FAIL;  // a handler error makes httpd close the socket
    }

    if (ws_pkt.len > 0)
    {
        uint8_t* buf = (uint8_t*)malloc(ws_pkt.len + 1);
        if (!buf)
        {
            return ESP_ERR_NO_MEM;
        }

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
    {
        return;
    }

    static uint8_t packet[PID_STREAM_PACKET_SIZE];
    esp_err_t      err = pid_stream_packet_get(pid, packet);

    if (err == ESP_OK)
    {
        httpd_ws_frame_t ws_frame = {.final      = true,
                                     .fragmented = false,
                                     .type       = HTTPD_WS_TYPE_BINARY,
                                     .payload    = packet,
                                     .len        = PID_STREAM_PACKET_SIZE,
                                     .left_len   = 0};

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
                                     .len        = CAN_STATUS_PACKET_SIZE,
                                     .left_len   = 0};
        AsyncWebServer::getInstance().wsBroadcast(&ws_frame);
    }
}

// Any socket that closes (close frame or not) may have been the last stream client. Runs in the httpd task.
static void ws_socket_closed(int fd)
{
    if (AsyncWebServer::getInstance().getActiveWSClientCount(fd) == 0)
    {
        b_pid_stream_enabled.store(false);
        wsStreamingEnabled.store(false);
    }
}

static void WSDataStreamTask(void* pvParameters)
{
    if (WSDataStreamSemaphore != nullptr)
    {
        TickType_t       xLastWakeTime = xTaskGetTickCount();
        const TickType_t xPeriod       = pdMS_TO_TICKS(WS_DAT_STREAM_PERIOD);

        while (!wsTaskStopRequested.load())
        {
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            if (wsTaskStopRequested.load())
            {
                break;
            }
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

    wsTaskRunning.store(false);
    vTaskDelete(NULL);
}

// ============================================================================
// 4. ROUTE TABLE & REGISTRATION
// ============================================================================

#define ROUTE_ENTRY(uri, method, handler, gated) {uri, HTTP_##method, handler, (gated) != 0},
const RouteDef api_routes[] = {API_ROUTES(ROUTE_ENTRY)};
#undef ROUTE_ENTRY

static esp_err_t register_routes()
{
    AsyncWebServer& server     = AsyncWebServer::getInstance();
    size_t          num_routes = sizeof(api_routes) / sizeof(api_routes[0]);

    for (size_t i = 0; i < num_routes; i++)
    {
        ESP_RETURN_ON_ERROR(server.registerRoute(api_routes[i].uri, api_routes[i].method, route_shim,
                                                  const_cast<RouteDef*>(&api_routes[i])),
                            TAG, "route %s", api_routes[i].uri);
    }
    return server.registerSocketRoute("/ws", ws_socket_handler, NULL, ws_pre_handshake, ws_post_handshake);
}

}  // namespace

// ============================================================================
// 5. PUBLIC BOOTSTRAPPER (Sits at the bottom looking up at the namespace)
// ============================================================================

// The BOOT button and the PIN state only matter while the API is up. Both come up before the first request can arrive.
// A failure here never stops the device: without the button nobody can claim or recover, without a readable PIN record
// the gated routes answer 503, and everything open keeps working.
static void start_access_control()
{
    const esp_err_t buttonErr = button_init();
    if (buttonErr != ESP_OK)
    {
        ESP_LOGW(TAG, "BOOT button unavailable (%s); PIN claim and recovery disabled", esp_err_to_name(buttonErr));
    }
    const esp_err_t authErr = auth_service::start();
    if (authErr != ESP_OK)
    {
        ESP_LOGE(TAG, "Access control failed to start (%s); gated routes closed", esp_err_to_name(authErr));
    }
}

// Hook first, button second. ESP_ERR_TIMEOUT: the button task is still busy, call again.
static esp_err_t stop_access_control()
{
    auth_service::stop();
    return button_deinit();
}

esp_err_t setup_web_server()
{
    start_access_control();

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
    // A peer that vanished without FIN or RST (phone left the Wi-Fi) is dropped after about 10 + 3 * 5 s.
    server_config.httpd_config.keep_alive_enable   = true;
    server_config.httpd_config.keep_alive_idle     = 10;
    server_config.httpd_config.keep_alive_interval = 5;
    server_config.httpd_config.keep_alive_count    = 3;
    server_config.on_socket_closed               = ws_socket_closed;
    server_config.request_filter                 = request_filter;

    esp_err_t ret = AsyncWebServer::getInstance().start(server_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start web server");
        (void)stop_access_control();
        return ret;
    }

    ret = register_routes();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Route registration failed (%s); web server disabled", esp_err_to_name(ret));
        AsyncWebServer::getInstance().stop();
        (void)stop_access_control();
        return ret;
    }

    OBD2::getInstance().subscribe(pid_stream_callback);

    WSDataStreamSemaphore = xSemaphoreCreateBinary();
    if (WSDataStreamSemaphore == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create the WS stream semaphore");
        AsyncWebServer::getInstance().stop();
        (void)stop_access_control();
        return ESP_ERR_NO_MEM;
    }

    wsTaskStopRequested.store(false);
    wsTaskRunning.store(true);
    BaseType_t result =
        xTaskCreatePinnedToCore(WSDataStreamTask, "WSDataStreamTask", WS_DAT_STREAM_TASK_STACK_SIZE, NULL,
                                WS_DAT_STREAM_TASK_PRIORITY, &xWSDataStreamTaskHandle, WS_DAT_STREAM_TASK_CORE_ID);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create WSDataStreamTask!");
        wsTaskRunning.store(false);
        AsyncWebServer::getInstance().stop();
        (void)stop_access_control();
        vSemaphoreDelete(WSDataStreamSemaphore);
        WSDataStreamSemaphore = nullptr;
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Order matters (WEB-1): the stream task leaves on its own first (no vTaskDelete inside a broadcast), then httpd stops
// (no handler can touch the semaphore any more), and only then is the semaphore deleted. A timeout anywhere leaves
// everything that is still needed alive, so the caller can retry.
esp_err_t stop_web_server()
{
    constexpr int kTaskExitPolls = 100;  // 10 ms each

    if (wsTaskRunning.load())
    {
        wsTaskStopRequested.store(true);
        if (WSDataStreamSemaphore != nullptr)
        {
            xSemaphoreGive(WSDataStreamSemaphore);  // wakes the task if it sleeps on the semaphore
        }
        for (int i = 0; i < kTaskExitPolls && wsTaskRunning.load(); i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (wsTaskRunning.load())
        {
            ESP_LOGE(TAG, "WSDataStreamTask did not exit");
            return ESP_ERR_TIMEOUT;
        }
    }
    xWSDataStreamTaskHandle = nullptr;

    const esp_err_t err = AsyncWebServer::getInstance().stop();
    if (err != ESP_OK)
    {
        return err;
    }

    if (stop_access_control() != ESP_OK)
    {
        ESP_LOGW(TAG, "BOOT button task did not exit; the next stop retries");
        return ESP_ERR_TIMEOUT;
    }

    if (WSDataStreamSemaphore != nullptr)
    {
        vSemaphoreDelete(WSDataStreamSemaphore);
        WSDataStreamSemaphore = nullptr;
    }
    b_pid_stream_enabled.store(false);
    wsStreamingEnabled.store(false);
    return ESP_OK;
}

// TODO add nrc endpoint and maybe callback with ws frame
