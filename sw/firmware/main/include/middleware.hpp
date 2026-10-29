// middleware.hpp
#pragma once
#include <cstdint>

#include "cJSON.h"
#include "esp_err.h"
#include "recorder_core.hpp"
#include "tree_limits.hpp"

// Forward declarations to minimize includes
class PIDDefinition;
struct PIDData_t;

#define MSG_TYPE_LOG 0x01
#define MSG_TYPE_PID 0x02
#define MSG_TYPE_CAN_STATUS 0x03
#define MSG_TYPE_RECORDER 0x04  // = recorder_http::kMsgTypeRecorder

#define PID_STREAM_PACKET_SIZE 20
#define CAN_STATUS_PACKET_SIZE 12
#define RECORDER_STATE_PACKET_SIZE 16

// The HTTP outcome is explicit and is never inferred from the JSON body.
struct MiddlewareJsonResult
{
    cJSON* body;
    int    http_status;
    unsigned retry_after_s = 0;
};

// Middlewares
MiddlewareJsonResult m_pid_def_get(int filter_id);
MiddlewareJsonResult m_pid_data_get(int filter_id);
MiddlewareJsonResult m_pid_def_delete(int filter_id);
MiddlewareJsonResult m_pid_def_post(cJSON* data);
// Returns an error JSON tree on failure and nullptr after a committed PUT.
// The caller owns and must delete a returned tree. A successful call reports
// HTTP 204 through http_status and transfers no JSON ownership.
cJSON* m_pid_def_put(cJSON* data, int* http_status);
// Sets the auto poll intent; the body is {auto_poll, polling} read after the setter ran (HTTP 200), or an error body and 503 when OBD2 is stopped or busy.
MiddlewareJsonResult m_auto_poll_set(bool on);
MiddlewareJsonResult m_can_bus_get();
MiddlewareJsonResult m_wifi_scan_start();
MiddlewareJsonResult m_wifi_scan_get();
MiddlewareJsonResult m_obdii_get();
MiddlewareJsonResult m_obdii_set(cJSON* payload);
MiddlewareJsonResult m_system_get();
MiddlewareJsonResult m_system_reboot();
// Fixed-capacity admission; finish must run after the response attempt, including a failed send.
uint32_t m_system_sleep_reserve();
void m_system_sleep_finish(uint32_t token, bool responseSent);
MiddlewareJsonResult m_sd_card_copy(cJSON* payload);
MiddlewareJsonResult m_sdcard_info_get();
MiddlewareJsonResult m_sdcard_format_post();
// Directory tree below `path`, bounded by `limits` (depth, entry count, top-level offset). The body has "truncated".
MiddlewareJsonResult m_sdcard_file_tree_get(const char* path, const sd_tree::Limits& limits);
MiddlewareJsonResult m_sdcard_file_delete_delete(const char* path);
MiddlewareJsonResult m_vin_get();
MiddlewareJsonResult m_dtc_get(int mode);
MiddlewareJsonResult m_dtc_description_get(const char* target_codes[], size_t count);
MiddlewareJsonResult m_vin_request();
MiddlewareJsonResult m_dtc_request(int mode);
// Sends OBD mode 0x04 (clears codes on a real vehicle). The route is gated by a Bearer session; there is no other check.
MiddlewareJsonResult m_clear_dtc_request();
MiddlewareJsonResult m_static_pid_request();

MiddlewareJsonResult m_pid_def_set(cJSON* payload);
MiddlewareJsonResult m_pid_def_save(cJSON* payload);
MiddlewareJsonResult m_pid_def_load(cJSON* payload);
// Recorder (R4). Start: 200 state, 409 already recording/stopping, 422 bad PID list, 503 no card or recorder not running.
// Stop is idempotent. `n` = 0 starts the PIDs OBD2 currently polls. Stop blocks up to 5 s: call it from a REST worker only.
MiddlewareJsonResult m_recorder_get();
MiddlewareJsonResult m_recorder_start(const recorder_core::PidSel* pids, size_t n);
MiddlewareJsonResult m_recorder_stop();
MiddlewareJsonResult m_recorder_trips_get();
// CAN view mode (can_mode.hpp) and raw send (raw_tx_policy.hpp). Mode: GET 200 {mode, fc, modes}; POST body {mode, fc?}: 422 bad body or
// mode, 409 recording, 503 not RUNNING or polling busy. Send: 200 {status:sent} once the TX-done hook confirmed the frame; 403, 409, 422,
// 429, 503 as in api.md. Both take ownership of nothing: the caller deletes `payload`.
MiddlewareJsonResult m_can_mode_get();
MiddlewareJsonResult m_can_mode_set(const cJSON* payload);
MiddlewareJsonResult m_can_send(const cJSON* payload);
esp_err_t recorder_state_packet_get(uint8_t* out_packet);

MiddlewareJsonResult m_settings_get();
MiddlewareJsonResult m_settings_set(cJSON* payload);

cJSON* single_pid_def_get(uint16_t pid, esp_err_t* operation_error = nullptr);
cJSON* single_pid_data_get(uint16_t pid, esp_err_t* operation_error = nullptr);

esp_err_t pid_stream_packet_get(uint16_t pid, uint8_t* out_packet);
esp_err_t can_status_packet_get(uint8_t* out_packet);
