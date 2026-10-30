#pragma once

// REST route table as an X-macro (P2.11 NB1, B20). X(uri, METHOD, handler, gated): webserver.cpp expands it into RouteDef
// entries, host tests expand it to the URI and method only (handler tokens are never evaluated there). No ESP-IDF
// includes.
//
// `gated` (P2.10c): 1 = needs a valid Bearer session (requireAdmin) before the handler runs. The set is the access-control
// decision in docs/decisions.md (2026-10-18); test_api_routes.cpp pins it exactly. New routes default to 0 only by
// being written that way: decide for each one.
//
// httpd matches the path only (the query string is stripped first), so a documented route never needs a trailing '*'.
// The only wildcard patterns are those where the handler validates the whole remainder as a path parameter
// (<pid> or <path>); anything else would answer typos such as GET /api/v1/dtcs.

// clang-format off
#define API_ROUTES(X)                                                                          \
  X("/api/v1/wifi/scan",             POST,   p_wifi_scan_handler,              1)                \
  X("/api/v1/wifi/scan",             GET,    g_wifi_scan_handler,              1)                \
  X("/",                            GET,    index_handler,                   0)                \
  X("/api/v1/pid_def/*",            GET,    g_pid_def_index_handler,         0)                \
  X("/api/v1/pid_def/*",            DELETE, d_pid_def_index_handler,         1)                \
  X("/api/v1/pid_def",              GET,    g_pid_def_index_handler,         0)                \
  X("/api/v1/pid_def",              POST,   p_pid_def_index_handler,         1)                \
  X("/api/v1/pid_def",              PUT,    put_pid_def_index_handler,       1)                \
  X("/api/v1/pid_def/save",         POST,   p_pid_def_save_index_handler,    1)                \
  X("/api/v1/pid_def/load",         POST,   p_pid_def_load_index_handler,    1)                \
  X("/api/v1/can_bus",              GET,    g_can_bus_index_handler,         0)                \
  X("/api/v1/obd2",                 GET,    g_obdii_index_handler,           0)                \
  X("/api/v1/obd2",                 PATCH,  patch_obd2_index_handler,        0)                \
  X("/api/v1/pid_data/*",           GET,    g_pid_data_index_handler,        0)                \
  X("/api/v1/pid_data",             GET,    g_pid_data_index_handler,        0)                \
  X("/api/v1/req/static_pid",       POST,   p_static_pid_index_handler,      0)                \
  X("/api/v1/vin",                  GET,    g_vin_index_handler,             0)                \
  X("/api/v1/req/vin",              POST,   p_vin_index_handler,             0)                \
  X("/api/v1/dtc",                  GET,    g_dtc_index_handler,             0)                \
  X("/api/v1/dtc/descriptions",     GET,    g_dtc_descriptions_handler,      0)                \
  X("/api/v1/req/dtc",              POST,   p_dtc_index_handler,             0)                \
  X("/api/v1/req/clear_dtc",        POST,   p_clear_dtc_index_handler,       1)                \
  X("/api/v1/recorder",             GET,    g_recorder_handler,              0)                \
  X("/api/v1/recorder/start",       POST,   p_recorder_start_handler,        1)                \
  X("/api/v1/recorder/stop",        POST,   p_recorder_stop_handler,         1)                \
  X("/api/v1/recorder/trips",       GET,    g_recorder_trips_handler,        1)                \
  X("/api/v1/can/mode",             GET,    g_can_mode_handler,              0)                \
  X("/api/v1/can/mode",             POST,   p_can_mode_handler,              1)                \
  X("/api/v1/can/send",             POST,   p_can_send_handler,              1)                \
  X("/api/v1/system",               GET,    g_system_index_handler,          0)                \
  X("/api/v1/system/reboot",        POST,   p_system_reboot_index_handler,   1)                \
  X("/api/v1/system/sleep",         POST,   p_system_sleep_index_handler,    1)                \
  X("/api/v1/system/keep_awake",    POST,   p_system_keep_awake_handler,     1)                \
  X("/api/v1/sd_card/info",         GET,    g_sd_card_info_handler,          0)                \
  X("/api/v1/sd_card/copy",         POST,   p_sd_card_copy_handler,          1)                \
  X("/api/v1/sd_card/format",       POST,   p_sd_card_format_handler,        1)                \
  X("/api/v1/sd_card/tree",         GET,    g_sd_card_file_tree_handler,     1)                \
  X("/api/v1/sd_card/tree/*",       GET,    g_sd_card_file_tree_handler,     1)                \
  X("/api/v1/sd_card/file/*",       PUT,    put_file_upload_handler,         1)                \
  X("/api/v1/sd_card/file/*",       DELETE, d_file_delete_handler,           1)                \
  X("/api/v1/sd_card/file/*",       GET,    g_sd_card_file_read_handler,     1)                \
  X("/api/v1/auth",                 GET,    g_auth_index_handler,            0)                \
  X("/api/v1/auth/claim",           POST,   p_auth_claim_handler,            0)                \
  X("/api/v1/auth/login",           POST,   p_auth_login_handler,            0)                \
  X("/api/v1/auth/logout",          POST,   p_auth_logout_handler,           1)                \
  X("/api/v1/auth/pin",             PUT,    put_auth_pin_handler,            1)                \
  X("/api/v1/settings",             GET,    g_settings_index_handler,        1)                \
  X("/api/v1/settings",             PATCH,  patch_settings_index_handler,    1)
// clang-format on

// Routes that put an OBD request on the bus or arm the DTC/VIN collectors. They answer 409 RAW_MODE outside Monitor mode
// (checked under the mode lock; see can_mode_service::ObdRequestGuard), 503 BUSY if the lock is not obtained. Everything under
// /api/v1/req/ belongs here; test_api_routes.cpp pins the list. Not here, guarded on their own: POST /can/send (needs Raw),
// POST /recorder/start and PATCH /obd2 auto_poll true (refused in Raw). Read-only GETs of stored data are not requests.
// clang-format off
#define OBD_REQUEST_ROUTES(X)                \
  X("/api/v1/req/static_pid", POST)          \
  X("/api/v1/req/vin",        POST)          \
  X("/api/v1/req/dtc",        POST)          \
  X("/api/v1/req/clear_dtc",  POST)
// clang-format on
