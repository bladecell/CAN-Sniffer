#pragma once

// REST route table as an X-macro (P2.11 NB1, B20). X(uri, METHOD, handler): webserver.cpp expands it into RouteDef
// entries, host tests expand it to the URI and method only (handler tokens are never evaluated there). No ESP-IDF
// includes.
//
// httpd matches the path only (the query string is stripped first), so a documented route never needs a trailing '*'.
// The only wildcard patterns are those where the handler validates the whole remainder as a path parameter
// (<pid> or <path>); anything else would answer typos such as GET /api/v1/dtcs.

// clang-format off
#define API_ROUTES(X)                                                                  \
  X("/",                          GET,    index_handler)                               \
  X("/api/v1/pid_def/*",          GET,    g_pid_def_index_handler)                     \
  X("/api/v1/pid_def/*",          DELETE, d_pid_def_index_handler)                     \
  X("/api/v1/pid_def",            GET,    g_pid_def_index_handler)                     \
  X("/api/v1/pid_def",            POST,   p_pid_def_index_handler)                     \
  X("/api/v1/pid_def",            PUT,    put_pid_def_index_handler)                   \
  X("/api/v1/pid_def/save",       POST,   p_pid_def_save_index_handler)                \
  X("/api/v1/pid_def/load",       POST,   p_pid_def_load_index_handler)                \
  X("/api/v1/can_bus",            GET,    g_can_bus_index_handler)                     \
  X("/api/v1/obd2",               GET,    g_obdii_index_handler)                       \
  X("/api/v1/pid_data/*",         GET,    g_pid_data_index_handler)                    \
  X("/api/v1/pid_data",           GET,    g_pid_data_index_handler)                    \
  X("/api/v1/req/auto_poll",      POST,   p_auto_poll_index_handler)                   \
  X("/api/v1/req/static_pid",     POST,   p_static_pid_index_handler)                  \
  X("/api/v1/vin",                GET,    g_vin_index_handler)                         \
  X("/api/v1/req/vin",            POST,   p_vin_index_handler)                         \
  X("/api/v1/dtc",                GET,    g_dtc_index_handler)                         \
  X("/api/v1/req/dtc",            POST,   p_dtc_index_handler)                         \
  X("/api/v1/req/clear_dtc",      POST,   p_clear_dtc_index_handler)                   \
  X("/api/v1/system",             GET,    g_system_index_handler)                      \
  X("/api/v1/system/reboot",      POST,   p_system_reboot_index_handler)               \
  X("/api/v1/system/copy_file",   POST,   p_system_copy_file_index_handler)            \
  X("/api/v1/sd_card/info",       GET,    g_sd_card_info_handler)                      \
  X("/api/v1/sd_card/format",     POST,   p_sd_card_format_handler)                    \
  X("/api/v1/sd_card/tree",       GET,    g_sd_card_file_tree_handler)                 \
  X("/api/v1/sd_card/tree/*",     GET,    g_sd_card_file_tree_handler)                 \
  X("/api/v1/sd_card/file/*",     POST,   p_file_upload_handler)                       \
  X("/api/v1/sd_card/file/*",     DELETE, d_file_delete_handler)                       \
  X("/api/v1/sd_card/file/*",     GET,    g_sd_card_file_read_handler)                 \
  X("/api/v1/settings",           GET,    g_settings_index_handler)                    \
  X("/api/v1/settings",           POST,   p_settings_index_handler)
// clang-format on
