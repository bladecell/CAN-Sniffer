// Recorder wire rules shared by REST and WebSocket (R4). Hardware-free, host-tested.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "cJSON.h"
#include "esp_err.h"
#include "recorder_core.hpp"

namespace recorder_http
{
constexpr uint8_t kMsgTypeRecorder  = 0x04;
constexpr size_t  kStateFrameBytes  = 16;

// POST /recorder/start body: an object whose only key is an optional "pids" array (at most 32) of
// {pid: 0..65535, min_interval_ms?: 0..60000}; nothing else. `out` must hold kMaxPids entries.
// n = 0 means "the PIDs OBD2 currently polls". False for anything else.
inline bool integerIn(const cJSON* v, double lo, double hi, double& out)
{
  if (!cJSON_IsNumber(v) || !std::isfinite(v->valuedouble) || v->valuedouble < lo || v->valuedouble > hi ||
      v->valuedouble != std::floor(v->valuedouble))
  {
    return false;
  }
  out = v->valuedouble;
  return true;
}

inline bool parseStart(const cJSON* body, recorder_core::PidSel* out, size_t& n)
{
  n = 0;
  if (!cJSON_IsObject(body) || out == nullptr) { return false; }
  for (const cJSON* k = body->child; k != nullptr; k = k->next)
  {
    if (k->string == nullptr || std::strcmp(k->string, "pids") != 0) { return false; }
  }
  const cJSON* pids = cJSON_GetObjectItemCaseSensitive(body, "pids");
  if (pids == nullptr) { return true; }
  if (!cJSON_IsArray(pids)) { return false; }
  for (const cJSON* item = pids->child; item != nullptr; item = item->next)
  {
    if (n == recorder_core::kMaxPids || !cJSON_IsObject(item)) { return false; }
    double pid = 0, spacing = 0;
    bool   havePid = false;
    for (const cJSON* f = item->child; f != nullptr; f = f->next)
    {
      if (f->string != nullptr && std::strcmp(f->string, "pid") == 0) { havePid = integerIn(f, 0, 65535, pid); if (!havePid) { return false; } }
      else if (f->string != nullptr && std::strcmp(f->string, "min_interval_ms") == 0)
      {
        if (!integerIn(f, 0, recorder_core::kMaxSpacingMs, spacing)) { return false; }
      }
      else { return false; }
    }
    if (!havePid) { return false; }
    out[n++] = {static_cast<uint16_t>(pid), static_cast<uint16_t>(spacing)};
  }
  return true;
}

// HTTP status for Recorder::start(). ESP_ERR_NOT_FINISHED = a session is live (recording, stopping, or paused by a card
// error), so a client must stop first (409); ESP_ERR_NOT_ALLOWED = start cooldown (429 + Retry-After); ESP_ERR_INVALID_STATE = the recorder is not initialized (503).
inline int startHttpStatus(esp_err_t err)
{
  switch (err)
  {
    case ESP_OK: return 200;
    case ESP_ERR_NOT_FINISHED: return 409;
    case ESP_ERR_NOT_ALLOWED: return 429;
    case ESP_ERR_INVALID_ARG: return 422;
    case ESP_ERR_INVALID_STATE:
    case ESP_ERR_NOT_FOUND:
    case ESP_ERR_TIMEOUT:
    case ESP_FAIL: return 503;  // ESP_FAIL: the first trip file could not be opened (the reason is in last_error)
    default: return 500;
  }
}

struct StateFields
{
  recorder_core::State state;
  uint32_t             trip;
  uint32_t             rows;
  uint32_t             dropped;
  bool                 hasError;
};

// 0x04, length 14, state (0 idle, 1 recording, 2 stopping, 3 error), u32 trip, u32 rows, u32 dropped (little-endian), error flag.
inline void encodeStateFrame(uint8_t* out, const StateFields& f)
{
  size_t o = 0;
  out[o++] = kMsgTypeRecorder;
  out[o++] = kStateFrameBytes - 2;
  out[o++] = static_cast<uint8_t>(f.state);
  for (const uint32_t v : {f.trip, f.rows, f.dropped})
  {
    for (int shift = 0; shift < 32; shift += 8) { out[o++] = static_cast<uint8_t>(v >> shift); }
  }
  out[o++] = f.hasError ? 1 : 0;
}
}  // namespace recorder_http
