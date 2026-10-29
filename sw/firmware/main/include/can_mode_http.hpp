#pragma once

// POST /can/mode body and the HTTP mapping of can_mode::Status. Hardware-free, host-tested.
#include <cstring>

#include "can_mode.hpp"
#include "cJSON.h"

namespace can_mode_http
{
// {"mode": "<name>", "fc": bool?} and nothing else. `mode` points into `body`.
inline bool parseBody(const cJSON* body, const char*& mode, bool& fc)
{
  mode = nullptr;
  fc   = false;
  if (!cJSON_IsObject(body)) { return false; }
  bool haveFc = false;
  for (const cJSON* k = body->child; k != nullptr; k = k->next)
  {
    if (k->string == nullptr) { return false; }
    if (std::strcmp(k->string, "mode") == 0 && mode == nullptr)
    {
      if (!cJSON_IsString(k) || k->valuestring == nullptr) { return false; }
      mode = k->valuestring;
    }
    else if (std::strcmp(k->string, "fc") == 0 && !haveFc)
    {
      if (!cJSON_IsBool(k)) { return false; }
      fc     = cJSON_IsTrue(k);
      haveFc = true;
    }
    else { return false; }
  }
  return mode != nullptr;
}

inline int httpStatus(can_mode::Status s)
{
  switch (s)
  {
    case can_mode::Status::ok: return 200;
    case can_mode::Status::unknownMode: return 422;
    case can_mode::Status::recording: return 409;
    default: return 503;
  }
}

inline const char* code(can_mode::Status s)
{
  switch (s)
  {
    case can_mode::Status::unknownMode: return "BAD_MODE";
    case can_mode::Status::notRunning: return "NOT_RUNNING";
    case can_mode::Status::recording: return "RECORDING";
    default: return "BUSY";
  }
}

inline const char* reason(can_mode::Status s)
{
  switch (s)
  {
    case can_mode::Status::unknownMode: return "mode must be \"monitor\" or \"raw\"; body {\"mode\":\"raw\",\"fc\":true}";
    case can_mode::Status::notRunning: return "The device is not in RUNNING state";
    case can_mode::Status::recording: return "The recorder is recording; stop it first";
    default: return "Polling could not be stopped";
  }
}
}  // namespace can_mode_http
