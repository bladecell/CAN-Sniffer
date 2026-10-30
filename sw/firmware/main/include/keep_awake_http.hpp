#pragma once

// POST /system/keep_awake body: exactly one key, `enabled`, a JSON boolean. Hardware-free, host-tested.
#include <cstring>

#include "cJSON.h"

namespace keep_awake_http
{
inline bool parseBody(const cJSON* body, bool& enabled)
{
  if (!cJSON_IsObject(body) || body->child == nullptr || body->child->next != nullptr)
  {
    return false;
  }
  const cJSON* v = body->child;
  if (v->string == nullptr || strcmp(v->string, "enabled") != 0 || !cJSON_IsBool(v))
  {
    return false;
  }
  enabled = cJSON_IsTrue(v);
  return true;
}
}  // namespace keep_awake_http
