#pragma once

// PATCH /obd2 body rule (P2.12 BR1). Hardware-free, host-tested. Exactly one key, `auto_poll`, a JSON boolean.

#include <cstring>

#include "cJSON.h"

namespace obd2_patch
{
inline bool parseAutoPoll(const cJSON* body, bool& on)
{
  if (!cJSON_IsObject(body) || body->child == nullptr || body->child->next != nullptr)
  {
    return false;
  }
  const cJSON* v = body->child;
  if (v->string == nullptr || strcmp(v->string, "auto_poll") != 0 || !cJSON_IsBool(v))
  {
    return false;
  }
  on = cJSON_IsTrue(v);
  return true;
}
}  // namespace obd2_patch
