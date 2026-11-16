#pragma once

// POST /can/send: what a raw CAN request may look like. Hardware-free and host-tested. Only standard OBD request IDs, a
// single-frame request, and a fail-closed allow-list of services (docs/decisions.md, 2026-10-29). Clear DTC (0x04) is not
// on it: that stays on its own route.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "cJSON.h"

namespace raw_tx
{
constexpr uint32_t kMinIntervalMs = 250;  // one send per 250 ms, device wide
constexpr size_t   kMaxHexChars   = 23;   // "xx xx xx xx xx xx xx xx"

enum class Error : uint8_t
{
  none,
  badBody,  // 422 BAD_JSON
  badId,    // 422 BAD_ID
  badData,  // 422 BAD_DATA
  blocked   // 403 SERVICE_BLOCKED
};

struct Request
{
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
};

constexpr uint8_t kAllowedServices[] = {0x01, 0x02, 0x03, 0x07, 0x09, 0x0A, 0x19, 0x22};

inline bool idAllowed(uint32_t id)
{
  return id == 0x7DF || (id >= 0x7E0 && id <= 0x7E7);
}

inline bool sidAllowed(uint8_t sid)
{
  for (const uint8_t a : kAllowedServices)
  {
    if (a == sid) { return true; }
  }
  return false;
}

inline int hexValue(char c)
{
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return -1;
}

// "02 01 00" or "020100" or a mix: pairs of hex digits, at most one space between two pairs, nothing else. 1 to `cap` bytes.
inline bool parseHex(const char* s, uint8_t* out, size_t cap, size_t& n)
{
  n = 0;
  if (s == nullptr || out == nullptr || std::strlen(s) > kMaxHexChars) { return false; }
  size_t i = 0;
  while (s[i] != '\0')
  {
    if (n > 0 && s[i] == ' ') { i++; }  // a single separator between pairs
    const int hi = hexValue(s[i]);
    const int lo = hi >= 0 ? hexValue(s[i + 1]) : -1;  // s[i + 1] exists: a non-NUL s[i] was read
    if (hi < 0 || lo < 0 || n >= cap) { n = 0; return false; }
    out[n++] = static_cast<uint8_t>(hi * 16 + lo);
    i += 2;
  }
  if (n == 0) { return false; }
  return true;
}

// Single-frame ISO-TP request: PCI 0x0N with 1 <= N <= 7 and N payload bytes present (extra bytes are padding), then the
// service id must be allow-listed.
inline Error validate(uint32_t id, const uint8_t* data, size_t len)
{
  if (!idAllowed(id)) { return Error::badId; }
  if (data == nullptr || len < 2 || len > 8) { return Error::badData; }
  if ((data[0] & 0xF0) != 0) { return Error::badData; }
  const size_t n = data[0] & 0x0F;
  if (n < 1 || n > 7 || n + 1 > len) { return Error::badData; }
  return sidAllowed(data[1]) ? Error::none : Error::blocked;
}

// body: {"id": number, "data": "hex"} and nothing else.
inline Error parse(const cJSON* body, Request& out)
{
  out = Request{};
  if (!cJSON_IsObject(body)) { return Error::badBody; }
  const cJSON* id   = nullptr;
  const cJSON* data = nullptr;
  for (const cJSON* k = body->child; k != nullptr; k = k->next)
  {
    if (k->string == nullptr) { return Error::badBody; }
    if (std::strcmp(k->string, "id") == 0 && id == nullptr) { id = k; }
    else if (std::strcmp(k->string, "data") == 0 && data == nullptr) { data = k; }
    else { return Error::badBody; }
  }
  if (id == nullptr || data == nullptr || !cJSON_IsString(data) || data->valuestring == nullptr) { return Error::badBody; }
  const double v = cJSON_IsNumber(id) ? id->valuedouble : -1.0;
  if (!(v >= 0.0 && v <= 4095.0) || v != std::floor(v)) { return Error::badId; }  // also catches NaN, inf, strings
  const uint32_t canId = static_cast<uint32_t>(v);
  if (!idAllowed(canId)) { return Error::badId; }
  size_t n = 0;
  if (!parseHex(data->valuestring, out.data, sizeof(out.data), n)) { out = Request{}; return Error::badData; }
  const Error e = validate(canId, out.data, n);
  if (e != Error::none) { out = Request{}; return e; }
  out.id  = canId;
  out.len = static_cast<uint8_t>(n);
  return Error::none;
}

inline int httpStatus(Error e)
{
  return e == Error::blocked ? 403 : (e == Error::none ? 200 : 422);
}

inline const char* code(Error e)
{
  switch (e)
  {
    case Error::badId: return "BAD_ID";
    case Error::badData: return "BAD_DATA";
    case Error::blocked: return "SERVICE_BLOCKED";
    default: return "BAD_JSON";
  }
}

inline const char* reason(Error e)
{
  switch (e)
  {
    case Error::badId: return "id must be 2015 (0x7DF) or 2016-2023 (0x7E0-0x7E7)";
    case Error::badData: return "data must be 1-8 hex bytes with a valid single-frame length byte";
    case Error::blocked: return "This service is not allowed";
    default: return "Body must be {\"id\":n,\"data\":\"02 01 00\"}";
  }
}
}  // namespace raw_tx
