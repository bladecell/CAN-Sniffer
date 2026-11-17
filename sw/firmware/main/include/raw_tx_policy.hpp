#pragma once

// POST /can/send: what a raw CAN send may look like. Hardware-free and host-tested. Any 11-bit ID and 1 to 8 data bytes
// (docs/decisions.md UI3e-D1, D4: the OBD-only allow-list is gone); the page's warning and the login are the gate. A
// request is either one frame ({"id","data"}) or a sender plan ({"frames":[...],"period_ms","repeat"}). Bad input is an
// error, never an abort.
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
  badFrames,  // 422 BAD_FRAMES (plan: frames is not a list of 1 to 32 frame objects)
  badPeriod,  // 422 BAD_PERIOD
  badRepeat   // 422 BAD_REPEAT
};

constexpr size_t   kMaxFrames   = 32;       // frames in one sequence
constexpr uint32_t kMinPeriodMs = 10;       // between two frames of the sender
constexpr uint32_t kMaxPeriodMs = 3600000;
constexpr uint32_t kMaxRepeat   = 65535;    // passes over the sequence; 0 = until stopped

struct Request
{
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
};

inline bool idAllowed(uint32_t id)
{
  return id <= 0x7FF;  // standard 11-bit IDs; the TX path does not send extended frames
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

inline Error validate(uint32_t id, const uint8_t* data, size_t len)
{
  if (!idAllowed(id)) { return Error::badId; }
  if (data == nullptr || len < 1 || len > 8) { return Error::badData; }
  return Error::none;
}

// An integer in [lo, hi]; NaN, inf, fractions and non-numbers fail.
inline bool intIn(const cJSON* v, double lo, double hi, uint32_t& out)
{
  if (!cJSON_IsNumber(v)) { return false; }
  const double d = v->valuedouble;
  if (!(d >= lo && d <= hi) || d != std::floor(d)) { return false; }
  out = static_cast<uint32_t>(d);
  return true;
}

// {"id": number, "data": "hex"} and nothing else, as one frame object.
inline Error parseFrame(const cJSON* body, Request& out)
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
  uint32_t canId = 0;
  if (!intIn(id, 0.0, 4095.0, canId) || !idAllowed(canId)) { return Error::badId; }  // also catches NaN, inf, strings
  size_t n = 0;
  if (!parseHex(data->valuestring, out.data, sizeof(out.data), n)) { out = Request{}; return Error::badData; }
  const Error e = validate(canId, out.data, n);
  if (e != Error::none) { out = Request{}; return e; }
  out.id  = canId;
  out.len = static_cast<uint8_t>(n);
  return Error::none;
}

inline Error parse(const cJSON* body, Request& out)
{
  return parseFrame(body, out);
}

// A sender plan: frames sent one every period_ms, the whole sequence repeated `repeat` times.
struct Plan
{
  Request  frames[kMaxFrames];
  uint8_t  count;
  uint32_t periodMs;
  uint32_t repeat;  // 0 = until stopped
};

// body: {"frames":[{"id","data"}, ...1 to 32], "period_ms": 10..3600000, "repeat": 0..65535 (default 1)} and nothing else.
inline Error parsePlan(const cJSON* body, Plan& out)
{
  out = Plan{};
  if (!cJSON_IsObject(body)) { return Error::badBody; }
  const cJSON* frames = nullptr;
  const cJSON* period = nullptr;
  const cJSON* repeat = nullptr;
  for (const cJSON* k = body->child; k != nullptr; k = k->next)
  {
    if (k->string == nullptr) { return Error::badBody; }
    if (std::strcmp(k->string, "frames") == 0 && frames == nullptr) { frames = k; }
    else if (std::strcmp(k->string, "period_ms") == 0 && period == nullptr) { period = k; }
    else if (std::strcmp(k->string, "repeat") == 0 && repeat == nullptr) { repeat = k; }
    else { return Error::badBody; }
  }
  if (frames == nullptr || !cJSON_IsArray(frames)) { return Error::badFrames; }
  size_t n = 0;
  for (const cJSON* f = frames->child; f != nullptr; f = f->next)
  {
    if (n >= kMaxFrames) { out = Plan{}; return Error::badFrames; }
    if (!cJSON_IsObject(f)) { out = Plan{}; return Error::badFrames; }
    const Error e = parseFrame(f, out.frames[n]);
    if (e == Error::badBody) { out = Plan{}; return Error::badFrames; }  // a stray key or a wrong type inside a frame
    if (e != Error::none) { out = Plan{}; return e; }
    n++;
  }
  if (n == 0) { out = Plan{}; return Error::badFrames; }
  uint32_t per = 0;
  if (period == nullptr || !intIn(period, kMinPeriodMs, kMaxPeriodMs, per)) { out = Plan{}; return Error::badPeriod; }
  uint32_t rep = 1;
  if (repeat != nullptr && !intIn(repeat, 0.0, kMaxRepeat, rep)) { out = Plan{}; return Error::badRepeat; }
  out.count    = static_cast<uint8_t>(n);
  out.periodMs = per;
  out.repeat   = rep;
  return Error::none;
}

// Which frame goes next. repeat 0 never finishes. Not thread-safe: the sender task owns it.
class Cursor
{
public:
  Cursor(size_t count, uint32_t repeat) : count_(count), repeat_(repeat) {}
  bool   done() const { return count_ == 0 || (repeat_ != 0 && sent_ >= static_cast<uint64_t>(count_) * repeat_); }
  size_t next()
  {
    const size_t i = static_cast<size_t>(sent_ % count_);
    sent_++;
    return i;
  }
  uint64_t sent() const { return sent_; }

private:
  size_t   count_;
  uint32_t repeat_;
  uint64_t sent_ = 0;
};

inline int httpStatus(Error e)
{
  return e == Error::none ? 200 : 422;
}

inline const char* code(Error e)
{
  switch (e)
  {
    case Error::badId: return "BAD_ID";
    case Error::badData: return "BAD_DATA";
    case Error::badFrames: return "BAD_FRAMES";
    case Error::badPeriod: return "BAD_PERIOD";
    case Error::badRepeat: return "BAD_REPEAT";
    default: return "BAD_JSON";
  }
}

inline const char* reason(Error e)
{
  switch (e)
  {
    case Error::badId: return "id must be 0-2047 (11-bit)";
    case Error::badData: return "data must be 1-8 hex bytes, optional single spaces";
    case Error::badFrames: return "frames must be a list of 1-32 {\"id\",\"data\"} objects";
    case Error::badPeriod: return "period_ms must be an integer 10-3600000";
    case Error::badRepeat: return "repeat must be an integer 0-65535 (0 = until stopped)";
    default: return "Body must be {\"id\":n,\"data\":\"02 01 00\"} or {\"frames\":[...],\"period_ms\":n,\"repeat\":n}";
  }
}
}  // namespace raw_tx
