// Automatic trip start/stop decisions (P3-R6). Pure: no ESP-IDF, no clock, no allocation. Host-tested (test_trip_trigger.cpp).
// The caller samples the inputs once a second and acts on the returned Action. Bad input never throws or aborts:
// a NaN, infinite or negative RPM/speed simply is not evidence that the car is running.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace trip_trigger
{
enum class Mode : uint8_t
{
  Manual,  // trips start and stop only on POST /recorder/start|stop
  Auto,    // start when the bus is up and RPM or speed is above 0, stop after kEndIdleMs without that
};

struct Config
{
  Mode mode              = Mode::Manual;
  bool continuousPolling = false;  // opt-in: keep polling on while the bus is up, so RPM and speed are seen
};

constexpr const char* modeName(Mode m)
{
  return m == Mode::Auto ? "auto" : "manual";
}

inline bool parseMode(const char* s, Mode& out)
{
  if (s == nullptr) { return false; }
  if (std::strcmp(s, "auto") == 0) { out = Mode::Auto; return true; }
  if (std::strcmp(s, "manual") == 0) { out = Mode::Manual; return true; }
  return false;
}

constexpr uint32_t kEndIdleMs = 60000;  // bus lost or RPM/speed 0 for this long ends an automatic trip

constexpr bool wantPolling(const Config& c, bool busUp)
{
  return c.continuousPolling && busUp;
}

struct Input
{
  bool     busUp;
  float    rpm;       // NaN when unknown or stale
  float    speedKmh;  // NaN when unknown or stale
  float    accelG;    // reserved for the IMU (HW v2): ignored, pass NaN
  bool     recording; // a session is live
  bool     canStart;  // the recorder would accept a start now (cooldown over, not stopping)
  uint32_t nowMs;
};

enum class Action : uint8_t
{
  None,
  Start,
  Stop,
};

class TripTrigger
{
public:
  Action step(const Config& cfg, const Input& in)
  {
    if (cfg.mode != Mode::Auto)
    {
      reset();
      return Action::None;
    }
    if (owned_ && !in.recording)
    {
      // Our session is gone. After a Stop we sent that is the normal end; otherwise the user stopped it or the start failed:
      // stay quiet until the car has been off for kEndIdleMs.
      armed_ = stopSent_;
      owned_ = false;
      stopSent_ = false;
    }
    const bool running = in.busUp && (positive(in.rpm) || positive(in.speedKmh));
    if (running)
    {
      idle_ = false;
      if (!owned_ && !in.recording && armed_ && in.canStart)
      {
        owned_ = true;
        stopSent_ = false;
        return Action::Start;
      }
      return Action::None;
    }
    if (!idle_)
    {
      idle_      = true;
      idleSince_ = in.nowMs;
    }
    if (static_cast<uint32_t>(in.nowMs - idleSince_) < kEndIdleMs) { return Action::None; }
    if (owned_ && in.recording)
    {
      stopSent_ = true;
      return Action::Stop;
    }
    if (!owned_) { armed_ = true; }
    return Action::None;
  }

private:
  static bool positive(float v) { return std::isfinite(v) && v > 0.0f; }
  void reset()
  {
    owned_ = stopSent_ = idle_ = false;
    armed_ = true;
  }

  bool     owned_     = false;  // the last Start came from here and the session has not ended yet
  bool     stopSent_  = false;
  bool     armed_     = true;
  bool     idle_      = false;
  uint32_t idleSince_ = 0;
};
}  // namespace trip_trigger
