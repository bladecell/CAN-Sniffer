// Recorder pure core: no ESP-IDF, no FreeRTOS, no allocation. Host-tested (test_recorder_core.cpp).
#pragma once
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace recorder_core
{
constexpr size_t   kMaxRowBytes   = 40;    // longest CSV row with its newline
// Flush policy, tuned here only. A power cut loses at most kFlushMs of rows (docs/sd-card.md). Fewer, larger writes
// keep the card out of the open/append/close churn that wedged it at 2 s / 4 KB.
constexpr size_t   kBatchBytes    = 8192;  // line buffer size and size flush trigger
constexpr uint32_t kFlushMs       = 10000;
constexpr uint64_t kRotateBytes   = 8ull * 1024 * 1024;
constexpr uint32_t kMaxTripIndex  = 999999;
constexpr size_t   kMaxPids       = 32;
constexpr uint16_t kMaxSpacingMs  = 60000;
constexpr char     kCsvHeader[]   = "t_ms,pid,value,valid\n";
constexpr char     kTripDir[]     = "/logs/trips";

enum class State : uint8_t
{
  Idle,
  Recording,
  Stopping,
  Error,
};

constexpr const char* stateName(State s)
{
  return s == State::Idle ? "idle" : s == State::Recording ? "recording" : s == State::Stopping ? "stopping" : "error";
}

constexpr bool canTransition(State from, State to)
{
  switch (from)
  {
    case State::Idle: return to == State::Recording || to == State::Error;
    case State::Recording: return to == State::Stopping || to == State::Error;
    case State::Stopping: return to == State::Idle;
    case State::Error: return to == State::Recording || to == State::Idle || to == State::Stopping;
  }
  return false;
}

class StateMachine
{
public:
  State get() const { return s_.load(); }
  // Atomic: succeeds only if the state is `from` and from -> to is legal.
  bool move(State from, State to) { return canTransition(from, to) && s_.compare_exchange_strong(from, to); }

private:
  std::atomic<State> s_{State::Idle};
};

// Lets stop() close the door on the OBD callback and wait for a callback already running. enter() increments before it
// checks, close() clears before idle() is read, so a callback is either refused or counted.
class CallbackGate
{
public:
  void open() { open_.store(true); }
  void close() { open_.store(false); }
  bool enter()
  {
    inFlight_.fetch_add(1);
    if (open_.load()) { return true; }
    inFlight_.fetch_sub(1);
    return false;
  }
  void leave() { inFlight_.fetch_sub(1); }
  bool idle() const { return inFlight_.load() == 0; }

private:
  std::atomic<bool>     open_{false};
  std::atomic<uint32_t> inFlight_{0};
};

struct Row
{
  uint32_t tMs;
  uint16_t pid;
  float    value;
  bool     valid;
};

// "t_ms,pid,value,valid\n". Invalid or non-finite values give an empty value and valid=0.
// Returns the byte count (the NUL is not counted), or 0 when `cap` cannot hold the row and its NUL.
inline size_t formatRow(char* out, size_t cap, const Row& r)
{
  if (out == nullptr || cap == 0) { return 0; }
  const bool ok = r.valid && std::isfinite(r.value);
  const int  n  = ok ? snprintf(out, cap, "%lu,%u,%.8g,1\n", static_cast<unsigned long>(r.tMs), static_cast<unsigned>(r.pid),
                                static_cast<double>(r.value))
                     : snprintf(out, cap, "%lu,%u,,0\n", static_cast<unsigned long>(r.tMs), static_cast<unsigned>(r.pid));
  if (n < 0 || static_cast<size_t>(n) >= cap)
  {
    out[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(n);
}

// trip_NNNNNN.csv or .json
inline bool formatTripName(char* out, size_t cap, uint32_t index, bool sidecar)
{
  if (out == nullptr || index > kMaxTripIndex) { return false; }
  const int n = snprintf(out, cap, "trip_%06lu.%s", static_cast<unsigned long>(index), sidecar ? "json" : "csv");
  return n > 0 && static_cast<size_t>(n) < cap;
}

inline bool parseTripIndex(const char* name, uint32_t& index)
{
  if (name == nullptr || std::strncmp(name, "trip_", 5) != 0) { return false; }
  uint32_t v = 0;
  for (size_t i = 5; i < 11; ++i)
  {
    if (name[i] < '0' || name[i] > '9') { return false; }
    v = v * 10 + static_cast<uint32_t>(name[i] - '0');
  }
  if (std::strcmp(name + 11, ".csv") != 0 && std::strcmp(name + 11, ".json") != 0) { return false; }
  index = v;
  return true;
}

// Feed every directory entry; next() is the index for a new trip. Sidecars count so an orphan never gets reused.
class TripScanner
{
public:
  void add(const char* name)
  {
    uint32_t i = 0;
    if (parseTripIndex(name, i) && i > max_) { max_ = i; }
  }
  bool next(uint32_t& index) const
  {
    if (max_ >= kMaxTripIndex) { return false; }
    index = max_ + 1;
    return true;
  }

private:
  uint32_t max_ = 0;
};

constexpr size_t kMaxListedTrips = 100;

// The newest kMaxListedTrips trip CSVs of a directory scan, highest index first. Sidecars and other names are ignored.
class NewestTrips
{
public:
  void addName(const char* name)
  {
    uint32_t i = 0;
    if (!parseTripIndex(name, i) || std::strcmp(name + 11, ".csv") != 0) { return; }
    ++total_;
    size_t at = n_;
    while (at > 0 && idx_[at - 1] < i) { --at; }
    if (at >= kMaxListedTrips) { return; }
    const size_t last = n_ < kMaxListedTrips ? n_ : kMaxListedTrips - 1;
    for (size_t k = last; k > at; --k) { idx_[k] = idx_[k - 1]; }
    idx_[at] = i;
    if (n_ < kMaxListedTrips) { ++n_; }
  }
  size_t   count() const { return n_; }
  uint32_t at(size_t i) const { return i < n_ ? idx_[i] : 0; }
  bool     truncated() const { return total_ > n_; }

private:
  uint32_t idx_[kMaxListedTrips]{};
  size_t   n_     = 0;
  size_t   total_ = 0;
};

struct PidSel
{
  uint16_t pid;
  uint16_t minSpacingMs;  // 0 keeps every update
};

enum class ListError : uint8_t
{
  Ok,
  Empty,
  TooMany,
  Duplicate,
  BadSpacing,
  Unknown,
};

// `exists(pid)` says whether the PID has a definition.
template <class Exists>
ListError validatePidList(const PidSel* in, size_t n, Exists exists)
{
  if (in == nullptr || n == 0) { return ListError::Empty; }
  if (n > kMaxPids) { return ListError::TooMany; }
  for (size_t i = 0; i < n; ++i)
  {
    if (in[i].minSpacingMs > kMaxSpacingMs) { return ListError::BadSpacing; }
    if (!exists(in[i].pid)) { return ListError::Unknown; }
    for (size_t j = 0; j < i; ++j)
    {
      if (in[j].pid == in[i].pid) { return ListError::Duplicate; }
    }
  }
  return ListError::Ok;
}

// Selected PIDs plus per-PID minimum spacing. One producer thread calls accept().
class PidFilter
{
public:
  bool set(const PidSel* in, size_t n)
  {
    if (in == nullptr || n > kMaxPids) { return false; }
    n_ = n;
    for (size_t i = 0; i < n; ++i)
    {
      sel_[i]  = in[i];
      seen_[i] = false;
    }
    return true;
  }
  size_t size() const { return n_; }
  const PidSel& at(size_t i) const { return sel_[i < n_ ? i : 0]; }
  bool accept(uint16_t pid, uint32_t tMs)
  {
    for (size_t i = 0; i < n_; ++i)
    {
      if (sel_[i].pid != pid) { continue; }
      if (seen_[i] && (tMs == last_[i] || static_cast<uint32_t>(tMs - last_[i]) < sel_[i].minSpacingMs)) { return false; }
      seen_[i] = true;
      last_[i] = tMs;
      return true;
    }
    return false;
  }

private:
  PidSel   sel_[kMaxPids]{};
  uint32_t last_[kMaxPids]{};
  bool     seen_[kMaxPids]{};
  size_t   n_ = 0;
};

constexpr bool shouldFlush(size_t pendingBytes, uint32_t nowMs, uint32_t lastFlushMs)
{
  return pendingBytes > 0 && (pendingBytes + kMaxRowBytes > kBatchBytes || static_cast<uint32_t>(nowMs - lastFlushMs) >= kFlushMs);
}

// A flush that failed is retried at most every kFlushMs, so a missing card is not hammered (or waited on while stopping).
constexpr bool mayFlush(bool degraded, uint32_t nowMs, uint32_t lastFlushMs)
{
  return !degraded || static_cast<uint32_t>(nowMs - lastFlushMs) >= kFlushMs;
}

constexpr bool shouldRotate(uint64_t fileBytes, size_t pendingBytes)
{
  return fileBytes + pendingBytes >= kRotateBytes;
}

// FNV-1a; pass the previous result as `seed` to chain.
constexpr uint32_t fnv1a32(const char* data, size_t n, uint32_t seed = 0x811C9DC5u)
{
  uint32_t h = seed;
  for (size_t i = 0; i < n; ++i)
  {
    h ^= static_cast<uint8_t>(data[i]);
    h *= 16777619u;
  }
  return h;
}
}  // namespace recorder_core
