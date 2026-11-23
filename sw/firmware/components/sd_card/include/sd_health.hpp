#pragma once

// SD card health (hardware-free, host-tested). A card that hangs makes every transfer wait for its driver timeout while
// the SD lease is held, so the web UI stalls. After kTimeoutLimit consecutive I/O timeouts the card is marked
// unavailable and unmounted; acquire_operation() then refuses at once. Mount, removal and trip are published as a state
// change that the supervisor loop turns into WebSocket frame 0x06 (docs/api.md).

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace sd_health
{
constexpr int      kTimeoutLimit   = 3;        // consecutive I/O timeouts before the card is declared unavailable
constexpr int64_t  kOpDeadlineUs   = 3000000;  // total time budget of one atomic save or copy
constexpr uint32_t kRetryBackoffMs = 60000;    // between remount attempts while unavailable and the card is still inserted

constexpr uint8_t kMsgTypeSd        = 0x06;
constexpr size_t  kStateFrameBytes  = 5;

enum class Reason : uint8_t
{
  Mounted    = 0,  // the card is mounted
  Removed    = 1,  // no card in the slot
  IoTimeouts = 2,  // unmounted after kTimeoutLimit consecutive I/O timeouts
  Unmounted  = 3,  // unmounted for another reason (format, sleep, mount failure)
};

struct State
{
  bool   mounted     = false;
  bool   unavailable = false;
  Reason reason      = Reason::Unmounted;
};

// FATFS reports a card that stops answering as EIO (driver timeout) or ETIMEDOUT; ENOSPC and the rest stay plain failures.
inline esp_err_t ioError(int err)
{
  return (err == EIO || err == ETIMEDOUT) ? ESP_ERR_TIMEOUT : ESP_FAIL;
}

// 0x06 frame: [0]=0x06, [1]=3 (payload length), [2] mounted, [3] unavailable, [4] reason. Returns bytes written, 0 if `cap` is too small.
inline size_t encodeState(const State& s, uint8_t* out, size_t cap)
{
  if (out == nullptr || cap < kStateFrameBytes) { return 0; }
  out[0] = kMsgTypeSd;
  out[1] = kStateFrameBytes - 2;
  out[2] = s.mounted ? 1 : 0;
  out[3] = s.unavailable ? 1 : 0;
  out[4] = static_cast<uint8_t>(s.reason);
  return kStateFrameBytes;
}

// Writers run under the SD lease (or the supervisor loop); takeChange() has one consumer (the supervisor loop).
class Tracker
{
public:
  // ESP_OK clears the count, ESP_ERR_TIMEOUT counts, anything else is neither. True once, on the call that trips the card.
  bool reportIo(esp_err_t err)
  {
    if (err == ESP_OK) { timeouts_.store(0); return false; }
    if (err != ESP_ERR_TIMEOUT || unavailable_.load()) { return false; }
    if (timeouts_.fetch_add(1) + 1 < kTimeoutLimit) { return false; }
    unavailable_.store(true);
    retryArmed_.store(false);
    publish({false, true, Reason::IoTimeouts});
    return true;
  }

  void noteMounted()
  {
    timeouts_.store(0);
    unavailable_.store(false);
    publish({true, false, Reason::Mounted});
  }

  // Keeps the IoTimeouts reason while the card is unavailable (the trip is followed by its unmount).
  void noteUnmounted(Reason why)
  {
    if (unavailable_.load()) { publish({false, true, Reason::IoTimeouts}); }
    else { publish({false, false, why}); }
  }

  // The driver was shut down on purpose (deinit before sleep): nothing is unavailable any more.
  void noteDeinit()
  {
    timeouts_.store(0);
    unavailable_.store(false);
    publish({false, false, Reason::Unmounted});
  }

  void noteRemoved()
  {
    timeouts_.store(0);
    unavailable_.store(false);
    publish({false, false, Reason::Removed});
  }

  bool  unavailable() const { return unavailable_.load(); }
  State state() const { return unpack(packed_.load()); }

  // True when the state changed since the last call; `out` is the current state.
  bool takeChange(State& out)
  {
    const uint32_t now = seq_.load();
    if (now == seen_) { return false; }
    seen_ = now;
    out   = state();
    return true;
  }

  // One remount attempt per kRetryBackoffMs while unavailable. The first call after the trip only arms the timer.
  bool retryDue(uint32_t nowMs)
  {
    if (!unavailable_.load()) { return false; }
    if (!retryArmed_.exchange(true)) { lastTryMs_ = nowMs; return false; }
    if (static_cast<uint32_t>(nowMs - lastTryMs_) < kRetryBackoffMs) { return false; }
    lastTryMs_ = nowMs;
    return true;
  }

private:
  static uint32_t pack(const State& s)
  {
    return (s.mounted ? 1u : 0u) | (s.unavailable ? 2u : 0u) | (static_cast<uint32_t>(s.reason) << 2);
  }
  static State unpack(uint32_t v)
  {
    return {(v & 1u) != 0, (v & 2u) != 0, static_cast<Reason>((v >> 2) & 3u)};
  }
  void publish(const State& s)
  {
    const uint32_t v = pack(s);
    if (packed_.exchange(v) != v) { seq_.fetch_add(1); }
  }

  std::atomic<int>      timeouts_{0};
  std::atomic<bool>     unavailable_{false};
  std::atomic<bool>     retryArmed_{false};
  std::atomic<uint32_t> packed_{pack(State{})};
  std::atomic<uint32_t> seq_{0};
  uint32_t              seen_      = 0;
  uint32_t              lastTryMs_ = 0;
};
}  // namespace sd_health
