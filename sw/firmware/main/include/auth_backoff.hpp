#pragma once

// Global failed-login counter with exponential delay (P2.10c). One counter for the whole device, not per client: on an
// open Wi-Fi AP a per-client key (an IP or MAC) is free to change. It blocks login only, never an existing session.
// Time is a caller-supplied millisecond clock, so host tests drive it directly.

#include <cstdint>

namespace auth
{
class LoginBackoff
{
public:
  static constexpr uint32_t kCapS = 900;  // longest wait; recovery by BOOT hold exists, so no lock-out beyond this

  // Wait after the Nth consecutive failure: 1, 2, 4, ... s, capped. 0 failures: no wait.
  static uint32_t delayS(uint32_t failures)
  {
    if (failures == 0)
    {
      return 0;
    }
    return failures - 1 >= 10 ? kCapS : ((1u << (failures - 1)) < kCapS ? (1u << (failures - 1)) : kCapS);
  }

  // Seconds the caller must still wait (rounded up, at least 1 while blocked); 0 means a login may be tried now.
  uint32_t retryAfterS(uint64_t nowMs) const
  {
    if (nowMs >= blockedUntilMs_)
    {
      return 0;
    }
    return static_cast<uint32_t>((blockedUntilMs_ - nowMs + 999) / 1000);
  }

  // Call only for an attempt that was actually checked (not for one refused while blocked).
  void onFailure(uint64_t nowMs)
  {
    if (failures_ < 32)
    {
      failures_++;
    }
    blockedUntilMs_ = nowMs + static_cast<uint64_t>(delayS(failures_)) * 1000;
  }

  void onSuccess() { reset(); }
  void reset()
  {
    failures_       = 0;
    blockedUntilMs_ = 0;
  }
  uint32_t failures() const { return failures_; }

private:
  uint32_t failures_       = 0;
  uint64_t blockedUntilMs_ = 0;
};
}  // namespace auth
