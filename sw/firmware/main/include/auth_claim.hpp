#pragma once

// Claim window (P2.10c): the first PIN may be set only within 60 s after a BOOT press made since the button driver
// started, and one press opens it once. Times are esp_timer microseconds; the button reports 0 for "no press".

#include <cstdint>

namespace auth
{
constexpr int64_t kClaimWindowUs = 60LL * 1000 * 1000;

class ClaimGate
{
public:
  // True when `lastPressUs` is a real press, newer than the one already used, and at most 60 s old.
  bool open(int64_t lastPressUs, int64_t nowUs) const
  {
    return lastPressUs > 0 && lastPressUs != usedPressUs_ && nowUs >= lastPressUs && nowUs - lastPressUs <= kClaimWindowUs;
  }

  // Seconds left in the window (rounded up), 0 when closed.
  uint32_t remainingS(int64_t lastPressUs, int64_t nowUs) const
  {
    if (!open(lastPressUs, nowUs))
    {
      return 0;
    }
    return static_cast<uint32_t>((kClaimWindowUs - (nowUs - lastPressUs) + 999999) / 1000000);
  }

  // A claim succeeded: this press cannot open the window again.
  void consume(int64_t lastPressUs) { usedPressUs_ = lastPressUs; }

private:
  int64_t usedPressUs_ = 0;
};
}  // namespace auth
