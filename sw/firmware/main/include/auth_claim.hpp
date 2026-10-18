#pragma once

// Claim window (P2.10c/d): a parked PIN is committed by a BOOT press made after the post and at most 60 s after it.
// Times are esp_timer microseconds; the button reports 0 for "no press".

#include <cstdint>

namespace auth
{
constexpr int64_t kClaimWindowUs = 60LL * 1000 * 1000;
}  // namespace auth
