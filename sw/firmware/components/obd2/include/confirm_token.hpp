// One-shot confirm token for destructive requests. Hardware-free: the clock and the random source are injected.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

class ConfirmToken
{
public:
  static constexpr size_t kTokenLen  = 16;
  static constexpr size_t kTokenSize = kTokenLen + 1;

  using Clock = uint64_t (*)();
  using Rng   = uint32_t (*)();

  ConfirmToken(Clock nowMs, Rng rng, uint32_t ttlMs) : nowMs_(nowMs), rng_(rng), ttlMs_(ttlMs) {}

  // Replaces any pending token with a fresh one and copies it, NUL-terminated, to out.
  void issue(char out[kTokenSize])
  {
    static const char hex[] = "0123456789abcdef";
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < kTokenLen; i += 8)
    {
      uint32_t r = rng_();
      for (size_t j = 0; j < 8; ++j)
      {
        token_[i + j] = hex[(r >> (28 - 4 * j)) & 0xF];
      }
    }
    token_[kTokenLen] = '\0';
    issuedMs_         = nowMs_();
    valid_            = true;
    std::memcpy(out, token_, kTokenSize);
  }

  // Answer to an unconfirmed call: the pending token while it is unexpired and the caller presented none,
  // otherwise a fresh one. Returns the ms left on the token copied to out.
  uint32_t challenge(const char* candidate, char out[kTokenSize])
  {
    if (candidate == nullptr || candidate[0] == '\0')
    {
      std::lock_guard<std::mutex> lock(mtx_);
      const uint64_t              age = nowMs_() - issuedMs_;
      if (valid_ && age < ttlMs_)
      {
        std::memcpy(out, token_, kTokenSize);
        return static_cast<uint32_t>(ttlMs_ - age);
      }
    }
    issue(out);
    return ttlMs_;
  }

  // True once for the pending, unexpired token. A mismatch leaves the pending token alone.
  bool consume(const char* candidate)
  {
    if (candidate == nullptr)
    {
      return false;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (!valid_ || std::strlen(candidate) != kTokenLen || std::memcmp(candidate, token_, kTokenLen) != 0)
    {
      return false;
    }
    valid_ = false;
    return nowMs_() - issuedMs_ < ttlMs_;
  }

  uint32_t ttlMs() const { return ttlMs_; }

private:
  Clock      nowMs_;
  Rng        rng_;
  uint32_t   ttlMs_;
  std::mutex mtx_;
  char       token_[kTokenSize] = {};
  uint64_t   issuedMs_          = 0;
  bool       valid_             = false;
};
