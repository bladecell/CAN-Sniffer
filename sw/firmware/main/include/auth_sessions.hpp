#pragma once

// Bearer session table (P2.10c): RAM only, 4 sessions, 30 min idle and 12 h absolute expiry. Tokens come from an
// injected random source (the hardware RNG on the device). The lookup compares every slot in constant time. No ESP-IDF
// includes, so host tests drive it with a fake clock and RNG. Not thread-safe: the caller holds a lock.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "auth_pin.hpp"

namespace auth
{
class SessionTable
{
public:
  static constexpr size_t   kMax      = 4;
  static constexpr size_t   kTokenLen = 32;  // hex characters, 128 random bits
  static constexpr uint64_t kIdleMs   = 30ULL * 60 * 1000;
  static constexpr uint64_t kMaxAgeMs = 12ULL * 60 * 60 * 1000;

  using Rng = void (*)(uint8_t* buf, size_t len);

  // Opens a session and writes the NUL-terminated token to out[kTokenLen + 1]. When all slots are in use the one idle
  // longest is replaced (a forgotten browser tab must not lock the owner out). False when the RNG gives no usable token.
  bool create(uint64_t nowMs, Rng rng, char* out)
  {
    if (rng == nullptr || out == nullptr)
    {
      return false;
    }
    purge(nowMs);
    char tok[kTokenLen + 1];
    bool made = false;
    for (int attempt = 0; attempt < 3 && !made; attempt++)
    {
      uint8_t raw[kTokenLen / 2];
      rng(raw, sizeof(raw));
      uint8_t any = 0;
      for (uint8_t b : raw)
      {
        any = static_cast<uint8_t>(any | b);
      }
      if (any == 0)
      {
        continue;  // a dead RNG returning zeros must never produce a guessable token
      }
      static const char hex[] = "0123456789abcdef";
      for (size_t i = 0; i < sizeof(raw); i++)
      {
        tok[2 * i]     = hex[raw[i] >> 4];
        tok[2 * i + 1] = hex[raw[i] & 0xF];
      }
      tok[kTokenLen] = '\0';
      made           = find(tok) == nullptr;
    }
    if (!made)
    {
      return false;
    }
    Slot* s = freeSlot();
    s->used = true;
    memcpy(s->token, tok, sizeof(tok));
    s->createdMs = nowMs;
    s->lastMs    = nowMs;
    memcpy(out, tok, sizeof(tok));
    return true;
  }

  // True for a live session; refreshes its idle timer. An expired one is removed.
  bool validate(const char* token, uint64_t nowMs)
  {
    Slot* s = find(token);
    if (s == nullptr)
    {
      return false;
    }
    if (expired(*s, nowMs))
    {
      s->used = false;
      return false;
    }
    s->lastMs = nowMs;
    return true;
  }

  bool revoke(const char* token)
  {
    Slot* s = find(token);
    if (s == nullptr)
    {
      return false;
    }
    s->used = false;
    return true;
  }

  void clear()
  {
    for (Slot& s : slots_)
    {
      s.used = false;
      memset(s.token, 0, sizeof(s.token));
    }
  }

  size_t count(uint64_t nowMs)
  {
    purge(nowMs);
    size_t n = 0;
    for (const Slot& s : slots_)
    {
      n += s.used ? 1 : 0;
    }
    return n;
  }

private:
  struct Slot
  {
    bool     used = false;
    char     token[kTokenLen + 1] = {};
    uint64_t createdMs = 0;
    uint64_t lastMs    = 0;
  };

  static bool expired(const Slot& s, uint64_t nowMs)
  {
    const uint64_t idle = nowMs >= s.lastMs ? nowMs - s.lastMs : 0;
    const uint64_t age  = nowMs >= s.createdMs ? nowMs - s.createdMs : 0;
    return idle >= kIdleMs || age >= kMaxAgeMs;
  }

  void purge(uint64_t nowMs)
  {
    for (Slot& s : slots_)
    {
      if (s.used && expired(s, nowMs))
      {
        s.used = false;
      }
    }
  }

  // Compares against every slot so the time does not show which one (or how much of it) matched.
  Slot* find(const char* token)
  {
    if (token == nullptr || strlen(token) != kTokenLen)
    {
      return nullptr;
    }
    Slot* hit = nullptr;
    for (Slot& s : slots_)
    {
      const bool same = constantTimeEq(reinterpret_cast<const uint8_t*>(s.token),
                                       reinterpret_cast<const uint8_t*>(token), kTokenLen);
      if (same && s.used)
      {
        hit = &s;
      }
    }
    return hit;
  }

  Slot* freeSlot()
  {
    Slot* oldest = &slots_[0];
    for (Slot& s : slots_)
    {
      if (!s.used)
      {
        return &s;
      }
      if (s.lastMs < oldest->lastMs)
      {
        oldest = &s;
      }
    }
    return oldest;
  }

  Slot slots_[kMax];
};
}  // namespace auth
