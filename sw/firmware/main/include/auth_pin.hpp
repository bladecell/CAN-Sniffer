#pragma once

// PIN policy, constant-time compare and the stored PIN record (P2.10c). No ESP-IDF includes, so host tests include it.
// The key derivation (PBKDF2-HMAC-SHA256) is injected, so the logic here runs on the host with a fake.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace auth
{
constexpr size_t   kPinDigits   = 6;
constexpr size_t   kSaltLen     = 16;
constexpr size_t   kHashLen     = 32;
constexpr uint32_t kMinIter     = 1000;     // a stored count outside [kMinIter, kMaxIter] is corrupt, never run
constexpr uint32_t kMaxIter     = 1000000;
constexpr size_t   kRecordSize  = 1 + 4 + kSaltLen + kHashLen;
constexpr uint8_t  kRecordVer   = 1;

// Exactly six ASCII digits, nothing else (no sign, space, NUL-padding or newline).
inline bool isValidPin(const char* pin, size_t len)
{
  if (pin == nullptr || len != kPinDigits)
  {
    return false;
  }
  for (size_t i = 0; i < len; i++)
  {
    if (pin[i] < '0' || pin[i] > '9')
    {
      return false;
    }
  }
  return true;
}

// Lower-case hex of n bytes into out[2n + 1].
inline void hexEncode(const uint8_t* in, size_t n, char* out)
{
  static const char hex[] = "0123456789abcdef";
  for (size_t i = 0; i < n; i++)
  {
    out[2 * i]     = hex[in[i] >> 4];
    out[2 * i + 1] = hex[in[i] & 0xF];
  }
  out[2 * n] = '\0';
}

// Compares n bytes without an early exit.
inline bool constantTimeEq(const uint8_t* a, const uint8_t* b, size_t n)
{
  volatile uint8_t diff = 0;
  for (size_t i = 0; i < n; i++)
  {
    diff = static_cast<uint8_t>(diff | (a[i] ^ b[i]));
  }
  return diff == 0;
}

struct PinRecord
{
  uint32_t iterations;
  uint8_t  salt[kSaltLen];
  uint8_t  hash[kHashLen];
};

// One NVS blob, so a power cut cannot leave a new salt next to an old hash: ver(1) | iterations(4, LE) | salt | hash.
inline void encodeRecord(const PinRecord& r, uint8_t out[kRecordSize])
{
  out[0] = kRecordVer;
  for (int i = 0; i < 4; i++)
  {
    out[1 + i] = static_cast<uint8_t>(r.iterations >> (8 * i));
  }
  memcpy(out + 5, r.salt, kSaltLen);
  memcpy(out + 5 + kSaltLen, r.hash, kHashLen);
}

// False for a wrong size, an unknown version or an iteration count outside the sane range.
inline bool decodeRecord(const uint8_t* buf, size_t len, PinRecord* r)
{
  if (buf == nullptr || r == nullptr || len != kRecordSize || buf[0] != kRecordVer)
  {
    return false;
  }
  uint32_t it = 0;
  for (int i = 0; i < 4; i++)
  {
    it |= static_cast<uint32_t>(buf[1 + i]) << (8 * i);
  }
  if (it < kMinIter || it > kMaxIter)
  {
    return false;
  }
  r->iterations = it;
  memcpy(r->salt, buf + 5, kSaltLen);
  memcpy(r->hash, buf + 5 + kSaltLen, kHashLen);
  return true;
}

// PBKDF2-HMAC-SHA256(pin, salt, iterations) into out[kHashLen]. False on any failure.
using Kdf = bool (*)(const uint8_t* pin, size_t pinLen, const uint8_t* salt, size_t saltLen, uint32_t iterations,
                     uint8_t* out);

inline bool verifyPin(const char* pin, size_t len, const PinRecord& r, Kdf kdf)
{
  uint8_t got[kHashLen];
  if (!isValidPin(pin, len) || kdf == nullptr ||
      !kdf(reinterpret_cast<const uint8_t*>(pin), len, r.salt, kSaltLen, r.iterations, got))
  {
    return false;
  }
  return constantTimeEq(got, r.hash, kHashLen);
}
}  // namespace auth
