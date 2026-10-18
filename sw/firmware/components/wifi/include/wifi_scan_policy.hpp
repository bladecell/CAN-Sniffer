#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>

namespace wifi_scan
{
constexpr size_t kMaxResults = 20;
constexpr int64_t kDeadlineUs = 5000000;
constexpr int64_t kCooldownUs = 10000000;
constexpr int64_t kCacheUs = 30000000;

// The owner must drain an aborted scan before arming again; this gate rejects duplicate/closed-epoch events only.
class CompletionGate
{
public:
  void arm() { armed.store(true); }
  void close() { armed.store(false); }
  bool accept() { return armed.exchange(false); }
private:
  std::atomic<bool> armed{false};
};

inline bool mayReconnect(bool apOnly, bool closing, bool scanning)
{
  return !apOnly && !closing && !scanning;
}

enum class Admission { Ready, Busy, Down, Cooldown };

inline Admission admit(bool running, bool active, bool fault, int64_t now, int64_t lastStart)
{
  if (!running || fault) { return Admission::Down; }
  if (active) { return Admission::Busy; }
  if (lastStart >= 0 && now - lastStart < kCooldownUs) { return Admission::Cooldown; }
  return Admission::Ready;
}

inline bool expired(int64_t now, int64_t completed) { return now - completed >= kCacheUs; }
inline bool deadlineReached(int64_t now, int64_t started) { return now - started >= kDeadlineUs; }

// SSIDs are bytes, not necessarily UTF-8. Keep exact bytes in hex; display replaces malformed UTF-8/control bytes.
inline void encodeSsid(const uint8_t* input, size_t capacity, char (&text)[33], char (&hex)[65])
{
  constexpr char digits[] = "0123456789abcdef";
  size_t n = 0;
  while (n < capacity && n < 32 && input[n] != 0) { ++n; }
  for (size_t i = 0; i < n; ++i)
  {
    hex[2 * i] = digits[input[i] >> 4];
    hex[2 * i + 1] = digits[input[i] & 15];
  }
  hex[2 * n] = 0;
  size_t out = 0;
  for (size_t i = 0; i < n;)
  {
    const uint8_t c = input[i];
    size_t len = c < 0x80 ? 1 : c >= 0xc2 && c <= 0xdf ? 2 : c >= 0xe0 && c <= 0xef ? 3 :
                 c >= 0xf0 && c <= 0xf4 ? 4 : 0;
    bool valid = len != 0 && len <= n - i && c >= 0x20 && c != 0x7f;
    for (size_t j = 1; valid && j < len; ++j) { valid = (input[i + j] & 0xc0) == 0x80; }
    if (valid && len >= 3)
    {
      valid = !(c == 0xe0 && input[i + 1] < 0xa0) && !(c == 0xed && input[i + 1] >= 0xa0) &&
              !(c == 0xf0 && input[i + 1] < 0x90) && !(c == 0xf4 && input[i + 1] >= 0x90);
    }
    if (!valid) { text[out++] = '?'; ++i; }
    else { for (size_t j = 0; j < len; ++j) { text[out++] = static_cast<char>(input[i++]); } }
  }
  text[out] = 0;
}
}
