#pragma once

// Hardware-free validation of the persisted Wi-Fi, CAN and system settings (P2.9a, SET-1).
// No ESP-IDF includes, so host tests include it directly. Enums arrive as plain ints: validate first, cast afterwards.
// Every check returns nullptr when the value is fine, else a short reason for the REST error body.
// Values are rejected, never clamped.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace settings_validation
{
constexpr size_t kSsidMax     = 32;
constexpr size_t kPassMin     = 8;
constexpr size_t kPassMax     = 63;
constexpr size_t kWifiStrBuf  = 64;   // NVS read buffer for Wi-Fi strings: kPassMax + NUL fits
constexpr size_t kPathMax     = 255;
constexpr size_t kPathBuf     = 256;  // NVS read buffer for the system paths: kPathMax + NUL fits
constexpr int    kChannelMin  = 1;
constexpr int    kChannelMax  = 13;
constexpr int    kMaxConnMax  = 10;  // ESP32-S3 soft-AP limit

constexpr std::string_view kSdPrefix = "/sdcard/";

// A JSON number as an integer in [lo, hi]. The double -> integer cast of NaN, inf or an out-of-range value is undefined.
inline bool checkedInt(double v, double lo, double hi, int64_t& out)
{
  if (!std::isfinite(v) || v < lo || v > hi || v != std::floor(v))
  {
    return false;
  }
  out = static_cast<int64_t>(v);
  return true;
}

// --- Wi-Fi (numbers match wifi_mode_t / wifi_auth_mode_t) ---
inline bool wifiModeValid(int64_t m) { return m >= 1 && m <= 3; }  // STA, AP, APSTA
inline bool wifiAuthValid(int64_t a) { return a == 0 || a == 2 || a == 3 || a == 4 || a == 6 || a == 7; }  // OPEN, WPA, WPA2, WPA/WPA2, WPA3, WPA2/WPA3
inline bool channelValid(int64_t c) { return c >= kChannelMin && c <= kChannelMax; }
inline bool maxConnValid(int64_t n) { return n >= 1 && n <= kMaxConnMax; }
inline bool ssidValid(std::string_view s, bool allowEmpty) { return s.size() <= kSsidMax && (allowEmpty || !s.empty()); }
inline bool passwordValid(std::string_view p) { return p.empty() || (p.size() >= kPassMin && p.size() <= kPassMax); }

inline const char* wifiCheck(std::string_view ssid, std::string_view pass, int64_t auth, int64_t mode, int64_t channel,
                             int64_t maxConn, std::string_view staSsid, std::string_view staPass, int64_t staAuth)
{
  if (!ssidValid(ssid, false))
  {
    return "ssid must be 1..32 bytes";
  }
  if (!passwordValid(pass))
  {
    return "password must be empty or 8..63 bytes";
  }
  if (!ssidValid(staSsid, true))
  {
    return "sta_ssid must be at most 32 bytes";
  }
  if (!passwordValid(staPass))
  {
    return "sta_password must be empty or 8..63 bytes";
  }
  if (!wifiModeValid(mode))
  {
    return "mode must be 1 (STA), 2 (AP) or 3 (APSTA)";
  }
  if (!wifiAuthValid(auth))
  {
    return "auth_mode must be 0, 2, 3, 4, 6 or 7";
  }
  if (!wifiAuthValid(staAuth))
  {
    return "sta_auth_mode must be 0, 2, 3, 4, 6 or 7";
  }
  if (!channelValid(channel))
  {
    return "channel must be 1..13";
  }
  if (!maxConnValid(maxConn))
  {
    return "max_connections must be 1..10";
  }
  if (auth != 0 && pass.empty())
  {
    return "password required when auth_mode is not open";
  }
  if (auth == 0 && !pass.empty())
  {
    return "password must be empty when auth_mode is open";
  }
  return nullptr;
}

// --- CAN ---
inline bool bitrateValid(int64_t b) { return b == 125000 || b == 250000 || b == 500000 || b == 1000000; }
inline bool rsModeValid(int64_t m) { return m == 0 || m == 1; }

// ESP32-S3 pins the CAN transceiver may use. Excluded: 0 (BOOT), 19/20 (USB), 22-25 (absent), 26-37 (flash, PSRAM),
// 43/44 (console UART0), 46 (input only) and anything else out of range.
inline bool gpioUsable(int64_t p) { return (p >= 1 && p <= 18) || p == 21 || (p >= 38 && p <= 42) || p == 47 || p == 48; }

// tx, rx, lbk and rs must be usable, distinct, and not one of the board pins in `reserved` (SD, LED, ...).
inline const char* canPinsCheck(int64_t tx, int64_t rx, int64_t lbk, int64_t rs, const int* reserved, size_t reservedCount)
{
  const int64_t pins[4] = {tx, rx, lbk, rs};
  for (int64_t p : pins)
  {
    if (!gpioUsable(p))
    {
      return "CAN pin is not a usable GPIO (flash, PSRAM, USB, console, BOOT or out of range)";
    }
    for (size_t i = 0; reserved != nullptr && i < reservedCount; i++)
    {
      if (p == reserved[i])
      {
        return "CAN pin is used by the board (SD card or LED)";
      }
    }
  }
  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = i + 1; j < 4; j++)
    {
      if (pins[i] == pins[j])
      {
        return "CAN pins must be distinct";
      }
    }
  }
  return nullptr;
}

inline bool filterValid(uint32_t id, uint32_t mask, bool isExt)
{
  const uint32_t max = isExt ? 0x1FFFFFFFu : 0x7FFu;
  return id <= max && mask <= max;
}

// --- System paths ---
inline bool sdPathValid(std::string_view p)
{
  if (p.size() <= kSdPrefix.size() || p.size() > kPathMax || p.substr(0, kSdPrefix.size()) != kSdPrefix)
  {
    return false;
  }
  for (char c : p)
  {
    if (static_cast<unsigned char>(c) < 0x20 || c == 0x7F)
    {
      return false;
    }
  }
  // Only a ".." path component climbs out; "a..b" is a legal name.
  size_t start = 0;
  while (start <= p.size())
  {
    size_t end = p.find('/', start);
    if (end == std::string_view::npos)
    {
      end = p.size();
    }
    if (p.substr(start, end - start) == "..")
    {
      return false;
    }
    start = end + 1;
  }
  return true;
}
}  // namespace settings_validation
