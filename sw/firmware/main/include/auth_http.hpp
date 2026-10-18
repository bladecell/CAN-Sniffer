#pragma once

// Mapping from auth results to HTTP, and the Authorization header parser (P2.10c). No ESP-IDF includes.

#include <cstddef>

#include "auth_core.hpp"

namespace auth
{
struct Http
{
  int         status;
  const char* code;    // stable client token (docs/api.md, Authentication)
  const char* reason;  // for people
};

constexpr size_t kAuthHeaderMax = 80;  // "Bearer " + 32 hex + slack; a longer header is refused unread

inline Http httpFor(Result r)
{
  switch (r)
  {
    case Result::Ok: return {200, "OK", "ok"};
    case Result::Unavailable: return {503, "AUTH_UNAVAILABLE", "Access control is not available"};
    case Result::NotClaimed: return {403, "NOT_CLAIMED", "Device has no PIN yet, claim it first"};
    case Result::AlreadyClaimed: return {409, "ALREADY_CLAIMED", "Device already has a PIN"};
    case Result::ButtonRequired: return {403, "BUTTON_REQUIRED", "Press the BOOT button on the device, then retry within 60 s"};
    case Result::BadFormat: return {422, "BAD_PIN_FORMAT", "PIN must be exactly 6 digits"};
    case Result::BadPin: return {401, "BAD_PIN", "Wrong PIN"};
    case Result::Backoff: return {429, "LOGIN_BACKOFF", "Too many wrong PINs, wait before retrying"};
    case Result::Unauthorized: return {401, "UNAUTHORIZED", "Login required"};
    default: return {500, "AUTH_INTERNAL", "Access control failed"};
  }
}

// "Bearer <token>" (scheme case-insensitive, one space, token without spaces or control bytes) -> token in out.
// False for anything else, including a token that does not fit.
inline bool bearerToken(const char* header, char* out, size_t cap)
{
  static const char kScheme[] = "bearer ";
  if (header == nullptr || out == nullptr || cap == 0)
  {
    return false;
  }
  for (size_t i = 0; i < sizeof(kScheme) - 1; i++)
  {
    const char c = header[i];
    if (c == '\0' || (c >= 'A' && c <= 'Z' ? static_cast<char>(c + 32) : c) != kScheme[i])
    {
      return false;
    }
  }
  const char* tok = header + sizeof(kScheme) - 1;
  size_t      n   = 0;
  while (tok[n] != '\0')
  {
    if (static_cast<unsigned char>(tok[n]) <= ' ' || static_cast<unsigned char>(tok[n]) >= 0x7F || n + 1 >= cap)
    {
      return false;
    }
    n++;
  }
  if (n == 0)
  {
    return false;
  }
  for (size_t i = 0; i < n; i++)
  {
    out[i] = tok[i];
  }
  out[n] = '\0';
  return true;
}
}  // namespace auth
