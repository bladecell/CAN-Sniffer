#pragma once

// Host/Origin guard and Content-Type policy (P2.10b). No ESP-IDF includes, so host tests include it directly.
//
// Why: the API is plain HTTP on a local network. A web page the user visits can send requests to the device
// (cross-site forgery) or re-point its own hostname at the device's IP (DNS rebinding). Browsers always send Host and, for
// cross-site requests and WebSocket upgrades, Origin. Accepting only our own names closes both. Clients that send no
// Origin (curl, scripts, same-origin GET) are not browsers acting for a foreign page and pass the Origin test.

#include <cstddef>
#include <initializer_list>
#include <cstring>

namespace host_guard
{
constexpr const char* kMdnsHost = "can-sniffer.local";

// Header buffer sizes the caller reads into. A longer header is rejected without being parsed.
constexpr size_t kHostMax        = 64;
constexpr size_t kOriginMax      = 96;
constexpr size_t kContentTypeMax = 64;

// The device's current addresses as dotted strings; null, empty or "0.0.0.0" (no lease yet) means "not set".
struct Ips
{
  const char* ap;
  const char* sta;
};

enum class Verdict
{
  Ok,
  BadHost,
  BadOrigin,
};

enum class Body
{
  Json,   // application/json, optional "; charset=utf-8"
  Octet,  // application/octet-stream, no parameters
};

namespace detail
{
inline char lower(char c)
{
  return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + ('a' - 'A')) : c;
}

inline bool eqNoCase(const char* a, size_t n, const char* b)
{
  if (strlen(b) != n)
  {
    return false;
  }
  for (size_t i = 0; i < n; i++)
  {
    if (lower(a[i]) != lower(b[i]))
    {
      return false;
    }
  }
  return true;
}

inline bool ipSet(const char* ip)
{
  return ip != nullptr && *ip != '\0' && strcmp(ip, "0.0.0.0") != 0;
}

// `s` is "host" or "host:80" with nothing else around it.
inline bool hostPortAllowed(const char* s, size_t len, const Ips& ips)
{
  if (len > 3 && s[len - 3] == ':' && s[len - 2] == '8' && s[len - 1] == '0')
  {
    len -= 3;  // the server listens on port 80 only; any other explicit port is a different origin
  }
  if (len == 0 || len >= kHostMax)
  {
    return false;
  }
  if (eqNoCase(s, len, kMdnsHost))
  {
    return true;
  }
  for (const char* ip : {ips.ap, ips.sta})
  {
    if (ipSet(ip) && strlen(ip) == len && memcmp(ip, s, len) == 0)
    {
      return true;
    }
  }
  return false;
}
}  // namespace detail

// Host header value. Missing (null) is refused: HTTP/1.1 requires it.
inline bool hostAllowed(const char* host, const Ips& ips)
{
  return host != nullptr && detail::hostPortAllowed(host, strlen(host), ips);
}

// Origin header value, only called when the header is present: "http://" + an allowed host. "null", https, a path or
// anything else is refused.
inline bool originAllowed(const char* origin, const Ips& ips)
{
  static const char kScheme[] = "http://";
  if (origin == nullptr || strncmp(origin, kScheme, sizeof(kScheme) - 1) != 0)
  {
    return false;
  }
  const char* rest = origin + sizeof(kScheme) - 1;
  return detail::hostPortAllowed(rest, strlen(rest), ips);
}

// Host first, then Origin (only when present).
inline Verdict check(const char* host, const char* origin, const Ips& ips)
{
  if (!hostAllowed(host, ips))
  {
    return Verdict::BadHost;
  }
  if (origin != nullptr && !originAllowed(origin, ips))
  {
    return Verdict::BadOrigin;
  }
  return Verdict::Ok;
}

// Content-Type of a request that carries a body. Null (header absent) is refused.
inline bool contentTypeOk(const char* ct, Body kind)
{
  if (ct == nullptr)
  {
    return false;
  }
  auto isWs = [](char c) { return c == ' ' || c == '\t'; };
  size_t typeEnd = 0;
  while (ct[typeEnd] != '\0' && ct[typeEnd] != ';')
  {
    typeEnd++;
  }
  size_t typeLen = typeEnd;
  while (typeLen > 0 && isWs(ct[typeLen - 1]))
  {
    typeLen--;
  }
  const char* type = ct;
  while (typeLen > 0 && isWs(*type))
  {
    type++;
    typeLen--;
  }
  const char* want = kind == Body::Json ? "application/json" : "application/octet-stream";
  if (!detail::eqNoCase(type, typeLen, want))
  {
    return false;
  }
  if (ct[typeEnd] == '\0')
  {
    return true;
  }
  if (kind == Body::Octet)
  {
    return false;
  }
  const char* p = ct + typeEnd + 1;  // after ';'
  while (isWs(*p))
  {
    p++;
  }
  size_t n = strlen(p);
  while (n > 0 && isWs(p[n - 1]))
  {
    n--;
  }
  return detail::eqNoCase(p, n, "charset=utf-8");
}
}  // namespace host_guard
