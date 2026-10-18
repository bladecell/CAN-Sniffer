#pragma once

// One error contract for every HTTP layer (P2.11 NB2, B22). No ESP-IDF includes, so host tests include it directly.
//
//   Content-Type: application/json
//   {"status":"error","reason":"<human text>","code":"<STABLE_TOKEN>"}
//
// `reason` is for people, `code` is for clients and never changes wording. Used by the async layer (429, 503), the
// handlers in webserver.cpp and the middleware result path. AsyncWebServer::sendError() puts it on the wire; auth
// (401, 403, 429 with its own Retry-After) goes through the same call.

#include <cstddef>
#include <cstdint>

namespace http_error
{
// Stable codes that are not just the status default. Middleware also uses esp_err_to_name() strings as codes.
constexpr const char* kBadJson      = "BAD_JSON";
constexpr const char* kBadPath      = "BAD_PATH";
constexpr const char* kShuttingDown = "SHUTTING_DOWN";
constexpr const char* kRateLimited  = "RATE_LIMITED";
constexpr const char* kBusy         = "BUSY";

// Largest body buildBody() can produce for the reason lengths we send; the reason is cut to fit.
constexpr size_t kBodyMax = 320;

// "404 Not Found" for the statuses this API sends (success and error), nullptr for anything else.
inline const char* statusLine(int status)
{
  switch (status)
  {
    case 200: return "200 OK";
    case 201: return "201 Created";
    case 202: return "202 Accepted";
    case 204: return "204 No Content";
    case 400: return "400 Bad Request";
    case 401: return "401 Unauthorized";
    case 403: return "403 Forbidden";
    case 404: return "404 Not Found";
    case 405: return "405 Method Not Allowed";
    case 408: return "408 Request Timeout";
    case 409: return "409 Conflict";
    case 413: return "413 Content Too Large";
    case 422: return "422 Unprocessable Entity";
    case 429: return "429 Too Many Requests";
    case 500: return "500 Internal Server Error";
    case 502: return "502 Bad Gateway";
    case 503: return "503 Service Unavailable";
    case 504: return "504 Gateway Timeout";
    default: return nullptr;
  }
}

// Error statuses only; anything unknown or not an error is sent as 500.
inline int errorStatus(int status)
{
  return (status >= 400 && statusLine(status) != nullptr) ? status : 500;
}

inline const char* defaultReason(int status)
{
  switch (status)
  {
    case 400: return "Bad request";
    case 401: return "Unauthorized";
    case 403: return "Forbidden";
    case 404: return "Not found";
    case 405: return "Method not allowed";
    case 408: return "Request timeout";
    case 409: return "Conflict";
    case 413: return "Content too large";
    case 422: return "Unprocessable entity";
    case 429: return "Too many requests";
    case 502: return "Bad gateway";
    case 503: return "Service unavailable";
    case 504: return "Gateway timeout";
    default: return "Internal server error";
  }
}

// The `code` sent when the caller gives none.
inline const char* defaultCode(int status)
{
  switch (status)
  {
    case 400: return "BAD_REQUEST";
    case 401: return "UNAUTHORIZED";
    case 403: return "FORBIDDEN";
    case 404: return "NOT_FOUND";
    case 405: return "METHOD_NOT_ALLOWED";
    case 408: return "REQUEST_TIMEOUT";
    case 409: return "CONFLICT";
    case 413: return "TOO_LARGE";
    case 422: return "UNPROCESSABLE";
    case 429: return kRateLimited;
    case 502: return "BAD_GATEWAY";
    case 503: return kBusy;
    case 504: return "GATEWAY_TIMEOUT";
    default: return "INTERNAL";
  }
}

// Seconds a client should wait before retrying; 0 means no Retry-After header.
inline uint32_t retryAfterS(int status)
{
  return (status == 429 || status == 503) ? 1 : 0;
}

namespace detail
{
inline bool put(char* out, size_t cap, size_t& n, char c)
{
  if (n + 1 >= cap)
  {
    return false;
  }
  out[n++] = c;
  return true;
}

inline bool putStr(char* out, size_t cap, size_t& n, const char* s)
{
  for (; *s != '\0'; s++)
  {
    if (!put(out, cap, n, *s))
    {
      return false;
    }
  }
  return true;
}

// Writes s as the inside of a JSON string. Non-ASCII bytes become '?' so a cut or hostile byte sequence can never make
// the document invalid. Stops at the first character that does not fit whole.
inline void putEscaped(char* out, size_t cap, size_t& n, const char* s)
{
  static const char hex[] = "0123456789abcdef";
  for (; *s != '\0'; s++)
  {
    const unsigned char c = static_cast<unsigned char>(*s);
    if (c == '"' || c == '\\')
    {
      if (n + 2 >= cap)
      {
        return;
      }
      out[n++] = '\\';
      out[n++] = static_cast<char>(c);
    }
    else if (c < 0x20)
    {
      if (n + 6 >= cap)
      {
        return;
      }
      out[n++] = '\\';
      out[n++] = 'u';
      out[n++] = '0';
      out[n++] = '0';
      out[n++] = hex[c >> 4];
      out[n++] = hex[c & 0xF];
    }
    else if (!put(out, cap, n, c < 0x80 ? static_cast<char>(c) : '?'))
    {
      return;
    }
  }
}
}  // namespace detail

// Writes {"status":"error","reason":..., "code":...} into out and NUL-terminates it. A null reason or code falls back
// to the defaults for `status`. The reason is cut when it does not fit; the code is never cut (codes are short tokens
// and a cap under 96 returns 0). Returns the length without the NUL, 0 when cap is too small.
inline size_t buildBody(char* out, size_t cap, int status, const char* reason, const char* code)
{
  if (out == nullptr || cap < 96)
  {
    return 0;
  }
  if (reason == nullptr || *reason == '\0')
  {
    reason = defaultReason(status);
  }
  if (code == nullptr || *code == '\0')
  {
    code = defaultCode(status);
  }

  // Room kept for the tail (`","code":"` + code + `"}` + NUL) so a long reason cannot crowd the code out.
  size_t codeLen = 0;
  while (code[codeLen] != '\0')
  {
    codeLen++;
  }
  const size_t tail = 10 + codeLen + 2 + 1;
  if (tail + 32 >= cap)
  {
    return 0;
  }

  size_t n = 0;
  detail::putStr(out, cap - tail + 1, n, "{\"status\":\"error\",\"reason\":\"");
  detail::putEscaped(out, cap - tail + 1, n, reason);
  detail::putStr(out, cap, n, "\",\"code\":\"");
  detail::putEscaped(out, cap, n, code);
  detail::putStr(out, cap, n, "\"}");
  out[n] = '\0';
  return n;
}
}  // namespace http_error
