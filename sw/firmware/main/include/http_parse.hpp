#pragma once

// Pure parsers for URI-derived input (P2.13a, TEST-1). No ESP-IDF includes, so host tests can feed them hostile
// strings. webserver.cpp keeps only the httpd glue (reading req->uri and the query string).

#include <cctype>
#include <cerrno>
#include <climits>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "sd_path.hpp"
#include "tree_limits.hpp"

namespace http_parse
{

enum class Result
{
  Ok,
  InvalidArg,
  NoMem,        // output buffer too small
  InvalidSize,  // too many items
};

// uri = apiRoute + [ "/" suffix ] [ "?" query ]. Percent-decodes the suffix into out and validates it as a relative
// SD path. Encoded delimiters ('/', '\\', '?', '#'), NUL and control bytes are rejected so an encoded delimiter can
// never change the path structure.
inline Result sdRestPath(const char* uri, const char* apiRoute, char* out, size_t outSize, bool allowRoot)
{
  if (uri == nullptr || apiRoute == nullptr || out == nullptr || outSize == 0)
  {
    return Result::InvalidArg;
  }

  const size_t routeLen = strlen(apiRoute);
  if (strncmp(uri, apiRoute, routeLen) != 0)
  {
    return Result::InvalidArg;
  }

  const char* read = uri + routeLen;
  if (*read != '\0' && *read != '/' && *read != '?')
  {
    return Result::InvalidArg;
  }

  char* write = out;
  char* end   = out + outSize - 1;

  while (*read != '\0' && *read != '?')
  {
    unsigned char decoded;
    if (*read == '%')
    {
      if (read[1] == '\0' || read[2] == '\0' || !std::isxdigit(static_cast<unsigned char>(read[1])) ||
          !std::isxdigit(static_cast<unsigned char>(read[2])))
      {
        return Result::InvalidArg;
      }

      const char hex[3] = {read[1], read[2], '\0'};
      decoded           = static_cast<unsigned char>(std::strtol(hex, nullptr, 16));
      read += 3;

      if (decoded == '/' || decoded == '\\' || decoded == '?' || decoded == '#')
      {
        return Result::InvalidArg;
      }
    }
    else
    {
      decoded = static_cast<unsigned char>(*read++);
    }

    if (decoded == 0 || decoded < 0x20 || decoded == 0x7f || decoded == '\\' || decoded == '#')
    {
      return Result::InvalidArg;
    }
    if (write == end)
    {
      return Result::NoMem;
    }
    *write++ = static_cast<char>(decoded);
  }

  // URI fragments are not valid in HTTP requests. Reject them even after a query delimiter rather than allowing
  // parser disagreement.
  if (strchr(read, '#') != nullptr)
  {
    return Result::InvalidArg;
  }

  *write = '\0';
  return sd_path::validRelative(out, allowRoot) ? Result::Ok : Result::InvalidArg;
}

// Whole string must be a base-10 int in range. Result::InvalidArg otherwise.
inline Result strictInt(const char* text, int* value)
{
  if (text == nullptr || value == nullptr)
  {
    return Result::InvalidArg;
  }
  char* end = nullptr;
  errno     = 0;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
  {
    return Result::InvalidArg;
  }
  *value = static_cast<int>(parsed);
  return Result::Ok;
}

// Integer path segment: digits then end of string or '?'.
inline Result pathInt(const char* uri, const char* route, int* value)
{
  if (uri == nullptr || route == nullptr || value == nullptr)
  {
    return Result::InvalidArg;
  }
  const size_t routeLen = strlen(route);
  if (strncmp(uri, route, routeLen) != 0)
  {
    return Result::InvalidArg;
  }

  const char* suffix = uri + routeLen;
  if (*suffix == '\0' || *suffix == '?')
  {
    return Result::InvalidArg;
  }

  char* end = nullptr;
  errno     = 0;
  const long parsed = std::strtol(suffix, &end, 10);
  if (end == suffix || (*end != '\0' && *end != '?') || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
  {
    return Result::InvalidArg;
  }

  *value = static_cast<int>(parsed);
  return Result::Ok;
}

// P2.11 NB5: a boolean query value is exactly true, false, 1 or 0 (true/false in any case). Anything else, including an
// empty value, is InvalidArg, so a typo cannot silently turn a switch off.
inline Result strictBool(const char* text, bool* out)
{
  if (text == nullptr || out == nullptr)
  {
    return Result::InvalidArg;
  }
  if (strcasecmp(text, "true") == 0 || strcmp(text, "1") == 0)
  {
    *out = true;
    return Result::Ok;
  }
  if (strcasecmp(text, "false") == 0 || strcmp(text, "0") == 0)
  {
    *out = false;
    return Result::Ok;
  }
  return Result::InvalidArg;
}

// Checks the query of `uri` (after the first '?') against a key whitelist. Every pair must have a non-empty key from
// `allowed` and no key may repeat; values are not looked at. `present` gets bit i set when allowed[i] occurs (n <= 8).
// No query at all is Ok with present == 0. Anything unknown is InvalidArg: callers use this to reject extra keys such as
// GET /dtc?codes=P0301&foo=1 instead of silently ignoring them.
inline Result queryKeys(const char* uri, const char* const* allowed, size_t n, unsigned* present)
{
  if (uri == nullptr || allowed == nullptr || present == nullptr || n > 8)
  {
    return Result::InvalidArg;
  }
  *present            = 0;
  const char* query   = strchr(uri, '?');
  if (query == nullptr)
  {
    return Result::Ok;
  }
  query++;
  while (*query != '\0')
  {
    const char* end = query;
    while (*end != '\0' && *end != '&' && *end != '=')
    {
      end++;
    }
    const size_t keyLen = static_cast<size_t>(end - query);
    int          found  = -1;
    for (size_t i = 0; i < n && keyLen > 0; i++)
    {
      if (strlen(allowed[i]) == keyLen && strncmp(allowed[i], query, keyLen) == 0)
      {
        found = static_cast<int>(i);
        break;
      }
    }
    if (found < 0 || (*present & (1u << found)) != 0)
    {
      return Result::InvalidArg;
    }
    *present |= 1u << found;
    while (*end != '\0' && *end != '&')
    {
      end++;
    }
    query = (*end == '&') ? end + 1 : end;
    if (*end == '&' && *query == '\0')
    {
      return Result::InvalidArg;  // trailing '&'
    }
  }
  return Result::Ok;
}

// GET /sd_card/tree query: optional depth (0..kMaxListDepth), limit (1..kMaxLimit) and offset (0..kMaxOffset), each at
// most once, no other key, decimal digits only. `out` keeps its defaults for absent keys and is untouched on error.
constexpr size_t kMaxTreeOffset = 100000;

inline Result treeLimits(const char* uri, sd_tree::Limits* out)
{
  static const char* const keys[] = {"depth", "limit", "offset"};
  unsigned                 present = 0;
  if (out == nullptr || queryKeys(uri, keys, 3, &present) != Result::Ok)
  {
    return Result::InvalidArg;
  }
  sd_tree::Limits parsed = *out;
  const char*     query  = strchr(uri, '?');
  if (query != nullptr)
  {
    query++;
  }
  while (query != nullptr && *query != '\0')
  {
    const char* eq  = query;
    while (*eq != '\0' && *eq != '=' && *eq != '&')
    {
      eq++;
    }
    const char* end = eq;
    while (*end != '\0' && *end != '&')
    {
      end++;
    }
    const size_t keyLen = static_cast<size_t>(eq - query);
    const char*  value  = (*eq == '=') ? eq + 1 : eq;
    const size_t valLen = static_cast<size_t>(end - value);
    char         text[12];
    if (valLen == 0 || valLen >= sizeof(text))
    {
      return Result::InvalidArg;
    }
    for (size_t i = 0; i < valLen; i++)
    {
      if (value[i] < '0' || value[i] > '9')
      {
        return Result::InvalidArg;  // no sign, no space
      }
      text[i] = value[i];
    }
    text[valLen] = '\0';
    int n        = 0;
    if (strictInt(text, &n) != Result::Ok || n < 0)
    {
      return Result::InvalidArg;
    }
    if (keyLen == 5 && strncmp(query, "depth", 5) == 0)
    {
      parsed.depth = n;
    }
    else if (keyLen == 5 && strncmp(query, "limit", 5) == 0)
    {
      parsed.limit = static_cast<size_t>(n);
    }
    else
    {
      if (static_cast<size_t>(n) > kMaxTreeOffset)
      {
        return Result::InvalidArg;
      }
      parsed.offset = static_cast<size_t>(n);
    }
    query = (*end == '&') ? end + 1 : end;
  }
  if (!parsed.valid())
  {
    return Result::InvalidArg;
  }
  *out = parsed;
  return Result::Ok;
}

inline int hexNibble(char value)
{
  if (value >= '0' && value <= '9')
  {
    return value - '0';
  }
  if (value >= 'A' && value <= 'F')
  {
    return value - 'A' + 10;
  }
  if (value >= 'a' && value <= 'f')
  {
    return value - 'a' + 10;
  }
  return -1;
}

inline bool percentDecodeDtcCodes(const char* encoded, char* decoded, size_t decodedSize)
{
  if (encoded == nullptr || decoded == nullptr || decodedSize == 0)
  {
    return false;
  }

  char*       write = decoded;
  const char* read  = encoded;
  char* const end   = decoded + decodedSize - 1;
  while (*read != '\0')
  {
    unsigned char value;
    if (*read == '%')
    {
      if (read[1] == '\0' || read[2] == '\0')
      {
        return false;
      }

      const int high = hexNibble(read[1]);
      const int low  = hexNibble(read[2]);
      if (high < 0 || low < 0)
      {
        return false;
      }

      value = static_cast<unsigned char>((high << 4) | low);
      read += 3;
    }
    else
    {
      value = static_cast<unsigned char>(*read++);
    }

    if (value == '\0' || write == end)
    {
      return false;
    }
    *write++ = static_cast<char>(value);
  }

  *write = '\0';
  return true;
}

// Decodes `encoded` (the raw "codes" query value) into scratch and splits it on ',' in place. outPtrs point into
// scratch. Empty tokens are InvalidArg, more than maxPtrs tokens is InvalidSize.
inline Result dtcCodeList(const char* encoded, char* scratch, size_t scratchLen, const char* outPtrs[],
                          size_t maxPtrs, size_t* outCount)
{
  *outCount = 0;
  if (!percentDecodeDtcCodes(encoded, scratch, scratchLen))
  {
    return Result::InvalidArg;
  }

  char* token = scratch;
  while (true)
  {
    if (*token == '\0')
    {
      return Result::InvalidArg;
    }
    if (*outCount == maxPtrs)
    {
      return Result::InvalidSize;
    }

    outPtrs[*outCount] = token;
    (*outCount)++;

    char* separator = strchr(token, ',');
    if (separator == nullptr)
    {
      return Result::Ok;
    }

    *separator = '\0';
    token      = separator + 1;
  }
}

inline bool isValidDtcCode(const char* code)
{
  if (code == nullptr || strlen(code) != 5)
  {
    return false;
  }

  const char system = code[0];
  if (system != 'P' && system != 'B' && system != 'C' && system != 'U')
  {
    return false;
  }
  if (code[1] < '0' || code[1] > '3')
  {
    return false;
  }

  for (size_t i = 2; i < 5; ++i)
  {
    if (!((code[i] >= '0' && code[i] <= '9') || (code[i] >= 'A' && code[i] <= 'F')))
    {
      return false;
    }
  }

  return true;
}

}  // namespace http_parse
