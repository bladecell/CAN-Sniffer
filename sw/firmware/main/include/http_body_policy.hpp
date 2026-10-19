#pragma once

#include <cstddef>
#include <cstdint>

namespace http_body_policy
{
enum class JsonLengthResult
{
  Ok,
  Required,
  TooLarge,
};

// Required-body JSON routes use the server's framing length. Optional-body
// save/load routes bypass this check when content_len is zero, as before.
inline JsonLengthResult jsonContentLength(size_t length, size_t maxLength)
{
  if (length == 0)
  {
    return JsonLengthResult::Required;
  }
  return length > maxLength ? JsonLengthResult::TooLarge : JsonLengthResult::Ok;
}

// An oversized but syntactically valid header returns maxLength + 1 so the
// caller retains its existing 413 precedence over a framing-value mismatch.
// maxLength is the upload byte cap (less than SIZE_MAX).
inline bool uploadContentLength(const char* header, size_t framedLength, size_t maxLength, size_t* length)
{
  if (header == nullptr || length == nullptr)
  {
    return false;
  }
  bool sawDigit = false;
  bool trailingWhitespace = false;
  bool exceedsSizeLimit = false;
  size_t parsedLength = 0;
  for (const char* value = header; *value != '\0'; ++value)
  {
    if (*value >= '0' && *value <= '9')
    {
      if (trailingWhitespace)
      {
        return false;
      }
      sawDigit = true;
      const size_t digit = static_cast<size_t>(*value - '0');
      if (!exceedsSizeLimit)
      {
        if (parsedLength > maxLength / 10 || (parsedLength == maxLength / 10 && digit > maxLength % 10))
        {
          exceedsSizeLimit = true;
        }
        else
        {
          parsedLength = parsedLength * 10 + digit;
        }
      }
    }
    else if (*value == ' ' || *value == '\t')
    {
      if (sawDigit)
      {
        trailingWhitespace = true;
      }
    }
    else
    {
      return false;
    }
  }
  if (!sawDigit)
  {
    return false;
  }
  if (exceedsSizeLimit)
  {
    *length = maxLength + 1;
    return true;
  }
  if (parsedLength != framedLength)
  {
    return false;
  }
  *length = parsedLength;
  return true;
}

enum class UploadResult
{
  Ok,
  Timeout,
  Disconnected,
  WriteError,
};

// The caller owns the buffer, storage lease and cleanup. A successful write is
// progress; socket timeouts do not reset either deadline. The absolute deadline
// includes time spent acquiring/opening storage before entering this loop.
// Recv obeys its buffer size, as httpd_req_recv does; bufferSize is nonzero.
template <typename Recv, typename Write, typename NowUs>
UploadResult receiveUpload(Recv&& recv, Write&& write, NowUs&& nowUs, char* buffer, size_t bufferSize,
                           size_t remaining, int timeoutCode, int64_t idlePeriodUs, int64_t absoluteDeadlineUs)
{
  int64_t noProgressDeadline = nowUs() + idlePeriodUs;
  if (nowUs() >= absoluteDeadlineUs)
  {
    return UploadResult::Timeout;
  }
  while (remaining > 0)
  {
    const int64_t now = nowUs();
    if (now >= noProgressDeadline || now >= absoluteDeadlineUs)
    {
      return UploadResult::Timeout;
    }
    const int received = recv(buffer, remaining < bufferSize ? remaining : bufferSize);
    if (received == timeoutCode)
    {
      continue;
    }
    if (received <= 0 || static_cast<size_t>(received) > remaining)
    {
      return UploadResult::Disconnected;
    }
    if (!write(buffer, static_cast<size_t>(received)))
    {
      return UploadResult::WriteError;
    }
    remaining -= static_cast<size_t>(received);
    noProgressDeadline = nowUs() + idlePeriodUs;
  }
  return nowUs() >= absoluteDeadlineUs ? UploadResult::Timeout : UploadResult::Ok;
}
}  // namespace http_body_policy
