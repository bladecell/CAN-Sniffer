#pragma once

// Receive loop with an absolute deadline (P2.9c, ASYNC-1). No ESP-IDF includes, so host tests can drive it with fakes.
// `recv(buf, len)` returns bytes read, `timeoutCode` when the socket read timed out (retry), or <= 0 for a closed or failed
// socket. A client that sends nothing (or one byte per timeout) can no longer hold an async worker forever.

#include <cstddef>
#include <cstdint>

namespace body_reader
{
enum class Result
{
  Ok,
  Timeout,
  Error,
};

template <typename Recv, typename NowUs>
Result readBody(Recv&& recv, NowUs&& nowUs, char* buf, size_t total, int timeoutCode, int64_t deadlineUs)
{
  size_t got = 0;
  while (got < total)
  {
    if (nowUs() >= deadlineUs)
    {
      return Result::Timeout;
    }
    const int r = recv(buf + got, total - got);
    if (r == timeoutCode)
    {
      continue;
    }
    if (r <= 0 || static_cast<size_t>(r) > total - got)
    {
      return Result::Error;
    }
    got += static_cast<size_t>(r);
  }
  return Result::Ok;
}
}  // namespace body_reader
