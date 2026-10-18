#pragma once

// Pure decisions behind send_json_response and the upload handlers, so host tests can pin them.
#include <cstdint>

#include "../../components/async_web_server/include/http_error.hpp"

namespace response_rules
{
enum class Kind { NoContent, Json, Error };

// A handler returns a JSON tree and a status. No tree: 204 is an empty success, anything else is an error response
// (a null tree with a success status means the allocation failed).
inline Kind kindFor(bool hasBody, int status)
{
  if (hasBody) { return Kind::Json; }
  return status == 204 ? Kind::NoContent : Kind::Error;
}

// 200 is httpd's default; every other status needs an explicit status line.
inline bool needsStatusLine(int status) { return status != 200; }

// Retry-After seconds for a response: only for 429 and 503, the handler's own value wins over the default.
inline uint32_t retryAfter(int status, uint32_t handlerValue)
{
  if (status < 400 || http_error::retryAfterS(status) == 0) { return 0; }
  return handlerValue != 0 ? handlerValue : http_error::retryAfterS(status);
}

// PUT /sd_card/file/<path>: 201 when the file or directory was created, 200 when it already existed.
inline int putStatus(bool created) { return created ? 201 : 200; }
}  // namespace response_rules
