#pragma once

#include <cstddef>
#include <cstring>

#include "esp_err.h"

namespace manual_sleep_http
{
inline bool accepts(const char* uri, size_t bodyBytes, bool transferEncoded)
{
    return uri != nullptr && strcmp(uri, "/api/v1/system/sleep") == 0 && bodyBytes == 0 && !transferEncoded;
}

// Auth is enforced by the existing gated route before reaching this seam.
template <class Reserve, class Reply, class Finish>
esp_err_t dispatch(const char* uri, size_t bodyBytes, bool transferEncoded, Reserve reserve, Reply reply, Finish finish)
{
    if (!accepts(uri, bodyBytes, transferEncoded))
    {
        return reply(400);
    }
    const auto token = reserve();
    if (token == 0)
    {
        return reply(503);
    }
    const esp_err_t error = reply(202);
    finish(token, error == ESP_OK);
    return error;
}
}  // namespace manual_sleep_http
