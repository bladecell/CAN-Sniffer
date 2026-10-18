#pragma once

#include <cstddef>
#include <cstring>

namespace wifi_scan_http
{
inline bool accepts(const char* uri, size_t bodyBytes, bool transferEncoded)
{
    return uri && bodyBytes == 0 && !transferEncoded && std::strchr(uri, '?') == nullptr;
}
}
