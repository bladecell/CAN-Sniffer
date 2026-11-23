#pragma once

// Internal-RAM admission control and the lwIP per-thread semaphore check (B-0230). No ESP-IDF includes, so the host
// tests include this header directly.
//
// Why: lwIP mallocs an 80 B semaphore from internal RAM the first time a thread calls a netconn API. When internal RAM is
// exhausted that malloc returns NULL and IDF dereferences it (crash). Refusing REST work while internal RAM is low keeps
// the headroom that new threads, sockets and WiFi buffers need.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace heap_admission
{
// REST requests are answered 503 (BUSY, Retry-After kRetryAfterS) while MALLOC_CAP_INTERNAL free is below this.
// 8 KiB, interim (card 0230). Measured steady-state internal free is ~26-31 KB with one client and the minimum is 3720 B,
// so 32 KiB refused nearly every call. The real fix is freeing internal RAM; raise this once that lands.
constexpr size_t   kMinInternalFreeBytes = 8 * 1024;
constexpr uint32_t kRetryAfterS          = 5;

// Routes that stay open under pressure so the UI can still log in and read device status.
// `uri` is req->uri (may carry a query string). WebSocket upgrades never pass through this check.
inline bool isCritical(const char* uri)
{
    if (uri == nullptr)
    {
        return false;
    }
    size_t len = strcspn(uri, "?#");
    if (len == strlen("/api/v1/system") && strncmp(uri, "/api/v1/system", len) == 0)
    {
        return true;
    }
    constexpr char   kAuth[]  = "/api/v1/auth";
    constexpr size_t kAuthLen = sizeof(kAuth) - 1;
    return len >= kAuthLen && strncmp(uri, kAuth, kAuthLen) == 0 && (len == kAuthLen || uri[kAuthLen] == '/');
}

// True when the request may be queued.
inline bool admit(size_t internalFree, const char* uri)
{
    return internalFree >= kMinInternalFreeBytes || isCritical(uri);
}

// `get` returns the thread's lwIP semaphore or nullptr (sys_thread_sem_get). A thread that gets nullptr must not call any
// socket API.
template <class Get>
inline bool threadSemReady(Get get)
{
    return get() != nullptr;
}
// Every worker left at start-up: nothing would consume a queued request.
inline bool allWorkersDown(size_t failed, size_t total)
{
    return total > 0 && failed >= total;
}
}  // namespace heap_admission
