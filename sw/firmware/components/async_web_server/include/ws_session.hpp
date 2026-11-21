#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>

// One WebSocket client at a time (decisions.md 2026-11-11, RD2 and RD4). Pure rules, host-tested.
namespace ws_session {

// Close codes in the private range (RFC 6455, 4000-4999). The frontend reads them from the close event.
constexpr uint16_t kCloseInUse = 4000;     // another session holds the device; reconnect with takeover=1 to replace it
constexpr uint16_t kCloseReplaced = 4001;  // this session was replaced by a newer one
constexpr uint16_t kCloseSameSession = 4002;  // the same page reconnected (same sid); the page reconnects quietly
// The frontend pings every 3 s. A holder silent for this long is treated as gone and is replaced without asking.
constexpr uint32_t kStaleMs = 10000;

struct Holder
{
    int fd = -1;
    uint32_t sid = 0;
    uint32_t lastRxMs = 0;
};

enum class Admit { accept, replace, busy };

// `sid` 0: the client sent none (scripts, old pages); it replaces the holder like a takeover.
inline Admit admit(const Holder& holder, uint32_t nowMs, uint32_t sid, bool takeover)
{
    if (holder.fd < 0) { return Admit::accept; }
    if (takeover || sid == 0 || sid == holder.sid || nowMs - holder.lastRxMs >= kStaleMs) { return Admit::replace; }
    return Admit::busy;
}

// A replace by the same live session id is a page reload: Raw CAN mode stays on and the new page re-subscribes.
// Takeover, no sid, a different sid and a stale holder end Raw mode with the replaced session.
inline bool keepsRawOnReplace(const Holder& holder, uint32_t nowMs, uint32_t sid, bool takeover)
{
    return !takeover && sid != 0 && sid == holder.sid && nowMs - holder.lastRxMs < kStaleMs;
}

// Close code for the replaced socket: the same page reconnecting is not another session moving in.
inline uint16_t replaceCode(const Holder& holder, uint32_t sid, bool takeover)
{
    return (!takeover && sid != 0 && sid == holder.sid) ? kCloseSameSession : kCloseReplaced;
}

// 1 to 8 hex digits, not zero. Anything else is 0 (no session id).
inline uint32_t parseSid(const char* text)
{
    if (!text || !*text) { return 0; }
    size_t len = 0;
    while (text[len] != 0)
    {
        const char c = text[len];
        const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        if (!hex || ++len > 8) { return 0; }
    }
    return static_cast<uint32_t>(std::strtoul(text, nullptr, 16));
}

// Payload of a close frame: the status code big-endian, or nothing for code 0. Returns the payload length.
inline size_t closePayload(uint16_t code, uint8_t out[2])
{
    if (code == 0) { return 0; }
    out[0] = static_cast<uint8_t>(code >> 8);
    out[1] = static_cast<uint8_t>(code & 0xFF);
    return 2;
}

}  // namespace ws_session
