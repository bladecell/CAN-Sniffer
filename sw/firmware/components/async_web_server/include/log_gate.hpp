#pragma once

// Lets a warning through at most once per interval and counts what it swallowed, so a flood of identical drops
// (a raw CAN stream outrunning the WebSocket) cannot flood the log. No ESP-IDF includes; host-tested.
// Races between tasks can let two lines through at the interval edge, never more, and nothing blocks.
#include <atomic>
#include <cstdint>

class LogGate
{
public:
    explicit constexpr LogGate(uint32_t intervalMs) : intervalMs_(intervalMs) {}

    // True when the caller should log; `suppressed` is then the number of calls refused since the last line.
    bool allow(uint32_t nowMs, uint32_t& suppressed)
    {
        if (started_.load() && static_cast<uint32_t>(nowMs - last_.load()) < intervalMs_)
        {
            suppressed_.fetch_add(1);
            return false;
        }
        started_.store(true);
        last_.store(nowMs);
        suppressed = suppressed_.exchange(0);
        return true;
    }

private:
    const uint32_t        intervalMs_;
    std::atomic<bool>     started_{false};
    std::atomic<uint32_t> last_{0};
    std::atomic<uint32_t> suppressed_{0};
};
