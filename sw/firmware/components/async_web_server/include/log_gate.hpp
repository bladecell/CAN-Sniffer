#pragma once

// Lets a warning through at most once per interval and counts what it swallowed, so a flood of identical drops
// (a raw CAN stream outrunning the WebSocket) cannot flood the log. No ESP-IDF includes; host-tested.
// Lock-free: the caller that wins the compare-exchange on the timestamp logs, so one line per interval even across tasks.
#include <atomic>
#include <cstdint>

class LogGate
{
public:
    explicit constexpr LogGate(uint32_t intervalMs) : intervalMs_(intervalMs) {}

    // True when the caller should log; `suppressed` is then the number of calls refused since the last line.
    bool allow(uint32_t nowMs, uint32_t& suppressed)
    {
        // The winner of the claim logs; a caller that loses the claim to another task counts as suppressed.
        uint32_t last = last_.load();
        if (!started_.load())
        {
            last_.store(nowMs);  // before the claim: a caller that sees started_ must see a valid timestamp
            bool expected = false;
            if (!started_.compare_exchange_strong(expected, true))
            {
                suppressed_.fetch_add(1);
                return false;
            }
        }
        else if (static_cast<uint32_t>(nowMs - last) < intervalMs_ || !last_.compare_exchange_strong(last, nowMs))
        {
            suppressed_.fetch_add(1);
            return false;
        }
        suppressed = suppressed_.exchange(0);
        return true;
    }

private:
    const uint32_t        intervalMs_;
    std::atomic<bool>     started_{false};
    std::atomic<uint32_t> last_{0};
    std::atomic<uint32_t> suppressed_{0};
};
