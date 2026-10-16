#pragma once

// Admission gate for CanDriver calls (B51). No ESP-IDF includes, so host tests can include it directly.
// enter() admits a call unless the gate is closed; leave() ends it. close() stops new admissions, drain() then waits
// (bounded) for the calls already inside. One atomic word: bit 31 = closed, low bits = calls in flight, so a call can
// never slip in between "closed" and "count".
#include <atomic>
#include <cstdint>

class OpGate
{
public:
    bool enter()
    {
        uint32_t s = state_.load();
        while (!(s & kClosed))
        {
            if (state_.compare_exchange_weak(s, s + 1))
            {
                return true;
            }
        }
        return false;
    }

    void leave() { state_.fetch_sub(1); }

    void close() { state_.fetch_or(kClosed); }

    void open() { state_.fetch_and(~kClosed); }

    bool closed() const { return (state_.load() & kClosed) != 0; }

    uint32_t inFlight() const { return state_.load() & ~kClosed; }

    // True once no call is in flight; false after timeoutMs. sleepFn(stepMs) waits one step.
    template <typename SleepFn>
    bool drain(uint32_t timeoutMs, uint32_t stepMs, SleepFn sleepFn) const
    {
        for (uint32_t waited = 0; inFlight() != 0; waited += stepMs)
        {
            if (waited >= timeoutMs)
            {
                return false;
            }
            sleepFn(stepMs);
        }
        return true;
    }

    // Releases the call on scope exit; admitted() says whether the call got in.
    class Scope
    {
    public:
        explicit Scope(OpGate& g) : gate_(g), admitted_(g.enter()) {}
        ~Scope()
        {
            if (admitted_)
            {
                gate_.leave();
            }
        }
        Scope(const Scope&)            = delete;
        Scope& operator=(const Scope&) = delete;
        bool admitted() const { return admitted_; }

    private:
        OpGate& gate_;
        bool    admitted_;
    };

private:
    static constexpr uint32_t kClosed = 0x80000000u;
    std::atomic<uint32_t>     state_{kClosed};  // starts closed: open() when the driver is up
};
