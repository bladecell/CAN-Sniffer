#pragma once

// Callback + argument published as one unit, readable from an ISR (B52). No ESP-IDF includes, so host tests can include it.
//
// Two fixed slots, no allocation, nobody frees anything. The reader (Guard) counts itself into the slot it read, checks the
// slot is still current, copies {fn,arg} and keeps the count until it goes out of scope (the callback runs inside it).
// set() fills the slot that is not current, publishes it with one pointer store, then waits for the readers of the old
// slot to leave. So: a reader always sees a matching pair, and when set() returns true no call of the previous pair is
// running or can start. A slot is rewritten only while its reader count is zero (stale readers that bail out on the
// recheck never touch fn/arg). The reader never blocks: after kRetries lost races it sees "no hook" (one TX-done report
// lost, only while a set() is in progress).
// set() must not be called from the callback itself (it would wait for its own reader) or from two threads at once
// (the second gets false).
#include <atomic>
#include <cstdint>

#if defined(__GNUC__)
#define HOOK_CELL_INLINE inline __attribute__((always_inline))
#else
#define HOOK_CELL_INLINE inline
#endif

template <typename Fn>
class HookCell
{
    struct Slot
    {
        Fn                    fn  = nullptr;
        void*                 arg = nullptr;
        std::atomic<uint32_t> readers{0};
    };

public:
    static constexpr int kRetries = 4;

    class Guard
    {
    public:
        Fn    fn  = nullptr;
        void* arg = nullptr;

        // probe (tests only; nullptr folds away when inlined) runs between counting into the slot and the recheck.
        HOOK_CELL_INLINE explicit Guard(HookCell& c, void (*probe)() = nullptr)
        {
            for (int i = 0; i < kRetries; i++)
            {
                Slot* p = c.cur_.load();
                p->readers.fetch_add(1);
                if (probe != nullptr)
                {
                    probe();
                }
                if (c.cur_.load() == p)
                {
                    slot_ = p;
                    fn    = p->fn;
                    arg   = p->arg;
                    return;
                }
                p->readers.fetch_sub(1);
            }
        }
        HOOK_CELL_INLINE ~Guard()
        {
            if (slot_ != nullptr)
            {
                slot_->readers.fetch_sub(1);
            }
        }
        Guard(const Guard&)            = delete;
        Guard& operator=(const Guard&) = delete;

    private:
        Slot* slot_ = nullptr;
    };

    // Replaces (or, with fn == nullptr, clears) the hook. sleepFn(stepMs) waits one step. False: another set() is running
    // or a reader did not leave within timeoutMs; the new pair may already be live then, the old one may still be running.
    template <typename SleepFn>
    bool set(Fn fn, void* arg, uint32_t timeoutMs, uint32_t stepMs, SleepFn sleepFn)
    {
        static_assert(std::atomic<Slot*>::is_always_lock_free, "the published pointer must be a plain word store");
        if (setting_.exchange(true))
        {
            return false;
        }
        Slot* const oldSlot = cur_.load();
        Slot* const newSlot = oldSlot == &slots_[0] ? &slots_[1] : &slots_[0];
        bool        ok      = waitIdle(newSlot, timeoutMs, stepMs, sleepFn);
        if (ok)
        {
            newSlot->fn  = fn;
            newSlot->arg = arg;
            cur_.store(newSlot);
            ok = waitIdle(oldSlot, timeoutMs, stepMs, sleepFn);
        }
        setting_.store(false);
        return ok;
    }

private:
    template <typename SleepFn>
    static bool waitIdle(Slot* s, uint32_t timeoutMs, uint32_t stepMs, SleepFn sleepFn)
    {
        if (stepMs == 0)
        {
            stepMs = 1;  // 0 would never reach the timeout
        }
        for (uint32_t waited = 0; s->readers.load() != 0; waited += stepMs)
        {
            if (waited >= timeoutMs)
            {
                return false;
            }
            sleepFn(stepMs);
        }
        return true;
    }

    Slot              slots_[2];
    std::atomic<Slot*> cur_{&slots_[0]};
    std::atomic<bool> setting_{false};
};
