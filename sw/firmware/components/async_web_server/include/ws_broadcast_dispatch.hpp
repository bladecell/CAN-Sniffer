#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

namespace ws_broadcast {

enum class Result { accepted, stopped, full, invalid, noMemory, queueFailed };

// Backend queue callbacks and session operations run in the HTTPD task. submit()
// may run in producers; stop() closes admission before the owner drains HTTPD.
//
// Two classes of broadcast, each with its own pending budget. Normal ones (PID updates, status, recorder state, pong) are
// at most kMaxPayload bytes, at most kMaxPending pending, and go to every upgraded client. Bulk ones (the raw CAN stream:
// batches and stats) may be larger, at most kMaxBulkPending pending, and go only to subscribed clients (Backend::subscribed;
// or, with a target descriptor, only to that one) whose socket has room (Backend::writable), so a slow phone cannot park the HTTPD task in a blocking send. The budgets are independent: a
// saturated stream never makes a normal submit fail. Bulk items that reached no writable client add their `units`
// (frames) to a counter the producer reads with takeBulkSkipped().
template <typename Backend>
class Dispatcher
{
public:
    enum class Class : uint8_t { normal, bulk };

    static constexpr size_t kMaxPayload = 125;
    static constexpr size_t kMaxBulkPayload = 257;
    // The SDK ctrl mailbox has CONFIG_LWIP_UDP_RECVMBOX_SIZE (6) slots; 3 + 1 leaves room for its own session closes
    static constexpr unsigned kMaxPending = 3;
    static constexpr unsigned kMaxBulkPending = 1;
    static constexpr size_t kMaxClients = 20;

    bool start()
    {
        unsigned expected = kStopped;
        return state_.compare_exchange_strong(expected, 0);
    }

    void stop() { state_.fetch_or(kStopped); }
    unsigned pending() const
    {
        const unsigned state = state_.load();
        return countOf(state, false) + countOf(state, true);
    }
    unsigned bulkPending() const { return countOf(state_.load(), true); }
    uint32_t takeBulkSkipped() { return skipped_.exchange(0); }

    Result submit(Backend backend, const uint8_t* payload, size_t len, Class cls = Class::normal, uint16_t units = 0,
                  int target = -1)
    {
        const bool bulk = cls == Class::bulk;
        if (!payload || len == 0 || len > (bulk ? kMaxBulkPayload : kMaxPayload)) { return Result::invalid; }
        // One word holds the stop bit and both counters, so admission and the stop are decided together.
        unsigned state = state_.load();
        for (;;)
        {
            if (state & kStopped) { return Result::stopped; }
            if (countOf(state, bulk) >= (bulk ? kMaxBulkPending : kMaxPending)) { return Result::full; }
            if (state_.compare_exchange_weak(state, state + unit(bulk))) { break; }
        }
        void* storage = backend.allocate(sizeof(Work) + len);
        if (!storage)
        {
            state_.fetch_sub(unit(bulk));
            return Result::noMemory;
        }
        Work* work = new (storage) Work{this, backend, len, units, target, bulk};
        std::memcpy(work->payload(), payload, len);
        if (!backend.queue(&execute, work))
        {
            dispose(work);
            return Result::queueFailed;
        }
        return Result::accepted;
    }

private:
    static constexpr unsigned kStopped = 1U << 31;
    static constexpr unsigned kBulkShift = 8;  // bits 0-7 normal count, 8-15 bulk count
    static constexpr unsigned unit(bool bulk) { return bulk ? 1U << kBulkShift : 1U; }
    static constexpr unsigned countOf(unsigned state, bool bulk) { return (state >> (bulk ? kBulkShift : 0)) & 0xFFU; }
    std::atomic<unsigned> state_{kStopped};

    std::atomic<uint32_t> skipped_{0};

    // The payload follows the header in the same allocation, so a 20-byte PID update does not pay for a bulk-sized slot.
    struct Work
    {
        Dispatcher* owner;
        Backend backend;
        size_t len;
        uint16_t units;
        int target;  // bulk only: -1 = every subscribed client, else just this descriptor
        bool bulk;
        uint8_t* payload() { return reinterpret_cast<uint8_t*>(this + 1); }
    };

    static void dispose(Work* work)
    {
        Dispatcher* owner = work->owner;
        Backend backend = work->backend;
        const bool bulk = work->bulk;
        work->~Work();
        backend.release(work);
        owner->state_.fetch_sub(unit(bulk));
    }

    static void execute(void* arg)
    {
        Work* work = static_cast<Work*>(arg);
        if (!(work->owner->state_.load() & kStopped))
        {
            int fds[kMaxClients];
            size_t count = kMaxClients;
            if (!work->backend.list(fds, count)) { count = 0; }
            // No HTTPD session can close/reuse an FD between this selection and
            // send: both happen in the same HTTPD work callback.
            bool skipped = false;
            for (size_t i = 0; i < count && i < kMaxClients; ++i)
            {
                if (work->backend.ready(fds[i]))
                {
                    if (work->bulk)
                    {
                        const bool wanted = work->target >= 0 ? fds[i] == work->target : work->backend.subscribed(fds[i]);
                        if (!wanted) { continue; }
                        if (!work->backend.writable(fds[i]))
                        {
                            skipped = true;
                            continue;
                        }
                    }
                    work->backend.send(fds[i], work->payload(), work->len);
                }
            }
            if (skipped) { work->owner->skipped_.fetch_add(work->units); }
        }
        dispose(work);
    }
};

} // namespace ws_broadcast
