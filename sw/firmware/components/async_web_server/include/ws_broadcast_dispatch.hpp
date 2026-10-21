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
template <typename Backend>
class Dispatcher
{
public:
    static constexpr size_t kMaxPayload = 125;
    // The SDK ctrl mailbox has CONFIG_LWIP_UDP_RECVMBOX_SIZE (6) slots; leave room for its own session closes
    static constexpr unsigned kMaxPending = 3;
    static constexpr size_t kMaxClients = 20;

    bool start()
    {
        unsigned expected = kStopped;
        return state_.compare_exchange_strong(expected, 0);
    }

    void stop() { state_.fetch_or(kStopped); }
    unsigned pending() const { return state_.load() & ~kStopped; }

    Result submit(Backend backend, const uint8_t* payload, size_t len)
    {
        if (!payload || len == 0 || len > kMaxPayload) { return Result::invalid; }
        unsigned state = state_.load();
        for (;;)
        {
            if (state & kStopped) { return Result::stopped; }
            if (state >= kMaxPending) { return Result::full; }
            if (state_.compare_exchange_weak(state, state + 1)) { break; }
        }
        void* storage = backend.allocate(sizeof(Work));
        if (!storage)
        {
            state_.fetch_sub(1);
            return Result::noMemory;
        }
        Work* work = new (storage) Work{this, backend, len, {}};
        std::memcpy(work->payload, payload, len);
        if (!backend.queue(&execute, work))
        {
            dispose(work);
            return Result::queueFailed;
        }
        return Result::accepted;
    }

private:
    static constexpr unsigned kStopped = 1U << 31;
    std::atomic<unsigned> state_{kStopped};

    struct Work
    {
        Dispatcher* owner;
        Backend backend;
        size_t len;
        uint8_t payload[kMaxPayload];
    };

    static void dispose(Work* work)
    {
        Dispatcher* owner = work->owner;
        Backend backend = work->backend;
        work->~Work();
        backend.release(work);
        owner->state_.fetch_sub(1);
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
            for (size_t i = 0; i < count && i < kMaxClients; ++i)
            {
                if (work->backend.ready(fds[i]))
                {
                    work->backend.send(fds[i], work->payload, work->len);
                }
            }
        }
        dispose(work);
    }
};

} // namespace ws_broadcast
