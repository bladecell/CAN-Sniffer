#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

namespace ws_broadcast {

enum class Result { accepted, stopped, full, invalid, noMemory, queueFailed };

// Decides when the one WebSocket client is skipped and when it is evicted. A full send buffer or a short Wi-Fi gap
// drops frames; only failures that keep coming for kEvictAfterMs close the socket. HTTPD task only.
class SendHealth
{
public:
    static constexpr unsigned kEvictFailures = 3;
    static constexpr uint32_t kEvictAfterMs = 5000;
    // After a failed send the client gets no frame for this long, so work already queued does not hit the same socket.
    static constexpr uint32_t kSkipMs = 250;

    bool skip(int fd, uint32_t nowMs)
    {
        if (fd != fd_)
        {
            fd_ = fd;
            failures_ = 0;
        }
        return failures_ != 0 && static_cast<int32_t>(nowMs - skipUntilMs_) < 0;
    }
    void ok() { failures_ = 0; }
    // True when the client should be closed.
    bool fail(uint32_t nowMs)
    {
        if (failures_ == 0) { firstFailMs_ = nowMs; }
        if (failures_ < kEvictFailures) { ++failures_; }
        skipUntilMs_ = nowMs + kSkipMs;
        return failures_ >= kEvictFailures && nowMs - firstFailMs_ >= kEvictAfterMs;
    }

private:
    int fd_ = -1;
    unsigned failures_ = 0;
    uint32_t firstFailMs_ = 0;
    uint32_t skipUntilMs_ = 0;
};

// Sends frames to the one active WebSocket client (Backend::active). Backend queue callbacks and session operations run
// in the HTTPD task. submit() and submitLow() may run in producers; stop() closes admission before the owner drains HTTPD.
//
// Three paths share the HTTPD ctrl queue, which has few slots (see kMaxPending), each with its own budget:
// - submit(): status and recorder frames, one queued work item each, at most kMaxHigh pending, at most kMaxPayload bytes.
// - submitLow(): PID updates. They wait in a ring of kLowFrames and one work item sends all of them, so a burst of PID
//   updates costs one slot and cannot push a status frame out.
// - submit(..., Class::bulk): the raw CAN stream (batches and stats), at most kMaxBulkPayload bytes, kMaxBulkPending
//   pending. It goes to the active client only when that client is subscribed (Backend::subscribed), or, with a target
//   descriptor, when the target is the active client. A client whose socket has no room (Backend::writable) or sits in a
//   send-failure skip window is passed over instead of blocking HTTPD in a send. Bulk items that reached nobody add their
//   `units` (frames) to a counter the producer reads with takeBulkSkipped(). The budgets are independent: a saturated
//   stream never makes a normal submit fail.
// Every send, bulk included, feeds SendHealth, so a stream into a stuck socket evicts it like any other frame.
template <typename Backend>
class Dispatcher
{
public:
    enum class Class : uint8_t { normal, bulk };

    static constexpr size_t kMaxPayload = 125;
    static constexpr size_t kMaxBulkPayload = 257;
    // The SDK ctrl mailbox has CONFIG_LWIP_UDP_RECVMBOX_SIZE (6) slots; 3 + 1 leaves room for its own session closes
    static constexpr unsigned kMaxPending = 3;
    // The PID drain item takes one of the kMaxPending slots, so status frames get the rest.
    static constexpr unsigned kMaxHigh = kMaxPending - 1;
    static constexpr unsigned kMaxBulkPending = 1;
    static constexpr size_t kLowFrames = 16;
    static constexpr size_t kLowPayload = 32;

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
        if (state_.load() & kStopped) { return Result::stopped; }
        if (!bulk && !reserveHigh()) { return Result::full; }
        const Result admitted = admit(bulk);
        if (admitted != Result::accepted)
        {
            if (!bulk) { high_.fetch_sub(1); }
            return admitted;
        }
        void* storage = backend.allocate(sizeof(Work) + len);
        if (!storage)
        {
            release(bulk);
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

    // A PID update: no allocation, dropped (full) when kLowFrames are already waiting.
    Result submitLow(Backend backend, const uint8_t* payload, size_t len)
    {
        if (!payload || len == 0 || len > kLowPayload) { return Result::invalid; }
        if (state_.load() & kStopped) { return Result::stopped; }
        backend.lock();
        if (lowCount_ >= kLowFrames)
        {
            backend.unlock();
            return Result::full;
        }
        LowFrame& slot = low_[(lowHead_ + lowCount_) % kLowFrames];
        slot.len = static_cast<uint8_t>(len);
        std::memcpy(slot.payload, payload, len);
        ++lowCount_;
        const bool needWork = !drainQueued_;
        drainQueued_ = true;
        backend.unlock();
        if (!needWork) { return Result::accepted; }

        // This producer owns the one drain item until it is queued or given up.
        drainBackend_ = backend;
        const Result admitted = admit(false);
        if (admitted != Result::accepted)
        {
            dropLow(backend);
            return admitted;
        }
        if (!backend.queue(&drain, this))
        {
            dropLow(backend);
            release(false);
            return Result::queueFailed;
        }
        return Result::accepted;
    }

private:
    static constexpr unsigned kStopped = 1U << 31;
    static constexpr unsigned kBulkShift = 8;  // bits 0-7 normal count, 8-15 bulk count
    static constexpr unsigned unit(bool bulk) { return bulk ? 1U << kBulkShift : 1U; }
    static constexpr unsigned countOf(unsigned state, bool bulk) { return (state >> (bulk ? kBulkShift : 0)) & 0xFFU; }
    // Rounds one drain item sends before it queues itself again, so a fast producer cannot hold the HTTPD task.
    static constexpr unsigned kDrainRounds = 4;
    // One word holds the stop bit and both counters, so admission and the stop are decided together.
    std::atomic<unsigned> state_{kStopped};
    // Status frames in flight (a subset of the normal count): the drain item keeps its own slot.
    std::atomic<unsigned> high_{0};
    SendHealth health_;

    struct LowFrame
    {
        uint8_t len;
        uint8_t payload[kLowPayload];
    };
    // Guarded by backend.lock(): the ring and the flag that says a drain item is queued or running.
    LowFrame low_[kLowFrames]{};
    size_t lowHead_ = 0;
    size_t lowCount_ = 0;
    bool drainQueued_ = false;
    Backend drainBackend_{};

    std::atomic<uint32_t> skipped_{0};

    // The payload follows the header in the same allocation, so a 20-byte PID update does not pay for a bulk-sized slot.
    struct Work
    {
        Dispatcher* owner;
        Backend backend;
        size_t len;
        uint16_t units;
        int target;  // bulk only: -1 = the active client if subscribed, else just this descriptor
        bool bulk;
        uint8_t* payload() { return reinterpret_cast<uint8_t*>(this + 1); }
    };

    bool reserveHigh()
    {
        unsigned high = high_.load();
        do
        {
            if (high >= kMaxHigh) { return false; }
        } while (!high_.compare_exchange_weak(high, high + 1));
        return true;
    }

    // Counts one pending item of the class unless stopped or its budget is spent.
    Result admit(bool bulk)
    {
        unsigned state = state_.load();
        for (;;)
        {
            if (state & kStopped) { return Result::stopped; }
            if (countOf(state, bulk) >= (bulk ? kMaxBulkPending : kMaxPending)) { return Result::full; }
            if (state_.compare_exchange_weak(state, state + unit(bulk))) { return Result::accepted; }
        }
    }

    void release(bool bulk) { state_.fetch_sub(unit(bulk)); }

    void dropLow(Backend& backend)
    {
        backend.lock();
        lowCount_ = 0;
        drainQueued_ = false;
        backend.unlock();
    }

    static void dispose(Work* work)
    {
        Dispatcher* owner = work->owner;
        Backend backend = work->backend;
        const bool bulk = work->bulk;
        work->~Work();
        backend.release(work);
        if (!bulk) { owner->high_.fetch_sub(1); }
        owner->release(bulk);
    }

    // `bulk` is null for normal frames. No HTTPD session can close/reuse an FD between selection and send: both happen in
    // the same HTTPD work callback.
    void deliver(Backend& backend, uint8_t* payload, size_t len, const Work* bulk = nullptr)
    {
        const int fd = backend.active();
        if (fd < 0 || !backend.ready(fd)) { return; }
        if (bulk && !(bulk->target >= 0 ? fd == bulk->target : backend.subscribed(fd))) { return; }
        const uint32_t now = backend.nowMs();
        // A socket that cannot take a frame right now is not written to: a blocking send would hold the HTTPD
        // task. The frame is dropped. The close is deferred to HTTPD's own queue.
        if (!health_.skip(fd, now))
        {
            if (backend.writable(fd) && backend.send(fd, payload, len))
            {
                health_.ok();
                return;
            }
            if (health_.fail(backend.nowMs())) { backend.evict(fd); }
        }
        if (bulk) { skipped_.fetch_add(bulk->units); }
    }

    static void execute(void* arg)
    {
        Work* work = static_cast<Work*>(arg);
        if (!(work->owner->state_.load() & kStopped))
        {
            work->owner->deliver(work->backend, work->payload(), work->len, work->bulk ? work : nullptr);
        }
        dispose(work);
    }

    static void drain(void* arg)
    {
        Dispatcher* self = static_cast<Dispatcher*>(arg);
        Backend backend = self->drainBackend_;
        LowFrame batch[kLowFrames];
        for (unsigned round = 0;; ++round)
        {
            const bool stopped = (self->state_.load() & kStopped) != 0;
            backend.lock();
            const size_t count = self->lowCount_;
            for (size_t i = 0; i < count; ++i) { batch[i] = self->low_[(self->lowHead_ + i) % kLowFrames]; }
            self->lowHead_ = (self->lowHead_ + count) % kLowFrames;
            self->lowCount_ = 0;
            // Nothing left, or stopping: this item is done. The flag clears under the lock, so the next submitLow
            // queues a new item and no frame is left waiting without one.
            const bool done = count == 0 || stopped;
            if (done) { self->drainQueued_ = false; }
            backend.unlock();
            if (done) { break; }
            for (size_t i = 0; i < count; ++i) { self->deliver(backend, batch[i].payload, batch[i].len); }
            if (round + 1 >= kDrainRounds)
            {
                // Still busy: give the HTTPD task back and continue from the queue. The pending count is kept.
                if (backend.queue(&drain, self)) { return; }
                self->dropLow(backend);
                break;
            }
        }
        self->release(false);
    }
};

} // namespace ws_broadcast
