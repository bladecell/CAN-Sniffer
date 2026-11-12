#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

namespace ws_broadcast {

enum class Result { accepted, stopped, full, invalid, noMemory, queueFailed };

// Lets one drop message through per interval and counts the ones it held back.
class DropLog
{
public:
    static constexpr uint32_t kIntervalMs = 5000;

    bool shouldLog(uint32_t nowMs, unsigned& suppressed)
    {
        if (armed_ && nowMs - lastMs_ < kIntervalMs)
        {
            ++suppressed_;
            return false;
        }
        armed_ = true;
        lastMs_ = nowMs;
        suppressed = suppressed_;
        suppressed_ = 0;
        return true;
    }

private:
    bool armed_ = false;
    uint32_t lastMs_ = 0;
    unsigned suppressed_ = 0;
};

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

// Sends frames to the one active WebSocket client. Backend queue callbacks and session operations run in the HTTPD
// task. submit() and submitLow() may run in producers; stop() closes admission before the owner drains HTTPD.
//
// Two paths share the HTTPD ctrl queue, which has few slots (see kMaxPending):
// - submit(): status and recorder frames, one queued work item each, at most kMaxHigh pending.
// - submitLow(): PID updates. They wait in a ring of kLowFrames and one work item sends all of them, so a burst of PID
//   updates costs one slot and cannot push a status frame out.
template <typename Backend>
class Dispatcher
{
public:
    static constexpr size_t kMaxPayload = 125;
    // The SDK ctrl mailbox has CONFIG_LWIP_UDP_RECVMBOX_SIZE (6) slots; leave room for its own session closes
    static constexpr unsigned kMaxPending = 3;
    static constexpr unsigned kMaxHigh = kMaxPending - 1;
    static constexpr size_t kLowFrames = 16;
    static constexpr size_t kLowPayload = 32;

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
        if (!admit()) { return Result::stopped; }
        unsigned high = high_.load();
        do
        {
            if (high >= kMaxHigh)
            {
                state_.fetch_sub(1);
                return Result::full;
            }
        } while (!high_.compare_exchange_weak(high, high + 1));
        void* storage = backend.allocate(sizeof(Work));
        if (!storage)
        {
            high_.fetch_sub(1);
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
        if (!admit())
        {
            dropLow(backend);
            return Result::stopped;
        }
        if (!backend.queue(&drain, this))
        {
            dropLow(backend);
            state_.fetch_sub(1);
            return Result::queueFailed;
        }
        return Result::accepted;
    }

private:
    static constexpr unsigned kStopped = 1U << 31;
    // Rounds one drain item sends before it queues itself again, so a fast producer cannot hold the HTTPD task.
    static constexpr unsigned kDrainRounds = 4;
    std::atomic<unsigned> state_{kStopped};
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

    struct Work
    {
        Dispatcher* owner;
        Backend backend;
        size_t len;
        uint8_t payload[kMaxPayload];
    };

    // Counts one pending item unless stopped.
    bool admit()
    {
        unsigned state = state_.load();
        do
        {
            if (state & kStopped) { return false; }
        } while (!state_.compare_exchange_weak(state, state + 1));
        return true;
    }

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
        work->~Work();
        backend.release(work);
        owner->high_.fetch_sub(1);
        owner->state_.fetch_sub(1);
    }

    void deliver(Backend& backend, uint8_t* payload, size_t len)
    {
        // No HTTPD session can close/reuse an FD between this selection and
        // send: both happen in the same HTTPD work callback.
        const int fd = backend.active();
        if (fd < 0 || !backend.ready(fd) || health_.skip(fd, backend.nowMs())) { return; }
        // A socket that cannot take a frame right now is not written to: a blocking send would hold the HTTPD
        // task. The frame is dropped. The close is deferred to HTTPD's own queue.
        if (backend.writable(fd) && backend.send(fd, payload, len)) { health_.ok(); }
        else if (health_.fail(backend.nowMs())) { backend.evict(fd); }
    }

    static void execute(void* arg)
    {
        Work* work = static_cast<Work*>(arg);
        if (!(work->owner->state_.load() & kStopped)) { work->owner->deliver(work->backend, work->payload, work->len); }
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
        self->state_.fetch_sub(1);
    }
};

} // namespace ws_broadcast
