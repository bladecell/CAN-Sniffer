#pragma once

// Raw CAN frame capture: frame record, bounded ring, wire encoders and the capture/pump core. No ESP-IDF includes, so
// host tests compile it directly. The Lock policy (a spinlock on the target, a counter in tests) serialises every
// access to the ring; the sink (WebSocket) is never called with the lock held.
//
// Wire formats (little-endian, see docs/api.md):
//   0x10 batch: [0x10][payloadLen u8][dropped u16][baseTs u32] then records
//               {dt i16, id u32 (bit31 ext, bit30 rtr), ctl u8 (bit7 TX, bits0-3 data bytes 0..8), data[len]}
//   0x11 stats: [0x11][21][flags u8][streamed u32][isrDropped u32][ringDropped u32][wsDropped u32][capacity u16][peak u16]
#include <cstddef>
#include <cstdint>

#if defined(__GNUC__)
#define CAN_RAW_INLINE inline __attribute__((always_inline))
#else
#define CAN_RAW_INLINE inline
#endif

namespace can_raw
{
constexpr uint8_t kMsgBatch = 0x10;
constexpr uint8_t kMsgStats = 0x11;
constexpr uint8_t kCmdStart = 0xB0;
constexpr uint8_t kCmdStop  = 0xB1;

constexpr uint32_t kIdExt      = 1u << 31;
constexpr uint32_t kIdRtr      = 1u << 30;
constexpr uint32_t kIdMask     = 0x1FFFFFFFu;
constexpr uint8_t  kCtlTx      = 0x80;
constexpr uint8_t  kCtlLenMask = 0x0F;

constexpr uint8_t kFlagActive   = 0x01;  // stats flags
constexpr uint8_t kFlagHwFilter = 0x02;  // the controller's acceptance filter is on: frames it rejects are never seen

constexpr size_t kBatchHeaderBytes = 8;
constexpr size_t kRecordMinBytes   = 7;
constexpr size_t kMaxBatchBytes    = 257;  // 2-byte header + 255-byte payload (the length byte is a u8)
constexpr size_t kStatsBytes       = 23;
constexpr size_t kRingCapacity     = 256;
constexpr size_t kPumpPeek         = 24;  // frames copied out per lock hold

struct Frame
{
  uint32_t ts_us;  // low 32 bits of the microsecond clock
  uint32_t id;     // 29-bit id, kIdExt / kIdRtr in the top bits
  uint8_t  ctl;    // kCtlTx | data byte count
  uint8_t  data[8];
};

CAN_RAW_INLINE Frame makeFrame(uint32_t id, bool ext, bool rtr, size_t len, const uint8_t* data, uint32_t tsUs, bool tx)
{
  Frame f{};
  f.ts_us = tsUs;
  f.id    = (id & kIdMask) | (ext ? kIdExt : 0) | (rtr ? kIdRtr : 0);
  if (rtr || data == nullptr)
  {
    len = 0;
  }
  if (len > 8)
  {
    len = 8;
  }
  for (size_t i = 0; i < len; i++)
  {
    f.data[i] = data[i];
  }
  f.ctl = static_cast<uint8_t>(static_cast<uint8_t>(len) | (tx ? kCtlTx : 0));
  return f;
}

// Fixed-capacity FIFO over storage the caller owns. Not thread-safe: the Core's lock covers it.
class Ring
{
public:
  CAN_RAW_INLINE void attach(Frame* storage, size_t capacity)
  {
    if (storage == nullptr || capacity == 0)
    {
      return;
    }
    buf_  = storage;
    cap_  = capacity;
    head_ = tail_ = count_ = peak_ = 0;
  }
  CAN_RAW_INLINE Frame* detach()
  {
    Frame* p = buf_;
    buf_     = nullptr;
    cap_ = head_ = tail_ = count_ = peak_ = 0;
    return p;
  }
  CAN_RAW_INLINE bool   attached() const { return buf_ != nullptr; }
  CAN_RAW_INLINE size_t capacity() const { return cap_; }
  CAN_RAW_INLINE size_t size() const { return count_; }
  CAN_RAW_INLINE size_t peak() const { return peak_; }
  CAN_RAW_INLINE void   resetPeak() { peak_ = count_; }

  CAN_RAW_INLINE bool push(const Frame& f)
  {
    if (buf_ == nullptr || count_ >= cap_)
    {
      return false;
    }
    buf_[head_] = f;
    head_       = head_ + 1 == cap_ ? 0 : head_ + 1;
    count_++;
    if (count_ > peak_)
    {
      peak_ = count_;
    }
    return true;
  }

  size_t peek(Frame* out, size_t max) const
  {
    if (buf_ == nullptr || out == nullptr)
    {
      return 0;
    }
    const size_t n = count_ < max ? count_ : max;
    size_t       i = tail_;
    for (size_t k = 0; k < n; k++)
    {
      out[k] = buf_[i];
      i      = i + 1 == cap_ ? 0 : i + 1;
    }
    return n;
  }

  void discard(size_t n)
  {
    if (buf_ == nullptr)
    {
      return;
    }
    if (n > count_)
    {
      n = count_;
    }
    tail_ = (tail_ + n) % cap_;
    count_ -= n;
  }

private:
  Frame* buf_   = nullptr;
  size_t cap_   = 0;
  size_t head_  = 0;
  size_t tail_  = 0;
  size_t count_ = 0;
  size_t peak_  = 0;
};

namespace detail
{
inline void put16(uint8_t* p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put32(uint8_t* p, uint32_t v)
{
  put16(p, static_cast<uint16_t>(v));
  put16(p + 2, static_cast<uint16_t>(v >> 16));
}
}  // namespace detail

// Packs as many leading frames as fit into `out` (at most kMaxBatchBytes). `used` is the number of frames consumed;
// returns the byte count, 0 when nothing fits. A frame whose time is more than an int16 of microseconds from the
// previous one starts the next batch. A length nibble above 8 (corrupt slot) is clamped.
inline size_t encodeBatch(const Frame* f, size_t n, uint16_t dropped, uint8_t* out, size_t cap, size_t& used)
{
  used = 0;
  if (f == nullptr || out == nullptr || n == 0 || cap < kBatchHeaderBytes + kRecordMinBytes)
  {
    return 0;
  }
  if (cap > kMaxBatchBytes)
  {
    cap = kMaxBatchBytes;
  }
  out[0] = kMsgBatch;
  detail::put16(out + 2, dropped);
  detail::put32(out + 4, f[0].ts_us);
  size_t   pos  = kBatchHeaderBytes;
  uint32_t prev = f[0].ts_us;
  for (size_t i = 0; i < n; i++)
  {
    size_t len = f[i].ctl & kCtlLenMask;
    if (len > 8)
    {
      len = 8;
    }
    if (pos + kRecordMinBytes + len > cap)
    {
      break;
    }
    const int32_t dt = static_cast<int32_t>(f[i].ts_us - prev);
    if (dt < INT16_MIN || dt > INT16_MAX)
    {
      break;  // only possible for i > 0: the first frame is its own base
    }
    detail::put16(out + pos, static_cast<uint16_t>(static_cast<int16_t>(dt)));
    detail::put32(out + pos + 2, f[i].id & ~(1u << 29));
    out[pos + 6] = static_cast<uint8_t>(static_cast<uint8_t>(len) | (f[i].ctl & kCtlTx));
    for (size_t k = 0; k < len; k++)
    {
      out[pos + 7 + k] = f[i].data[k];
    }
    pos += kRecordMinBytes + len;
    prev = f[i].ts_us;
    used++;
  }
  if (used == 0)
  {
    return 0;
  }
  out[1] = static_cast<uint8_t>(pos - 2);  // pos <= 257, so this fits
  return pos;
}

struct Stats
{
  uint8_t  flags;
  uint32_t streamed;
  uint32_t isrDropped;
  uint32_t ringDropped;
  uint32_t wsDropped;
  uint16_t capacity;
  uint16_t peak;
};

inline size_t encodeStats(const Stats& s, uint8_t* out, size_t cap)
{
  if (out == nullptr || cap < kStatsBytes)
  {
    return 0;
  }
  out[0] = kMsgStats;
  out[1] = kStatsBytes - 2;
  out[2] = s.flags;
  detail::put32(out + 3, s.streamed);
  detail::put32(out + 7, s.isrDropped);
  detail::put32(out + 11, s.ringDropped);
  detail::put32(out + 15, s.wsDropped);
  detail::put16(out + 19, s.capacity);
  detail::put16(out + 21, s.peak);
  return kStatsBytes;
}

enum class Command
{
  none,
  start,
  stop
};

// Only the first byte counts: like the other WebSocket commands, trailing bytes are ignored.
inline Command parseCommand(const uint8_t* buf, size_t len)
{
  if (buf == nullptr || len == 0)
  {
    return Command::none;
  }
  return buf[0] == kCmdStart ? Command::start : (buf[0] == kCmdStop ? Command::stop : Command::none);
}

// `remainingClients` counts the WebSocket clients that stay connected after the closing one.
inline bool stopOnClose(uint32_t remainingClients)
{
  return remainingClients == 0;
}

enum class SinkResult
{
  accepted,  // the transport took the bytes
  busy,      // try again later; keep the frames
  dropped    // the transport refused them for good; count them as lost
};

enum class Status
{
  ok,
  invalidState,  // not initialised (or the tap could not be installed)
  noMemory,
  timeout  // a tap call or the task did not leave in time: call again
};

// init/deinit/start/stop sequencing of the stream, separated from FreeRTOS so it can be tested with a fake Env. The caller
// serialises the calls (CanRaw holds a mutex). Env provides: createTask(), shutdownTask(), installTap(), removeTap(),
// allocRing(frames), freeRing(ptr), isrNow() (driver RX-drop counter) and wake() (nudge the idle task).
template <typename CoreT, typename Env>
class Lifecycle
{
public:
  Lifecycle(CoreT& core, Env& env) : core_(core), env_(env) {}

  bool initialized() const { return tapInstalled_; }

  // A task that failed to leave on a failed init stays in `taskUp_` and is reused by the next init: never two tasks.
  Status init()
  {
    if (tapInstalled_)
    {
      return Status::ok;
    }
    if (!taskUp_)
    {
      if (!env_.createTask())
      {
        return Status::noMemory;
      }
      taskUp_ = true;
    }
    if (!env_.installTap())
    {
      if (env_.shutdownTask())
      {
        taskUp_ = false;
      }
      return Status::invalidState;
    }
    tapInstalled_ = true;
    return Status::ok;
  }

  // Frees the ring first (a late tap then finds it inactive), then removes the tap, then ends the task. Each step is
  // skipped once done, so a timeout can simply be retried.
  Status deinit()
  {
    releaseRing();
    if (tapInstalled_)
    {
      if (!env_.removeTap())
      {
        return Status::timeout;
      }
      tapInstalled_ = false;
    }
    if (taskUp_)
    {
      if (!env_.shutdownTask())
      {
        return Status::timeout;
      }
      taskUp_ = false;
    }
    return Status::ok;
  }

  Status start()
  {
    if (!tapInstalled_)
    {
      return Status::invalidState;
    }
    if (core_.active())
    {
      return Status::ok;
    }
    Frame* ring = env_.allocRing(kRingCapacity);
    if (ring == nullptr)
    {
      return Status::noMemory;
    }
    if (!core_.start(ring, kRingCapacity, env_.isrNow()))
    {
      env_.freeRing(ring);
      return Status::ok;
    }
    env_.wake();
    return Status::ok;
  }

  void stop() { releaseRing(); }

private:
  void releaseRing()
  {
    if (Frame* ring = core_.stop(); ring != nullptr)
    {
      env_.freeRing(ring);
    }
  }

  CoreT& core_;
  Env&   env_;
  bool   taskUp_       = false;
  bool   tapInstalled_ = false;
};

template <typename Lock>
class Core
{
public:
  // False if already active or the storage is unusable. The storage stays the caller's property: stop() hands it back.
  // `isrBase` is the driver's cumulative RX-drop counter now; later `isrTotal` arguments are measured from it. It is
  // stored under the same lock the pump reads it under.
  bool start(Frame* storage, size_t capacity, uint32_t isrBase = 0)
  {
    typename Lock::Guard g(lock_);
    if (active_ || storage == nullptr || capacity == 0)
    {
      return false;
    }
    ring_.attach(storage, capacity);
    ringDropped_ = wsDropped_ = streamed_ = reported_ = 0;
    isrBase_ = isrBase;
    gen_++;
    active_         = true;
    activeHintFlag_ = true;
    return true;
  }

  // Returns the storage to free, or nullptr when not active.
  Frame* stop()
  {
    typename Lock::Guard g(lock_);
    active_         = false;
    activeHintFlag_ = false;
    return ring_.detach();
  }

  bool active()
  {
    typename Lock::Guard g(lock_);
    return active_;
  }

  // Producers (rx task and TX-done ISR). Constant time, never blocks beyond the lock.
  CAN_RAW_INLINE void tap(const Frame& f)
  {
    if (!activeHintFlag_)
    {
      return;  // stream off: no lock, no copy
    }
    typename Lock::Guard g(lock_);
    if (!active_)
    {
      return;
    }
    if (!ring_.push(f))
    {
      ringDropped_++;
    }
  }

  void addWsDropped(uint32_t n)
  {
    typename Lock::Guard g(lock_);
    wsDropped_ += n;
  }

  // Frames of an already accepted batch that reached no client (the transport found every socket full). They were counted
  // as streamed when the batch was accepted, so they move from there to the ws drops instead of being counted twice.
  void markSkipped(uint32_t n)
  {
    typename Lock::Guard g(lock_);
    streamed_ -= n < streamed_ ? n : streamed_;
    wsDropped_ += n;
  }

  // Sends up to maxBatches batches. Sink: (const uint8_t* data, size_t len, size_t frames) -> SinkResult.
  // `isrTotal` is the driver's cumulative RX-drop counter. Returns the frames delivered.
  template <typename Sink>
  size_t pump(Sink&& sink, uint32_t isrTotal, size_t maxBatches)
  {
    size_t delivered = 0;
    for (size_t b = 0; b < maxBatches; b++)
    {
      Frame    local[kPumpPeek];
      size_t   n = 0;
      uint32_t gen;
      uint32_t total;
      uint32_t reported;
      {
        typename Lock::Guard g(lock_);
        n        = ring_.peek(local, kPumpPeek);
        gen      = gen_;
        total    = (isrTotal - isrBase_) + ringDropped_ + wsDropped_;
        reported = reported_;
      }
      if (n == 0)
      {
        break;
      }
      const uint32_t pending = total - reported;
      const uint16_t dropped = pending > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(pending);
      uint8_t        buf[kMaxBatchBytes];
      size_t         used  = 0;
      const size_t   bytes = encodeBatch(local, n, dropped, buf, sizeof(buf), used);
      if (bytes == 0 || used == 0)
      {
        typename Lock::Guard g(lock_);  // cannot happen for valid frames; never wedge on one
        if (gen_ == gen)
        {
          ring_.discard(1);
          wsDropped_++;
        }
        continue;
      }
      const SinkResult r = sink(buf, bytes, used);  // no lock held
      if (r == SinkResult::busy)
      {
        break;
      }
      typename Lock::Guard g(lock_);
      if (gen_ != gen)
      {
        break;  // stopped and restarted meanwhile: those frames are gone, the new session's are not ours to discard
      }
      ring_.discard(used);
      if (r == SinkResult::accepted)
      {
        streamed_ += static_cast<uint32_t>(used);
        reported_ += dropped;
        delivered += used;
      }
      else
      {
        wsDropped_ += static_cast<uint32_t>(used);
      }
    }
    return delivered;
  }

  Stats snapshot(uint32_t isrTotal, uint8_t flags)
  {
    typename Lock::Guard g(lock_);
    Stats                s{};
    s.flags       = static_cast<uint8_t>(flags | (active_ ? kFlagActive : 0));
    s.streamed    = streamed_;
    s.isrDropped  = isrTotal - isrBase_;
    s.ringDropped = ringDropped_;
    s.wsDropped   = wsDropped_;
    s.capacity    = static_cast<uint16_t>(ring_.capacity());
    s.peak        = static_cast<uint16_t>(ring_.peak());
    ring_.resetPeak();
    return s;
  }

  Lock& lockForTest() { return lock_; }

private:
  Lock          lock_;
  Ring          ring_;
  bool          active_         = false;
  uint32_t      gen_            = 0;
  uint32_t      ringDropped_    = 0;
  uint32_t      wsDropped_      = 0;
  uint32_t      streamed_       = 0;
  uint32_t      reported_       = 0;
  uint32_t      isrBase_        = 0;
  volatile bool activeHintFlag_ = false;
};
}  // namespace can_raw
