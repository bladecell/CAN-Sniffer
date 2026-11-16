#pragma once

// CAN bus view modes (UI-CAN-7, UI-CAN-8): a table of modes with enter and exit actions and the operations each allows, and
// a controller that owns the one current mode of the device. It sits next to the supervisor, not in it: the supervisor
// states are power and bus health. No ESP-IDF includes; Env supplies the device (see FakeEnv in test_can_mode.cpp).
//
// Adding a mode (sniffer, ELM327 bridge, SavvyCAN bridge) means one more row in `modeTable()` and one more Op bit; the
// controller, the REST route and the revert paths do not change.
//
// Env: bool supervisorRunning(); bool recorderBusy(); bool autoPoll(); bool setAutoPoll(bool on) (true = done);
//      void fcDisarm(); bool obdRequestInFlight() (a REST OBD request is on the bus); void setObdParsing(bool on) (tell the
//      receive task whether to parse OBD frames); void setHealthPing(bool on) (the OBD2 health-check ping; off in Raw);
//      void setRawActive(bool on) (OBD2 refuses and drops every request that is not the client's own raw frame).
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace can_mode
{
enum Op : uint8_t
{
  kOpRawSend = 1u << 0,  // POST /can/send
  kOpPollOn  = 1u << 1,  // PATCH /obd2 {"auto_poll":true}
};

// A client that stops using Raw mode without leaving (page open, nothing sent) is put back after this long without a mode
// change or a raw send. The 1 Hz stream task checks it.
constexpr uint32_t kIdleTimeoutMs = 300000;

enum class Status
{
  ok,
  unknownMode,  // 422
  notRunning,   // 503: the supervisor is not in RUNNING
  recording,    // 409: the recorder is recording
  busy          // 503: polling could not be stopped (OBD2 stopped or its lock busy)
};

// What a mode remembers about the state it replaced.
struct Saved
{
  bool autoPoll       = false;  // polling was on when Raw was entered
  bool restorePending = false;  // switching it back on failed; retried by tick()
};

// REST OBD requests running right now, so Raw entry can wait for them. enter() is the mode check and the increment in one step
// (the caller holds the mode lock); leave() never underflows.
class RequestCounter
{
public:
  bool enter(bool monitor)
  {
    if (!monitor) { return false; }
    n_.fetch_add(1);
    return true;
  }
  void leave()
  {
    uint32_t n = n_.load();
    while (n != 0 && !n_.compare_exchange_weak(n, n - 1)) {}
  }
  bool busy() const { return n_.load() != 0; }

private:
  std::atomic<uint32_t> n_{0};
};

template <typename Env>
struct ModeDef
{
  const char* name;
  Status (*enter)(Env&, Saved&);
  bool (*exit)(Env&, Saved&);  // false: part of the undo failed (Saved::restorePending says what is left)
  uint8_t ops;
};

namespace detail
{
template <typename Env>
Status enterMonitor(Env&, Saved&)
{
  return Status::ok;
}
template <typename Env>
bool exitMonitor(Env&, Saved&)
{
  return true;
}

// Switch polling back on, but only if it is off now: if someone turned it on meanwhile there is nothing to undo.
template <typename Env>
bool restorePolling(Env& env, Saved& saved)
{
  if (!saved.autoPoll)
  {
    saved.restorePending = false;
    return true;
  }
  if (env.autoPoll() || env.setAutoPoll(true))
  {
    saved.restorePending = false;
    saved.autoPoll       = false;
    return true;
  }
  saved.restorePending = true;
  return false;
}

template <typename Env>
Status enterRaw(Env& env, Saved& saved)
{
  if (!env.supervisorRunning())
  {
    return Status::notRunning;
  }
  if (env.recorderBusy())
  {
    return Status::recording;
  }
  if (env.obdRequestInFlight())
  {
    return Status::busy;  // a DTC/VIN/PID request from REST is still running; Raw starts after it
  }
  if (!saved.restorePending)
  {
    saved.autoPoll = env.autoPoll();  // a restore still pending from the last session keeps the original "on"
  }
  if (env.autoPoll() && !env.setAutoPoll(false))
  {
    return Status::busy;
  }
  if (env.recorderBusy())
  {
    // A recording started between the check and the switch: it owns polling, give it back and stay in Monitor.
    restorePolling(env, saved);
    return Status::recording;
  }
  return Status::ok;
}

template <typename Env>
bool exitRaw(Env& env, Saved& saved)
{
  env.fcDisarm();
  return restorePolling(env, saved);
}
}  // namespace detail

// Index 0 is the default and the fallback: Monitor.
template <typename Env>
const ModeDef<Env>* modeTable()
{
  static const ModeDef<Env> kTable[] = {
      {"monitor", &detail::enterMonitor<Env>, &detail::exitMonitor<Env>, kOpPollOn},
      {"raw", &detail::enterRaw<Env>, &detail::exitRaw<Env>, kOpRawSend},
  };
  return kTable;
}
template <typename Env>
constexpr size_t modeCount()
{
  return 2;
}

template <typename Env>
class Controller
{
public:
  explicit Controller(Env& env) : env_(env) {}

  static size_t      count() { return modeCount<Env>(); }
  static const char* nameAt(size_t i) { return i < count() ? modeTable<Env>()[i].name : nullptr; }

  const char* name() const { return modeTable<Env>()[cur_].name; }
  bool        fc() const { return fc_; }
  bool        parsing() const { return parsing_; }

  // The state as one lock-free read (GET /can/mode, any task). Refreshed inside the controller by every transition, so no
  // caller can forget to: set, reset, deferred reset, idle timeout, supervisor leaving RUNNING.
  struct View
  {
    size_t mode;
    bool   fc;
    bool   parsing;
  };
  View view() const
  {
    const uint32_t v = view_.load();
    return View{static_cast<size_t>(v & 0xFF), (v & 0x100) != 0, (v & 0x200) != 0};
  }
  bool        allows(uint8_t op) const { return (modeTable<Env>()[cur_].ops & op) != 0; }
  bool        isDefault() const { return cur_ == 0; }

  // Work for tick() while this is true: Raw mode (idle timeout, supervisor state), a failed restore, a deferred reset.
  bool needsTick() const { return cur_ != 0 || saved_.restorePending || deferred_.load() != 0; }

  // For a caller that could not get the lock that serialises this controller: the reset is remembered (lock-free) and
  // applied by the next set() or tick().
  void deferReset(bool restore)
  {
    int expected = 0;
    deferred_.compare_exchange_strong(expected, restore ? 1 : 2);
  }
  void applyDeferred()
  {
    const int pending = deferred_.exchange(0);
    if (pending != 0)
    {
      reset(pending == 1);
    }
  }

  // `name` is compared exactly. `fc` (ISO-TP flow control) and `parsing` (OBD parsing of received frames) are independent
  // switches that only mean something in a mode that allows raw sends; leaving that mode resets both to false (parsing is
  // then on again in the receive task: Monitor always parses). Without `parsing` it is false.
  Status set(const char* name, bool fc, uint32_t nowMs) { return set(name, fc, false, nowMs); }
  Status set(const char* name, bool fc, bool parsing, uint32_t nowMs)
  {
    applyDeferred();
    size_t target = count();
    for (size_t i = 0; name != nullptr && i < count(); i++)
    {
      if (std::strcmp(name, modeTable<Env>()[i].name) == 0)
      {
        target = i;
        break;
      }
    }
    if (target == count())
    {
      return Status::unknownMode;
    }
    if (target == cur_ && fc_ && !fc)
    {
      env_.fcDisarm();  // staying in the mode but turning flow control off: an arming from an earlier send must not linger
    }
    if (target != cur_)
    {
      leave(true);
      const Status s = modeTable<Env>()[target].enter(env_, saved_);
      if (s != Status::ok)
      {
        return s;  // stays in the default mode
      }
      cur_ = target;
    }
    fc_           = allows(kOpRawSend) && fc;
    parsing_      = allows(kOpRawSend) && parsing;
    lastActivity_ = nowMs;
    publishParsing();
    return Status::ok;
  }

  void touch(uint32_t nowMs) { lastActivity_ = nowMs; }

  // About once a second. Reverts an idle Raw mode, or one that outlived RUNNING, and retries a failed restore.
  void tick(uint32_t nowMs)
  {
    applyDeferred();
    const bool running = env_.supervisorRunning();
    if (cur_ != 0)
    {
      // Not RUNNING: the bus dropped (OBD2 stays up and polling must come back) or OBD2 is stopping (the restore is refused
      // and dropped below). Idle: nobody has used Raw mode for kIdleTimeoutMs. Either way: best-effort restore.
      if (!running || static_cast<uint32_t>(nowMs - lastActivity_) >= kIdleTimeoutMs)
      {
        reset(true);
      }
    }
    if (cur_ == 0 && saved_.restorePending)
    {
      if (running)
      {
        detail::restorePolling(env_, saved_);
      }
      else
      {
        saved_ = Saved{};
      }
    }
  }

  // Back to Monitor (last WebSocket client closed, idle, bus drop, stop). restore = false when the polling flag must not be
  // touched because OBD2 is stopping.
  void reset(bool restore)
  {
    if (cur_ != 0)
    {
      leave(restore);
    }
  }

private:
  void leave(bool restore)
  {
    if (cur_ == 0)
    {
      return;
    }
    // Monitor's flags first (OBD2 requests allowed, parsing, ping), then the undo: restoring polling refills the poll queue,
    // which OBD2 would drop while it still believed Raw was active.
    const size_t was = cur_;
    cur_             = 0;
    fc_              = false;
    parsing_         = false;
    publishParsing();
    if (restore)
    {
      modeTable<Env>()[was].exit(env_, saved_);
    }
    else
    {
      env_.fcDisarm();
      saved_ = Saved{};
    }
  }

  // The receive task parses unless a mode that does not parse is on (only Raw without the option).
  void publishParsing()
  {
    env_.setObdParsing(cur_ == 0 || parsing_);
    env_.setHealthPing(cur_ == 0);  // Raw sends only what the user sends
    env_.setRawActive(cur_ != 0);
    view_.store(static_cast<uint32_t>(cur_) | (fc_ ? 0x100u : 0u) | (parsing_ ? 0x200u : 0u));
  }

  Env&     env_;
  size_t   cur_          = 0;
  bool     fc_           = false;
  bool     parsing_      = false;
  uint32_t lastActivity_ = 0;
  Saved    saved_;
  std::atomic<uint32_t> view_{0};
  std::atomic<int> deferred_{0};  // 0 none, 1 reset with restore, 2 reset without
};
}  // namespace can_mode
