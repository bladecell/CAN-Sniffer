#pragma once

// The device-wide CAN view mode (can_mode.hpp wired to the supervisor, the recorder and OBD2). One mode for the whole
// device. Callable from REST workers and the HTTP task; one mutex serialises every mode change and every action that
// depends on the mode (run()), and a small task (started with Raw mode, gone when there is nothing left to watch) does
// the periodic work, whether or not any WebSocket is open.
#include <cstddef>
#include <cstdint>

#include "can_mode.hpp"

namespace can_mode_service
{
struct Snapshot
{
  const char* name;
  bool        fc;       // isotp_flow_control
  bool        parsing;  // obd_parsing
};

// The mode table, for GET /can/mode.
size_t      modeCount();
const char* modeName(size_t i);

can_mode::Status set(const char* name, bool fc, bool parsing);
// Lock-free: the last state published under the lock, so it is also right while a long action holds the lock.
Snapshot get();

// Around a REST request that puts OBD frames on the bus or arms the DTC/VIN collectors. Allowed only in Monitor mode; while
// it lives Raw mode cannot start (it answers busy). Takes the mode lock for the check only, not for the request.
class ObdRequestGuard
{
public:
  enum class State
  {
    allowed,
    rawMode,  // 409 RAW_MODE
    busy      // 503 BUSY: the lock was not obtained
  };
  ObdRequestGuard();
  ~ObdRequestGuard();
  ObdRequestGuard(const ObdRequestGuard&)            = delete;
  ObdRequestGuard& operator=(const ObdRequestGuard&) = delete;
  State state() const { return state_; }

private:
  State state_;
};

enum class Requirement
{
  monitor,  // the default mode (POST /recorder/start)
  rawSend,  // a mode that allows raw sends (POST /can/send)
  pollOn    // a mode that allows turning polling on (PATCH /obd2 auto_poll true)
};
enum class Run
{
  ran,         // the action ran
  notAllowed,  // the mode does not fit; nothing ran
  busy         // the mode lock could not be had in time; nothing ran
};

// Runs `action(fc)` with the mode lock held, if the current mode fits `need`, so the mode cannot change while it runs and a
// mode switch cannot slip in between the mode check and the action. fc is the mode's flow-control option. The action must
// not call back into this service. It returns true if it counts as activity (restarts the idle timeout).
Run runImpl(Requirement need, bool (*action)(void* ctx, bool fc), void* ctx);
template <typename F>
Run run(Requirement need, F&& action)
{
  return runImpl(
      need, [](void* ctx, bool fc) { return (*static_cast<F*>(ctx))(fc); }, &action);
}

// Back to Monitor: last WebSocket client closed (restore = true), or a component stopping (restore = false when OBD2 goes
// down with it). If the lock is busy the request is remembered and applied by the next set() or periodic check.
void reset(bool restore);
}  // namespace can_mode_service
