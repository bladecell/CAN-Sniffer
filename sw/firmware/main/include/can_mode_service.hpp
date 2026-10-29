#pragma once

// The device-wide CAN view mode (can_mode.hpp wired to the supervisor, the recorder and OBD2). One mode for the whole
// device. Callable from REST workers, the HTTP task and the 1 Hz stream task; a mutex serialises them.
#include <cstddef>
#include <cstdint>

#include "can_mode.hpp"

namespace can_mode_service
{
struct Snapshot
{
  const char* name;
  bool        fc;
};

// The mode table, for GET /can/mode.
size_t      modeCount();
const char* modeName(size_t i);

can_mode::Status set(const char* name, bool fc);
Snapshot         get();
// Raw sends are allowed now (Raw mode), and the fc option of the current mode.
bool allowsRawSend();
bool fcEnabled();
// A raw send happened: restarts the idle timeout.
void touch();
// True in any mode other than the default. POST /recorder/start is refused then.
bool isRaw();
// 1 Hz: idle timeout, supervisor left RUNNING, failed restore retry.
void tick();
// Back to Monitor: last WebSocket client closed (restore = true), or a component stopping (restore = false when OBD2 goes
// down with it). If the mutex is busy the request is remembered and applied by the next call or tick.
void reset(bool restore);
}  // namespace can_mode_service
