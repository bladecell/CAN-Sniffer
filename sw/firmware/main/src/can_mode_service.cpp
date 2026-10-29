#include "can_mode_service.hpp"

#include <atomic>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "obd2.hpp"
#include "recorder.hpp"
#include "supervisor.hpp"

namespace
{
const char* const TAG = "CAN_MODE";

struct ProdEnv
{
  bool supervisorRunning() { return SUPERVISOR::getInstance().get_state() == SUPERVISOR::State::RUNNING; }
  bool recorderBusy()
  {
    const Recorder::State s = Recorder::getInstance().getState();
    return s == Recorder::State::Recording || s == Recorder::State::Stopping;
  }
  bool autoPoll() { return OBD2::getInstance().isAutoPoll(); }
  bool setAutoPoll(bool on) { return OBD2::getInstance().setAutoPoll(on) == ESP_OK; }
  void fcDisarm() { OBD2::getInstance().disarmRawFlowControl(); }
};

ProdEnv                          gEnv;
can_mode::Controller<ProdEnv>    gController(gEnv);
// Created on first use, from static storage: no allocation that could fail.
SemaphoreHandle_t mutex()
{
  static StaticSemaphore_t buf;
  static SemaphoreHandle_t handle = xSemaphoreCreateMutexStatic(&buf);
  return handle;
}
std::atomic<int>                 gRevertPending{0};  // 0 none, 1 revert and restore, 2 revert without restore

uint32_t nowMs()
{
  return static_cast<uint32_t>(pdTICKS_TO_MS(xTaskGetTickCount()));
}

// Caller holds the mutex.
void applyPendingRevert()
{
  const int pending = gRevertPending.exchange(0);
  if (pending != 0)
  {
    gController.reset(pending == 1);
  }
}

struct Lock
{
  bool held;
  explicit Lock(uint32_t waitMs) : held(xSemaphoreTake(mutex(), pdMS_TO_TICKS(waitMs)) == pdTRUE)
  {
    if (held) { applyPendingRevert(); }
  }
  ~Lock()
  {
    if (held) { xSemaphoreGive(mutex()); }
  }
};
}  // namespace

namespace can_mode_service
{
size_t modeCount()
{
  return can_mode::Controller<ProdEnv>::count();
}

const char* modeName(size_t i)
{
  return can_mode::Controller<ProdEnv>::nameAt(i);
}

can_mode::Status set(const char* name, bool fc)
{
  Lock l(1000);
  if (!l.held)
  {
    return can_mode::Status::busy;
  }
  const can_mode::Status s = gController.set(name, fc, nowMs());
  if (s == can_mode::Status::ok)
  {
    ESP_LOGI(TAG, "Mode %s (fc %d)", gController.name(), gController.fc() ? 1 : 0);
  }
  return s;
}

Snapshot get()
{
  Lock l(200);
  return l.held ? Snapshot{gController.name(), gController.fc()} : Snapshot{"monitor", false};
}

bool allowsRawSend()
{
  Lock l(200);
  return l.held && gController.allows(can_mode::kOpRawSend);
}

bool fcEnabled()
{
  Lock l(200);
  return l.held && gController.fc();
}

void touch()
{
  Lock l(200);
  if (l.held) { gController.touch(nowMs()); }
}

bool isRaw()
{
  Lock l(200);
  return l.held && !gController.isDefault();
}

void tick()
{
  Lock l(50);
  if (l.held) { gController.tick(nowMs()); }
}

void reset(bool restore)
{
  if (xSemaphoreTake(mutex(), pdMS_TO_TICKS(150)) == pdTRUE)
  {
    gRevertPending.store(0);
    gController.reset(restore);
    xSemaphoreGive(mutex());
    return;
  }
  int expected = 0;
  gRevertPending.compare_exchange_strong(expected, restore ? 1 : 2);  // applied by whoever takes the mutex next
}
}  // namespace can_mode_service
