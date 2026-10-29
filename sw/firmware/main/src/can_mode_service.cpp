#include "can_mode_service.hpp"

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

constexpr uint32_t kCheckPeriodMs = 1000;

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

ProdEnv                       gEnv;
can_mode::Controller<ProdEnv> gController(gEnv);
bool                          gCheckTaskRunning = false;  // changed with the mutex held only

// Created on first use, from static storage: no allocation that could fail.
SemaphoreHandle_t mutex()
{
  static StaticSemaphore_t buf;
  static SemaphoreHandle_t handle = xSemaphoreCreateMutexStatic(&buf);
  return handle;
}

uint32_t nowMs()
{
  return static_cast<uint32_t>(pdTICKS_TO_MS(xTaskGetTickCount()));
}

// Once a second while there is something to watch: the idle timeout of Raw mode, the supervisor leaving RUNNING, a restore
// that failed, a reset that could not take the lock. Leaves by itself when the controller needs nothing, so Monitor mode
// costs no task, and it needs no WebSocket. Holds nothing across its delay, so a stop or sleep never waits for it.
void checkTask(void*)
{
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(kCheckPeriodMs));
    if (xSemaphoreTake(mutex(), pdMS_TO_TICKS(500)) != pdTRUE)
    {
      continue;
    }
    gController.tick(nowMs());
    if (!gController.needsTick())
    {
      gCheckTaskRunning = false;
      xSemaphoreGive(mutex());
      vTaskDelete(nullptr);
    }
    xSemaphoreGive(mutex());
  }
}

// Mutex held.
void ensureCheckTask()
{
  if (gCheckTaskRunning || !gController.needsTick())
  {
    return;
  }
  if (xTaskCreate(checkTask, "can_mode", 3072, nullptr, tskIDLE_PRIORITY + 1, nullptr) == pdPASS)
  {
    gCheckTaskRunning = true;
  }
  else
  {
    ESP_LOGW(TAG, "No task for the Raw mode idle timeout");
  }
}

struct Lock
{
  bool held;
  explicit Lock(uint32_t waitMs) : held(xSemaphoreTake(mutex(), pdMS_TO_TICKS(waitMs)) == pdTRUE)
  {
    if (held) { gController.applyDeferred(); }
  }
  ~Lock()
  {
    if (held)
    {
      ensureCheckTask();
      xSemaphoreGive(mutex());
    }
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

Run runImpl(Requirement need, bool (*action)(void*, bool), void* ctx)
{
  Lock l(1000);
  if (!l.held)
  {
    return Run::busy;  // fail closed: the caller refuses instead of guessing the mode
  }
  const bool fits = need == Requirement::monitor  ? gController.isDefault()
                    : need == Requirement::rawSend ? gController.allows(can_mode::kOpRawSend)
                                                   : gController.allows(can_mode::kOpPollOn);
  if (!fits)
  {
    return Run::notAllowed;
  }
  if (action(ctx, gController.fc()) && need == Requirement::rawSend)
  {
    gController.touch(nowMs());
  }
  return Run::ran;
}

void reset(bool restore)
{
  if (xSemaphoreTake(mutex(), pdMS_TO_TICKS(150)) == pdTRUE)
  {
    gController.applyDeferred();
    gController.reset(restore);
    ensureCheckTask();
    xSemaphoreGive(mutex());
    return;
  }
  gController.deferReset(restore);  // applied by whoever takes the mutex next; the check task is running in Raw mode
}
}  // namespace can_mode_service
