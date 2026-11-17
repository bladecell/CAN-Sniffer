#include "can_mode_service.hpp"

#include <atomic>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstring>

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
  bool obdRequestInFlight();
  void setObdParsing(bool on) { OBD2::getInstance().setParsingEnabled(on); }
  void setHealthPing(bool on) { OBD2::getInstance().setHealthPingEnabled(on); }
  void setRawActive(bool on);
};

// Raw sender state. gPlan is written only by startSender under the mode lock while gSenderRunning is false; the task reads it
// until it clears gSenderRunning.
enum SenderError : uint8_t
{
  kErrNone,
  kErrBusy,
  kErrBusDown,
  kErrTxFailed,
  kErrTxTimeout,
  kErrBadFrame,
  kErrRecording,
  kErrPolling
};
constexpr int      kSenderBusyTolerance = 5;  // consecutive BUSY answers before the plan gives up
raw_tx::Plan       gPlan;
std::atomic<bool>     gSenderRunning{false};
std::atomic<bool>     gSenderStop{false};
std::atomic<uint32_t> gSenderSent{0};
std::atomic<uint32_t> gSenderTotal{0};
std::atomic<uint32_t> gSenderPeriod{0};
std::atomic<uint8_t>  gSenderError{kErrNone};

const char* senderErrorName(uint8_t e)
{
  switch (e)
  {
    case kErrBusy: return "BUSY";
    case kErrBusDown: return "BUS_DOWN";
    case kErrTxFailed: return "TX_FAILED";
    case kErrTxTimeout: return "TX_TIMEOUT";
    case kErrBadFrame: return "BAD_DATA";
    case kErrRecording: return "RECORDING";
    case kErrPolling: return "POLLING_ACTIVE";
    default: return nullptr;
  }
}

void ProdEnv::setRawActive(bool on)
{
  if (!on) { gSenderStop.store(true); }  // the mode ended: the sender goes with it (atomics only, the mode lock is held)
  OBD2::getInstance().setRawActive(on);
}

can_mode::RequestCounter gObdRequests;  // REST OBD requests running; entered under the mutex, left freely
bool ProdEnv::obdRequestInFlight()
{
  return gObdRequests.busy();
}

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

// One frame of the plan, through the mode lock like POST /can/send. kErrNone with `ended` set: Raw mode is gone, no error.
uint8_t sendPlanFrame(const raw_tx::Request& f, bool& ended)
{
  uint8_t    err = kErrNone;
  const auto run = can_mode_service::run(
      can_mode_service::Requirement::rawSend,
      [&](bool fc)
      {
        const Recorder::State rs = Recorder::getInstance().getState();
        if (rs == Recorder::State::Recording || rs == Recorder::State::Stopping) { err = kErrRecording; return false; }
        if (OBD2::getInstance().isAutoPoll()) { err = kErrPolling; return false; }
        switch (OBD2::getInstance().sendRawFrame(f.id, f.data, f.len, fc))
        {
          case ESP_OK: break;
          case ESP_ERR_NOT_FINISHED: err = kErrBusy; break;
          case ESP_ERR_TIMEOUT: err = kErrTxTimeout; break;
          case ESP_FAIL: err = kErrTxFailed; break;
          case ESP_ERR_INVALID_ARG: err = kErrBadFrame; break;
          default: err = kErrBusDown; break;
        }
        return false;  // not activity: the idle timeout still ends a plan nobody watches
      });
  if (run == can_mode_service::Run::notAllowed) { ended = true; }
  else if (run == can_mode_service::Run::busy) { err = kErrBusy; }
  return err;
}

void senderTask(void*)
{
  const raw_tx::Plan& plan = gPlan;
  raw_tx::Cursor      cursor(plan.count, plan.repeat);
  const TickType_t    period = pdMS_TO_TICKS(plan.periodMs) > 0 ? pdMS_TO_TICKS(plan.periodMs) : 1;
  TickType_t          next   = xTaskGetTickCount();
  int                 busy   = 0;
  while (!gSenderStop.load() && !cursor.done())
  {
    const raw_tx::Request& f = plan.frames[cursor.next() % raw_tx::kMaxFrames];
    bool                   ended = false;
    const uint8_t          err   = sendPlanFrame(f, ended);
    if (ended) { break; }
    if (err == kErrBusy)
    {
      if (++busy > kSenderBusyTolerance) { gSenderError.store(err); break; }  // transient: this slot is skipped
    }
    else if (err != kErrNone)
    {
      gSenderError.store(err);
      break;
    }
    else
    {
      busy = 0;
      gSenderSent.store(static_cast<uint32_t>(cursor.sent()));
    }
    next += period;
    for (;;)
    {
      const TickType_t now = xTaskGetTickCount();
      const int32_t    left = static_cast<int32_t>(next - now);
      if (gSenderStop.load() || cursor.done()) { break; }
      if (left <= 0)
      {
        if (left < -static_cast<int32_t>(period)) { next = now; }  // far behind: no burst to catch up
        break;
      }
      vTaskDelay(left > 50 ? 50 : left);
    }
  }
  gSenderRunning.store(false);
  vTaskDelete(nullptr);
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

can_mode::Status set(const char* name, bool fc, bool parsing)
{
  Lock l(1000);
  if (!l.held)
  {
    return can_mode::Status::busy;
  }
  const can_mode::Status s = gController.set(name, fc, parsing, nowMs());
  if (s == can_mode::Status::ok)
  {
    ESP_LOGI(TAG, "Mode %s (fc %d)", gController.name(), gController.fc() ? 1 : 0);
  }
  return s;
}

Snapshot get()
{
  const can_mode::Controller<ProdEnv>::View v = gController.view();
  return Snapshot{can_mode::Controller<ProdEnv>::nameAt(v.mode), v.fc, v.parsing};
}

ObdRequestGuard::ObdRequestGuard() : state_(State::busy)
{
  Lock l(1000);
  if (!l.held)
  {
    return;  // fail closed
  }
  if (!gObdRequests.enter(gController.isDefault()))  // under the mutex: Raw entry checks it under the same mutex
  {
    state_ = State::rawMode;
    return;
  }
  state_ = State::allowed;
}

ObdRequestGuard::~ObdRequestGuard()
{
  if (state_ == State::allowed)
  {
    gObdRequests.leave();
  }
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

SenderStart startSender(const raw_tx::Plan& plan)
{
  SenderStart result = SenderStart::busy;
  const auto  gate   = run(Requirement::rawSend,
                           [&](bool)
                           {
                             const Recorder::State rs = Recorder::getInstance().getState();
                             if (rs == Recorder::State::Recording || rs == Recorder::State::Stopping)
                             {
                               result = SenderStart::recording;
                               return false;
                             }
                             if (OBD2::getInstance().isAutoPoll())
                             {
                               result = SenderStart::polling;
                               return false;
                             }
                             if (gSenderRunning.load())
                             {
                               result = SenderStart::running;
                               return false;
                             }
                             gPlan = plan;
                             gSenderStop.store(false);
                             gSenderError.store(kErrNone);
                             gSenderSent.store(0);
                             gSenderTotal.store(plan.repeat == 0 ? 0 : static_cast<uint32_t>(plan.count) * plan.repeat);
                             gSenderPeriod.store(plan.periodMs);
                             gSenderRunning.store(true);
                             if (xTaskCreate(senderTask, "can_send", 3072, nullptr, tskIDLE_PRIORITY + 2, nullptr) != pdPASS)
                             {
                               gSenderRunning.store(false);
                               result = SenderStart::noTask;
                               return false;
                             }
                             result = SenderStart::started;
                             return true;  // starting counts as activity
                           });
  if (gate == Run::notAllowed) { return SenderStart::notRawMode; }
  if (gate == Run::busy) { return SenderStart::busy; }
  return result;
}

void stopSender()
{
  gSenderStop.store(true);
}

SenderStatus senderStatus()
{
  return SenderStatus{gSenderRunning.load(), gSenderSent.load(), gSenderTotal.load(), gSenderPeriod.load(),
                      senderErrorName(gSenderError.load())};
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
