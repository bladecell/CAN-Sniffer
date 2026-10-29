#include "can_raw.hpp"

#include "can_driver.hpp"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char* const TAG = "CAN_RAW";

namespace
{
constexpr uint32_t    kPumpPeriodMs   = 20;  // wake period while streaming
constexpr size_t      kBatchesPerWake = 4;   // 4 x up to 16 frames per 20 ms: 3200 frames/s ceiling
constexpr uint32_t    kTaskStackBytes = 4096;
constexpr UBaseType_t kTaskPriority   = tskIDLE_PRIORITY + 2;  // above the 1 Hz stream task, far below httpd (5) and CAN

// The ring is touched from the rx task, the TX-done ISR and the drain task, so one spinlock serves all three.
struct MuxLock
{
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  struct Guard
  {
    portMUX_TYPE& m;
    __attribute__((always_inline)) inline explicit Guard(MuxLock& l) : m(l.mux) { portENTER_CRITICAL_SAFE(&m); }
    __attribute__((always_inline)) inline ~Guard() { portEXIT_CRITICAL_SAFE(&m); }
    Guard(const Guard&)            = delete;
    Guard& operator=(const Guard&) = delete;
  };
};

can_raw::Core<MuxLock> gCore;

// CanDriver::setFrameTap callback: rx task for RX frames, TX-done ISR for TX frames. IRAM, short, no logging. Core::tap,
// Ring::push and makeFrame are always_inline, so no flash code runs here (checked with nm/objdump).
void IRAM_ATTR onFrame(void*, uint32_t id, bool ext, bool rtr, const uint8_t* data, uint8_t length, uint32_t tsUs, bool tx)
{
  gCore.tap(can_raw::makeFrame(id, ext, rtr, length, data, tsUs, tx));
}

esp_err_t toEsp(can_raw::Status s)
{
  switch (s)
  {
    case can_raw::Status::ok:
      return ESP_OK;
    case can_raw::Status::noMemory:
      return ESP_ERR_NO_MEM;
    case can_raw::Status::timeout:
      return ESP_ERR_TIMEOUT;
    default:
      return ESP_ERR_INVALID_STATE;
  }
}
}  // namespace

// The FreeRTOS side of can_raw::Lifecycle.
struct CanRawEnv
{
  CanRaw* self;

  bool createTask()
  {
    self->stopRequested_ = false;
    self->taskRunning_   = true;
    TaskHandle_t h       = nullptr;
    if (xTaskCreate(CanRaw::taskEntry, "can_raw", kTaskStackBytes, self, kTaskPriority, &h) != pdPASS)
    {
      self->taskRunning_ = false;
      return false;
    }
    self->task_ = h;
    return true;
  }
  bool shutdownTask()
  {
    self->stopRequested_ = true;
    xTaskNotifyGive(static_cast<TaskHandle_t>(self->task_));
    for (int waited = 0; self->taskRunning_ && waited < 1000; waited += 10)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (self->taskRunning_)
    {
      return false;
    }
    self->task_ = nullptr;
    return true;
  }
  bool installTap()
  {
    if (!CanDriver::getInstance().setFrameTap(onFrame, nullptr))
    {
      ESP_LOGW(TAG, "Frame tap install raced with another update");
      return false;
    }
    return true;
  }
  // False: a tap call is still running; the ring is already gone (a late tap finds it inactive), so a retry is safe.
  bool        removeTap() { return CanDriver::getInstance().setFrameTap(nullptr, nullptr); }
  can_raw::Frame* allocRing(size_t frames)
  {
    void* p = heap_caps_malloc(frames * sizeof(can_raw::Frame), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p == nullptr)
    {
      ESP_LOGW(TAG, "No memory for the raw frame ring (%u B)", static_cast<unsigned>(frames * sizeof(can_raw::Frame)));
    }
    return static_cast<can_raw::Frame*>(p);
  }
  void     freeRing(can_raw::Frame* ring) { heap_caps_free(ring); }
  uint32_t isrNow() { return CanDriver::getInstance().getRxDropped(); }
  void     wake() { xTaskNotifyGive(static_cast<TaskHandle_t>(self->task_)); }
};

namespace
{
CanRawEnv                                               gEnv{nullptr};
can_raw::Lifecycle<can_raw::Core<MuxLock>, CanRawEnv>   gLife(gCore, gEnv);
}  // namespace

CanRaw::CanRaw()
{
  lock_ = xSemaphoreCreateMutex();
  gEnv.self = this;
}

CanRaw& CanRaw::getInstance()
{
  static CanRaw instance;
  return instance;
}

esp_err_t CanRaw::init(SinkFn sink, void* arg)
{
  if (lock_ == nullptr || sink == nullptr)
  {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }
  sink_    = sink;
  sinkArg_ = arg;
  const esp_err_t ret = toEsp(gLife.init());
  initialized_        = gLife.initialized();
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
  return ret;
}

esp_err_t CanRaw::deinit()
{
  if (lock_ == nullptr || xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }
  const esp_err_t ret = toEsp(gLife.deinit());
  initialized_        = gLife.initialized();
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
  return ret;
}

esp_err_t CanRaw::start()
{
  if (lock_ == nullptr || xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }
  const esp_err_t ret = toEsp(gLife.start());
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
  return ret;
}

void CanRaw::stop()
{
  if (lock_ == nullptr || xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY) != pdTRUE)
  {
    return;
  }
  gLife.stop();
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
}

bool CanRaw::isActive()
{
  return gCore.active();
}

void CanRaw::addWsSkipped(uint32_t n)
{
  gCore.markSkipped(n);
}

bool CanRaw::sendStats()
{
  const uint8_t flags = CanDriver::getInstance().getConfig().filter ? can_raw::kFlagHwFilter : 0;
  uint8_t       buf[can_raw::kStatsBytes];
  const size_t  n = can_raw::encodeStats(gCore.snapshot(CanDriver::getInstance().getRxDropped(), flags), buf, sizeof(buf));
  return n != 0 && sink_ != nullptr && sink_(sinkArg_, buf, n, 0, -1) == can_raw::SinkResult::accepted;
}

void CanRaw::sendInactiveStats(int fd)
{
  if (!initialized_ || sink_ == nullptr)
  {
    return;
  }
  can_raw::Stats s{};
  uint8_t        buf[can_raw::kStatsBytes];
  const size_t   n = can_raw::encodeStats(s, buf, sizeof(buf));
  if (n != 0)
  {
    (void)sink_(sinkArg_, buf, n, 0, fd);
  }
}

void CanRaw::taskEntry(void* arg)
{
  static_cast<CanRaw*>(arg)->taskLoop();
  vTaskDelete(nullptr);
}

void CanRaw::taskLoop()
{
  can_raw::StatsSchedule sched;
  while (!stopRequested_)
  {
    if (!gCore.active())
    {
      sched.reset();  // the first stats frame goes out right after a start
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // no polling while the stream is off
      continue;
    }
    can_raw::drainPass(
        sched, static_cast<uint32_t>(pdTICKS_TO_MS(xTaskGetTickCount())), [this] { return sendStats(); },
        [this]
        {
          return gCore.pump([this](const uint8_t* d, size_t n, size_t frames) { return sink_(sinkArg_, d, n, frames, -1); },
                            CanDriver::getInstance().getRxDropped(), kBatchesPerWake);
        });
    vTaskDelay(pdMS_TO_TICKS(kPumpPeriodMs));
  }
  taskRunning_ = false;  // last access to this object
}
