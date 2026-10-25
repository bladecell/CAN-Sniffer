#include "can_raw.hpp"

#include "can_driver.hpp"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char* const TAG = "CAN_RAW";

namespace
{
constexpr uint32_t kPumpPeriodMs   = 20;    // wake period while streaming
constexpr size_t   kBatchesPerWake = 4;     // 4 x up to 16 frames per 20 ms: 3200 frames/s ceiling
constexpr uint32_t kStatsPeriodMs  = 1000;
constexpr uint32_t kTaskStackBytes = 4096;
constexpr UBaseType_t kTaskPriority = tskIDLE_PRIORITY + 2;  // above the 1 Hz stream task, far below httpd (5) and CAN

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

// CanDriver::setFrameTap callback: rx task for RX frames, TX-done ISR for TX frames. IRAM, short, no logging.
void IRAM_ATTR onFrame(void*, uint32_t id, bool ext, bool rtr, const uint8_t* data, uint8_t length, uint32_t tsUs, bool tx)
{
  gCore.tap(can_raw::makeFrame(id, ext, rtr, length, data, tsUs, tx));
}
}  // namespace

CanRaw::CanRaw()
{
  lock_ = xSemaphoreCreateMutex();
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
  esp_err_t ret = ESP_OK;
  if (!initialized_)
  {
    sink_          = sink;
    sinkArg_       = arg;
    stopRequested_ = false;
    taskRunning_   = true;
    TaskHandle_t h = nullptr;
    if (xTaskCreate(taskEntry, "can_raw", kTaskStackBytes, this, kTaskPriority, &h) != pdPASS)
    {
      taskRunning_ = false;
      ret          = ESP_ERR_NO_MEM;
    }
    else
    {
      task_ = h;
      if (!CanDriver::getInstance().setFrameTap(onFrame, nullptr))
      {
        ESP_LOGW(TAG, "Frame tap install raced with another update");
      }
      initialized_ = true;
    }
  }
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
  return ret;
}

esp_err_t CanRaw::deinit()
{
  if (lock_ == nullptr || xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t ret = ESP_OK;
  if (initialized_)
  {
    freeRing();
    // False: a tap call is still running. The ring is already gone (a late tap finds it inactive), but the task and the
    // hook stay so a later deinit() can finish the job.
    if (!CanDriver::getInstance().setFrameTap(nullptr, nullptr))
    {
      ret = ESP_ERR_TIMEOUT;
    }
    else
    {
      stopRequested_ = true;
      xTaskNotifyGive(static_cast<TaskHandle_t>(task_));
      for (int waited = 0; taskRunning_ && waited < 1000; waited += 10)
      {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      if (taskRunning_)
      {
        ret = ESP_ERR_TIMEOUT;
      }
      else
      {
        task_        = nullptr;
        initialized_ = false;
      }
    }
  }
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
  return ret;
}

esp_err_t CanRaw::start()
{
  if (lock_ == nullptr || xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t ret = ESP_OK;
  if (!initialized_)
  {
    ret = ESP_ERR_INVALID_STATE;
  }
  else if (!gCore.active())
  {
    can_raw::Frame* ring = static_cast<can_raw::Frame*>(
        heap_caps_malloc(can_raw::kRingCapacity * sizeof(can_raw::Frame), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (ring == nullptr)
    {
      ESP_LOGW(TAG, "No memory for the raw frame ring (%u B)",
               static_cast<unsigned>(can_raw::kRingCapacity * sizeof(can_raw::Frame)));
      ret = ESP_ERR_NO_MEM;
    }
    else
    {
      isrBase_ = CanDriver::getInstance().getRxDropped();
      if (!gCore.start(ring, can_raw::kRingCapacity))
      {
        heap_caps_free(ring);
      }
      else
      {
        ESP_LOGI(TAG, "Raw CAN stream on");
        xTaskNotifyGive(static_cast<TaskHandle_t>(task_));
      }
    }
  }
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
  return ret;
}

void CanRaw::freeRing()
{
  // Detached under the core's lock, freed after it: a tap or pump in flight finds an unattached ring.
  if (can_raw::Frame* ring = gCore.stop(); ring != nullptr)
  {
    heap_caps_free(ring);
    ESP_LOGI(TAG, "Raw CAN stream off");
  }
}

void CanRaw::stop()
{
  if (lock_ == nullptr || xSemaphoreTake(static_cast<SemaphoreHandle_t>(lock_), portMAX_DELAY) != pdTRUE)
  {
    return;
  }
  freeRing();
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(lock_));
}

bool CanRaw::isActive()
{
  return gCore.active();
}

void CanRaw::addWsDropped(uint32_t n)
{
  gCore.addWsDropped(n);
}

void CanRaw::sendStats()
{
  const uint32_t isr = CanDriver::getInstance().getRxDropped() - isrBase_;
  const uint8_t  flags = CanDriver::getInstance().getConfig().filter ? can_raw::kFlagHwFilter : 0;
  uint8_t        buf[can_raw::kStatsBytes];
  const size_t   n = can_raw::encodeStats(gCore.snapshot(isr, flags), buf, sizeof(buf));
  if (n != 0 && sink_ != nullptr)
  {
    (void)sink_(sinkArg_, buf, n, 0, false);
  }
}

void CanRaw::sendInactiveStats()
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
    (void)sink_(sinkArg_, buf, n, 0, false);
  }
}

void CanRaw::taskEntry(void* arg)
{
  static_cast<CanRaw*>(arg)->taskLoop();
  vTaskDelete(nullptr);
}

void CanRaw::taskLoop()
{
  TickType_t lastStats = 0;
  bool       wasIdle   = true;
  while (!stopRequested_)
  {
    if (!gCore.active())
    {
      wasIdle = true;
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // no polling while the stream is off
      continue;
    }
    const uint32_t isr = CanDriver::getInstance().getRxDropped() - isrBase_;
    gCore.pump([this](const uint8_t* d, size_t n, size_t frames) { return sink_(sinkArg_, d, n, frames, true); }, isr,
               kBatchesPerWake);
    const TickType_t now = xTaskGetTickCount();
    if (wasIdle || now - lastStats >= pdMS_TO_TICKS(kStatsPeriodMs))
    {
      wasIdle   = false;
      lastStats = now;
      sendStats();
    }
    vTaskDelay(pdMS_TO_TICKS(kPumpPeriodMs));
  }
  taskRunning_ = false;  // last access to this object
}
