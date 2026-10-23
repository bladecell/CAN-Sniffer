#include "harness.hpp"
#include <cstdio>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "obd2_simulator.hpp"
#include "settings.hpp"

namespace {
bool expect(bool condition, const char* label) {
  std::printf("P28 %s %s\n", condition ? "PASS" : "FAIL", label);
  return condition;
}
struct Heap { size_t internal; size_t total; UBaseType_t tasks; };
Heap settledHeap() {
  vTaskDelay(pdMS_TO_TICKS(500));  // Both idle tasks must reclaim self-deleted task stacks/TCBs.
  return {heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
          heap_caps_get_free_size(MALLOC_CAP_8BIT), uxTaskGetNumberOfTasks()};
}
bool stopped(CanDriver& can) {
  return expect(p28::snapshotCan(can).empty() && !xDataSimTaskHandle && !can.isInitialized(),
                "all cycle-owned resources released (lifetime mutexes/static sim queue excluded)");
}
bool initSim(CanDriver& can, const CanDriver::Config& config) {
  const esp_err_t err = can.init(config);
  std::printf("P28 init=%s\n", esp_err_to_name(err));
  return expect(err == ESP_OK && p28::snapshotCan(can).allocated() && xDataSimTaskHandle,
                "real CAN/simulator initialization");
}
}
namespace p28 {
bool cleanupCan() {
  releaseTaskEntry();
  auto& can = CanDriver::getInstance();
  for (unsigned i = 0; i < 3; ++i) {
    if (can.deinit() == ESP_OK && stopped(can)) return true;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  std::puts("P28 UNSAFE: resources retained; no reset, no next scenario");
  return false;
}
bool runLifecycle() {
  auto& can = CanDriver::getInstance();
  if (!cleanupCan()) return false;
  can.setSimHooks(start_sim_task, stop_sim_task, sim_notify);
  auto config = Settings::defaultCanConfig();
  config.debug = true;
  bool ok = initSim(can, config);
  const bool warmedClean = cleanupCan();
  if (!ok || !warmedClean) return false;
  const Heap baseline = settledHeap();
  for (unsigned cycle = 0; cycle < 10; ++cycle) {
    if (!initSim(can, config)) { cleanupCan(); return false; }
    const auto live = snapshotCan(can);
    std::printf("P28 cycle=%u stack_free tx=%u health=%u sim=%u\n", cycle,
                static_cast<unsigned>(uxTaskGetStackHighWaterMark(live.txTask)),
                static_cast<unsigned>(uxTaskGetStackHighWaterMark(live.healthTask)),
                static_cast<unsigned>(uxTaskGetStackHighWaterMark(xDataSimTaskHandle)));
    if (!cleanupCan()) return false;
    ok = expect(can.deinit() == ESP_OK && stopped(can), "double deinit") && ok;
    const Heap after = settledHeap();
    std::printf("P28 cycle=%u heap internal=%u baseline=%u total=%u baseline=%u tasks=%u baseline=%u\n", cycle,
                static_cast<unsigned>(after.internal), static_cast<unsigned>(baseline.internal),
                static_cast<unsigned>(after.total), static_cast<unsigned>(baseline.total),
                static_cast<unsigned>(after.tasks), static_cast<unsigned>(baseline.tasks));
    ok = expect(after.internal == baseline.internal && after.total == baseline.total && after.tasks == baseline.tasks,
                "settled heap and task count unchanged") && ok;
  }
  std::printf("P28 CASE cycles %s\n", ok ? "PASS" : "FAIL");
  armTaskInjection(Action::Fail);
  const esp_err_t failedInit = can.init(config);
  const bool failedClean = stopped(can);
  const bool failCase = expect(failedInit == ESP_ERR_NO_MEM && injectionHits() == 1 && failedClean,
                               "actual DataSimTask allocation failure propagates and rolls back");
  ok = failCase && ok;
  std::printf("P28 CASE create-failure %s\n", failCase ? "PASS" : "FAIL");
  if (!cleanupCan()) return false;
  armTaskInjection(Action::Pass);
  const bool retry = initSim(can, config);
  const bool retryClean = cleanupCan();
  if (!retry || !retryClean) return false;
  armTaskInjection(Action::Hold);
  if (!initSim(can, config)) { cleanupCan(); return false; }
  for (unsigned waited = 0; !taskEntryHeld() && waited < 1000; waited += 10) vTaskDelay(pdMS_TO_TICKS(10));
  const auto before = snapshotCan(can);
  const TaskHandle_t sim = xDataSimTaskHandle;
  const int64_t start = esp_timer_get_time();
  const esp_err_t stop = can.deinit();
  const int64_t elapsed = esp_timer_get_time() - start;
  const auto after = snapshotCan(can);
  const bool timeoutCase = expect(stop == ESP_ERR_TIMEOUT && elapsed >= 2000000 &&
      injectionHits() == 1 && taskEntryHeld() && !taskEntryExpired() && xDataSimTaskHandle == sim &&
      before.sameStorage(after) && after.resources && !can.isInitialized(),
      "real simulator stop wait timed out with live task and unchanged storage");
  std::printf("P28 stop_elapsed_us=%lld\n", static_cast<long long>(elapsed));
  const bool rejected = expect(can.init(config) == ESP_ERR_INVALID_STATE, "reinit rejected while resources retained");
  releaseTaskEntry();
  if (!cleanupCan()) return false;
  armTaskInjection(Action::Pass);
  const bool reinit = initSim(can, config);
  const bool clean = cleanupCan();
  const bool timeoutRecovered = timeoutCase && rejected && reinit && clean;
  std::printf("P28 CASE stop-timeout %s\n", timeoutRecovered ? "PASS" : "FAIL");
  const Heap final = settledHeap();
  const bool stable = expect(final.internal == baseline.internal && final.total == baseline.total &&
                            final.tasks == baseline.tasks, "fault recovery heap/tasks match warmed baseline");
  std::printf("P28 main_stack_free=%u\n", static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
  return ok && timeoutRecovered && stable;
}
}
