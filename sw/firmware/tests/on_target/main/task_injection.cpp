#include "harness.hpp"
#include <atomic>
#include "obd2_simulator.hpp"

namespace {
p28::TaskInjection injection;
std::atomic<bool> released{false}, held{false}, expired{false};
void heldEntry(void* arg) {
  held.store(true);
  const TickType_t start = xTaskGetTickCount();
  while (!released.load() && xTaskGetTickCount() - start < pdMS_TO_TICKS(10000)) vTaskDelay(pdMS_TO_TICKS(10));
  expired.store(!released.load());
  held.store(false);
  dataSimTaskWrapper(arg);
}
}
namespace p28 {
void armTaskInjection(Action action) { released.store(false); expired.store(false); injection.arm(action); }
void releaseTaskEntry() { released.store(true); }
bool taskEntryHeld() { return held.load(); }
bool taskEntryExpired() { return expired.load(); }
unsigned injectionHits() { return injection.hits(); }
}
extern "C" BaseType_t __real_xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, UBaseType_t,
                                                     TaskHandle_t*, BaseType_t);
extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(TaskFunction_t entry, const char* name, uint32_t stack,
                                                     void* arg, UBaseType_t priority, TaskHandle_t* handle, BaseType_t core) {
  // Preserve the original initiating-task/ordinal OBD injection before selecting simulator-only faults.
  if (legacyTaskCreationShouldFail()) return pdFAIL;
  const auto action = injection.take(name, entry == dataSimTaskWrapper && arg == &CanDriver::getInstance());
  if (action == p28::Action::Fail) {
    if (handle) *handle = nullptr;
    return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
  }
  return __real_xTaskCreatePinnedToCore(action == p28::Action::Hold ? heldEntry : entry,
                                      name, stack, arg, priority, handle, core);
}
