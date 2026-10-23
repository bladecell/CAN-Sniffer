#pragma once
#include "support.hpp"
#include "can_driver.hpp"

namespace p28 {
struct CanSnapshot {
  QueueHandle_t rx;
  QueueHandle_t tx;
  const void* ring;
  size_t ringSize;
  twai_node_handle_t node;
  SemaphoreHandle_t connected;
  TaskHandle_t txTask;
  TaskHandle_t healthTask;
  bool resources;
  bool txRunning;
  bool healthRunning;
  bool empty() const {
    return !rx && !tx && !ring && !ringSize && !node && !connected && !txTask && !healthTask &&
           !resources && !txRunning && !healthRunning;
  }
  bool allocated() const {
    return rx && tx && ring && ringSize == 21 && node && connected && txTask && healthTask && resources;
  }
  bool sameStorage(const CanSnapshot& b) const {
    return rx == b.rx && tx == b.tx && ring == b.ring && ringSize == b.ringSize && node == b.node &&
           connected == b.connected && txTask == b.txTask && healthTask == b.healthTask;
  }
};
CanSnapshot snapshotCan(const CanDriver& can);
void armTaskInjection(Action action);
void releaseTaskEntry();
bool taskEntryHeld();
bool taskEntryExpired();
unsigned injectionHits();
bool runLifecycle();
bool cleanupCan();
bool saveBackup();
bool runNvs(unsigned token);
bool restoreExternal(const char* rx, const char* tx);
bool restorationPending();
bool retryRestoration();
[[noreturn]] void stopUnsafe(const char* reason);
}  // namespace p28
bool legacyTaskCreationShouldFail();
void runUnityMenu();
