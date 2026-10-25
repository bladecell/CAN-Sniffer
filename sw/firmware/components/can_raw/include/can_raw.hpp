// Raw CAN frame stream (Monitor mode, read-only): taps every RX and TX frame at the CAN driver, keeps them in a bounded
// ring while a client has the stream on, and hands batches to a transport sink. The transport (WebSocket) lives in
// main; this component knows nothing about it. See docs/architecture.md (CAN raw stream) and docs/api.md (0x10, 0x11).
#pragma once
#include <cstddef>
#include <cstdint>

#include "can_raw_core.hpp"
#include "esp_err.h"

class CanRaw
{
public:
  // `bulk` is true for 0x10 batches (frames > 0 says how many frames the bytes carry) and false for 0x11 stats. Called from
  // the drain task, never with a CanRaw lock held, and must not block for long.
  using SinkFn = can_raw::SinkResult (*)(void* arg, const uint8_t* data, size_t len, size_t frames, bool bulk);

  static CanRaw& getInstance();

  // Creates the idle drain task and installs the tap. Safe to call twice. Nothing is captured until start().
  esp_err_t init(SinkFn sink, void* arg);
  // Stops the stream, removes the tap and ends the task. ESP_ERR_TIMEOUT: a tap call or the task did not leave in time;
  // nothing is freed that is still in use, call again. Safe to call twice and without init.
  esp_err_t deinit();

  // Starts the stream: allocates the ring (kRingCapacity frames of internal RAM) and wakes the drain task. Already
  // active is OK. ESP_ERR_INVALID_STATE: not initialized. ESP_ERR_NO_MEM: no ring; the stream stays off.
  esp_err_t start();
  // Stops the stream and frees the ring. Safe at any time, also when not active.
  void stop();
  bool isActive();
  bool isInitialized() const { return initialized_; }

  // Frames the transport lost after taking them from the ring (counted in the stream).
  void addWsDropped(uint32_t n);
  // One stats frame with the active bit clear, through the sink; lets a client learn that start() failed.
  void sendInactiveStats();

  CanRaw(const CanRaw&)            = delete;
  CanRaw& operator=(const CanRaw&) = delete;

private:
  CanRaw();
  void        taskLoop();
  static void taskEntry(void* arg);
  void        sendStats();
  void        freeRing();

  void*             lock_         = nullptr;  // SemaphoreHandle_t: serialises init, deinit, start and stop
  void*             task_         = nullptr;  // TaskHandle_t
  SinkFn            sink_         = nullptr;
  void*             sinkArg_      = nullptr;
  bool              initialized_  = false;
  volatile bool     taskRunning_  = false;
  volatile bool     stopRequested_ = false;
  uint32_t          isrBase_      = 0;
};
