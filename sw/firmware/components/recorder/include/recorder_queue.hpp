// Row queue between the OBD callback (one producer) and the writer task (one consumer). push() never blocks: when the
// queue is full it discards the oldest row, so a stalled card keeps the newest data. Host-tested.
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "recorder_core.hpp"

namespace recorder_core
{
class RowQueue
{
public:
  ~RowQueue() { destroy(); }
  bool create(size_t depth)
  {
    destroy();
    if (depth == 0) { return false; }
    dropped_.store(0);
    q_ = xQueueCreate(static_cast<UBaseType_t>(depth), sizeof(Row));
    return q_ != nullptr;
  }
  // Call only after the producer is gated off and the consumer has stopped.
  void destroy()
  {
    if (q_ != nullptr)
    {
      vQueueDelete(q_);
      q_ = nullptr;
    }
  }
  void push(const Row& r)
  {
    if (q_ == nullptr) { return; }
    if (xQueueSend(q_, &r, 0) == pdTRUE) { return; }
    Row old;
    if (xQueueReceive(q_, &old, 0) == pdTRUE) { dropped_.fetch_add(1); }
    if (xQueueSend(q_, &r, 0) != pdTRUE) { dropped_.fetch_add(1); }
  }
  bool pop(Row& r, TickType_t wait) { return q_ != nullptr && xQueueReceive(q_, &r, wait) == pdTRUE; }
  uint32_t dropped() const { return dropped_.load(); }

private:
  QueueHandle_t         q_ = nullptr;
  std::atomic<uint32_t> dropped_{0};
};
}  // namespace recorder_core
