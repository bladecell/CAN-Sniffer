#pragma once

#include <cstdint>
#include "esp_err.h"

namespace supervisor
{
struct SleepOperations
{
  void* context;
  esp_err_t (*timer)(void*, uint64_t);
  esp_err_t (*release)(void*);
  esp_err_t (*acquire)(void*);
};

// A single supervisor owner calls prepare/finish; no ISR or concurrent use.
class SleepTransaction
{
public:
  explicit SleepTransaction(const SleepOperations& operations) : ops_(operations) {}
  esp_err_t prepare(uint64_t us)
  {
    if (released_)
    {
      return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t timerError = ops_.timer(ops_.context, us);
    if (timerError != ESP_OK)
    {
      return timerError;
    }
    const esp_err_t error = ops_.release(ops_.context);
    if (error != ESP_OK)
    {
      return error;
    }
    released_ = true;
    return ESP_OK;
  }
  esp_err_t finish()
  {
    if (!released_)
    {
      return ESP_OK;
    }
    // One restore attempt per release, including a failed acquire. ERROR owns
    // recovery afterward; a retry here could silently nest a successful lock.
    released_ = false;
    return ops_.acquire(ops_.context);
  }
private:
  SleepOperations ops_;
  bool released_ = false;
};
}
