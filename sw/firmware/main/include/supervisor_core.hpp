#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "power_settings.hpp"
#include "supervisor_registry.hpp"

namespace supervisor
{
constexpr uint32_t kLoopMs = 1000;
constexpr uint32_t kInactiveTicks = 60;
constexpr float kAlternatorVoltage = static_cast<float>(power_settings::Config{}.stay_awake_voltage_mv) / 1000.0f;
constexpr float kLongSleepVoltage = static_cast<float>(power_settings::Config{}.low_battery_voltage_mv) / 1000.0f;
enum class State { UNINITIALIZED, STARTING, NOT_CONNECTED, RUNNING, STOPPING, SLEEPING, ERROR };
enum class StopAck { NONE, PENDING, SLEEP_READY, RECOVERY };

// Executed synchronously on init's caller only after registry admission. The
// application owns NVS/section loading; the core stores only its returned error
// and the copied power-settings snapshot. No callback/context is retained.
struct SettingsBootstrap
{
  void* context = nullptr;
  esp_err_t (*initialize)(void*) = nullptr;
};

// The policy reader runs once on init's caller after successful bootstrap. Other
// lifecycle operations run on the supervisor task (or inline failed-task recovery).
// Lock/unlock and atomic request/state methods are shared with other tasks.
struct Operations
{
  void* context;
  void (*lock)(void*);
  void (*unlock)(void*);
  esp_err_t (*createTask)(void*);
  bool (*busConnected)(void*);
  bool (*probeBus)(void*);
  float (*voltage)(void*);
  esp_err_t (*prepareSleep)(void*, uint64_t);
  esp_err_t (*finishSleep)(void*);
  void (*delayMs)(void*, uint32_t);
  esp_err_t (*standby)(void*, bool);
  esp_err_t (*ledInit)(void*);
  esp_err_t (*ledDeinit)(void*);
  void (*blink)(void*);
  uint32_t (*readFailures)(void*);
  void (*writeFailures)(void*, uint32_t);
  void (*deepSleep)(void*, uint32_t);
  void (*report)(void*, const char*, esp_err_t, bool);
  power_settings::Config (*powerSettings)(void*);
};

class Core
{
public:
  explicit Core(const Operations& operations) : ops_(operations)
  {
    for (size_t i = 0; i < legacy_.size(); ++i)
    {
      legacy_[i].name = Registry::legacyName(i);
    }
  }
  // Registration/init are single-owner boot calls, not ISR APIs. Status and sleep
  // admission may run concurrently; callbacks always execute outside the lock.
  esp_err_t registerComponent(const Descriptor& descriptor);
  esp_err_t init(const SettingsBootstrap& bootstrap);
  const auto& legacySetupSteps() const { return legacy_; }
  // One legacy supervisor iteration. Caller retains the 1 s delay plus bounded SD
  // status poll; inactivity/error timers count iterations, not wall-clock deadlines.
  void step();
  State state() const { return state_.load(); }
  void disconnected();
  bool requestStop();
  // Internal mailbox only: no blocking API, waiter handle or caller-owned storage.
  // SLEEP_READY acknowledges teardown, not supervisor task exit or permanent shutdown.
  StopAck stopAcknowledgment() const { return stopAck_.load(); }
  Lifecycle status(Id id) const;
  power_settings::Config powerPolicy() const;
  uint32_t reserveSleepRequest();
  bool finishSleepRequest(uint32_t token, bool responseSent);

private:
  esp_err_t initialize(Id id, bool boot = false);
  esp_err_t deinit(Id id);
  bool ledStop();
  bool shutdown(bool recovery = false);
  esp_err_t standby(bool enabled);
  void sleepCycle();
  void enterError();
  void recover();
  void invalidateSleepReservation();

  Registry registry_{};
  std::array<SetupStep, kLegacyCount> legacy_{};
  Lifecycle settings_{};
  Operations ops_;
  power_settings::Config policy_{};
  std::atomic<State> state_{State::UNINITIALIZED};
  std::atomic<StopAck> stopAck_{StopAck::NONE};
  uint32_t inactiveTicks_ = 0;
  uint32_t errorTicks_ = 0;
  uint8_t stopAttempts_ = 0;
  bool recoveryIssued_ = false;
  bool ledOff_ = false;
  uint32_t sleepReservation_ = 0;
  uint32_t nextSleepReservation_ = 0;
  bool admitting_ = false;
};
}
