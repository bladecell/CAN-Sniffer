// Trip recorder: subscribes to OBD2 PID updates and appends long-format CSV rows (t_ms,pid,value,valid) to
// /logs/trips/trip_NNNNNN.csv on the SD card, with a trip_NNNNNN.json sidecar. See docs/decisions.md (2026-10-24).
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "recorder_core.hpp"

class Recorder
{
public:
  using State = recorder_core::State;

  struct Config
  {
    recorder_core::PidSel pids[recorder_core::kMaxPids] = {};
    size_t                pidCount                      = 0;  // 0 = every PID OBD2 currently polls (capped at kMaxPids)
  };

  struct Status
  {
    State     state;
    esp_err_t lastError;  // last failure of this or the previous run; ESP_OK after a clean start
    uint32_t  trip;       // index of the current or last trip file, 0 before the first
    uint32_t  rows;       // rows written to the card
    uint32_t  dropped;    // rows lost to a full queue, a stuck card or a full line buffer
    uint32_t  fileBytes;  // size of the current trip CSV
  };

  static Recorder& getInstance();

  // Supervisor lifecycle. init() subscribes to OBD2 (OBD2::deinit clears subscribers, so every init re-subscribes);
  // deinit() stops a running recording first. ESP_ERR_TIMEOUT: the writer did not finish in 5 s, retry.
  esp_err_t init();
  esp_err_t deinit();

  // ESP_ERR_INVALID_STATE: not initialized. ESP_ERR_NOT_FINISHED: a session is live (recording, stopping, or paused in Error). ESP_ERR_INVALID_ARG: bad PID list.
  // ESP_ERR_NOT_FOUND: no card mounted (state becomes Error). ESP_ERR_NO_MEM: allocation failed.
  // Blocks for the lifecycle lock only; the writer runs in its own task.
  esp_err_t start(const Config& config);
  // Drains the queue, writes the last batch and the sidecar, frees the writer. Blocks up to 5 s; a second concurrent call returns ESP_ERR_TIMEOUT at once.
  esp_err_t stop();

  struct TripInfo
  {
    uint32_t index;
    uint32_t bytes;  // size of trip_NNNNNN.csv
    bool     sidecar;
  };
  // The newest trips (at most `max`, newest first) from /logs/trips. `truncated`: older ones exist. Takes the SD lease for
  // up to 250 ms and may be called while recording. No directory yet is an empty list.
  esp_err_t listTrips(TripInfo* out, size_t max, size_t& count, bool& truncated);

  State  getState() const { return state_.get(); }
  Status getStatus() const;

private:
  struct Session;

  Recorder();
  Recorder(const Recorder&)            = delete;
  Recorder& operator=(const Recorder&) = delete;

  void      onPid(uint16_t pid, uint32_t gen);
  esp_err_t stopLocked();
  static void taskEntry(void* arg);
  void        writerLoop(Session& s);
  bool        flush(Session& s);
  bool        flushBatch(Session& s);
  bool        fail(esp_err_t err);
  esp_err_t   openFile(Session& s);
  esp_err_t   writeSidecar(Session& s, bool complete);
  uint32_t    defHash(const Session& s) const;
  void        snapshotMeta(Session& s);

  recorder_core::StateMachine state_;
  recorder_core::CallbackGate gate_;
  SemaphoreHandle_t           lock_ = nullptr;
  Session*                    session_ = nullptr;  // written under lock_ while the gate is closed
  bool                        initialized_ = false;
  uint32_t                    nextTrip_ = 0;  // 0 = /logs/trips not scanned yet; only the writer task touches it
  std::atomic<uint32_t>       gen_{0};
  std::atomic<bool>           stopInFlight_{false};
  std::atomic<esp_err_t>      lastError_{ESP_OK};
  std::atomic<uint32_t>       trip_{0};
  std::atomic<uint32_t>       rows_{0};
  std::atomic<uint32_t>       dropped_{0};
  std::atomic<uint32_t>       fileBytes_{0};
};
