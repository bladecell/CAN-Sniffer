#include "recorder.hpp"

#include <dirent.h>
#include <sys/stat.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <new>
#include <string>

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "obd2.hpp"
#include "recorder_queue.hpp"
#include "sd_card.hpp"

using namespace recorder_core;

namespace
{
constexpr char     TAG[]            = "RECORDER";
constexpr size_t   kQueueDepth      = 256;
constexpr uint32_t kStackBytes      = 6144;  // opendir/readdir, cJSON, float snprintf and PidMeta share it
constexpr uint32_t kDefCheckMs      = 10000;  // how often the PID definitions are re-read to spot a change
constexpr uint32_t kStopWindowMs    = 5000;
constexpr uint32_t kSdWaitMs        = 250;
constexpr uint32_t kStartWaitMs     = 3000;  // start waits this long for the first file; a slower open still counts as started
constexpr size_t   kMaxScanEntries  = 20000;  // writer: once per boot, a hit refuses to record
constexpr size_t   kMaxListEntries  = 2000;   // listing: short lease so the writer is not starved

uint32_t nowMs()
{
  return pdTICKS_TO_MS(xTaskGetTickCount());  // same base as PIDData_t::lastUpdated
}

struct PidMeta
{
  uint16_t pid;
  uint16_t minSpacingMs;
  bool     known;
  char     name[32];
  char     unit[16];
  char     formula[96];
};

void copyText(char* dst, size_t cap, const std::string& src)
{
  (void)snprintf(dst, cap, "%s", src.c_str());
}

void readDef(const PidSel& sel, PidMeta& out)
{
  out = {};
  out.pid          = sel.pid;
  out.minSpacingMs = sel.minSpacingMs;
  PIDDefinitionData d;
  if (OBD2::getInstance().getDef(sel.pid, d) != ESP_OK) { return; }
  out.known = true;
  copyText(out.name, sizeof(out.name), d.name);
  copyText(out.unit, sizeof(out.unit), d.unit);
  copyText(out.formula, sizeof(out.formula), d.formula);
}

uint32_t hashMeta(const PidMeta& m, uint32_t h)
{
  h = fnv1a32(reinterpret_cast<const char*>(&m.pid), sizeof(m.pid), h);
  h = fnv1a32(reinterpret_cast<const char*>(&m.known), sizeof(m.known), h);
  h = fnv1a32(m.name, strlen(m.name) + 1, h);
  h = fnv1a32(m.unit, strlen(m.unit) + 1, h);
  return fnv1a32(m.formula, strlen(m.formula) + 1, h);
}

bool defaultPids(PidSel* out, size_t& n)
{
  auto& obd = OBD2::getInstance();
  n         = 0;
  for (const uint16_t pid : obd.getPIDs())
  {
    if (n == kMaxPids) { break; }
    if (obd.isSup(pid) && obd.getUpdateInterval(pid) != static_cast<uint16_t>(UPDATE_DISABLED)) { out[n++] = {pid, 0}; }
  }
  return n > 0;
}
}  // namespace

struct Recorder::Session
{
  RowQueue queue;
  PidFilter filter;
  char*    line = nullptr;  // kBatchBytes, filled by the writer
  size_t   lineLen = 0;
  uint32_t pendingRows = 0;
  uint32_t writerDropped = 0;
  PidMeta  meta[kMaxPids];
  size_t   metaCount = 0;
  uint32_t hash = 0;
  bool     fileOpen = false;
  bool     degraded = false;  // the last flush failed: retries are throttled
  uint32_t lastDefCheckMs = 0;
  bool     reopen = false;  // a write failed: the next good write starts a new file
  uint32_t fileIndex = 0;
  uint32_t fileRows = 0;
  uint32_t fileStartMs = 0;
  uint64_t fileBytes = 0;
  char     csvName[24] = {};
  char     jsonName[24] = {};
  char     csvPath[48] = {};
  char     jsonPath[48] = {};
  bool     enabledAutoPoll = false;
  std::atomic<int> firstOpen{0};  // 0 pending, 1 the first file is open, 2 it could not be opened
  std::atomic<bool> stopReq{false};
  std::atomic<bool> finished{false};

  ~Session() { delete[] line; }
};

Recorder& Recorder::getInstance()
{
  static Recorder instance;
  return instance;
}

Recorder::Recorder()
{
  lock_ = xSemaphoreCreateMutex();
}

namespace
{
struct Lock
{
  explicit Lock(SemaphoreHandle_t h) : h_(h) { held_ = h_ && xSemaphoreTake(h_, pdMS_TO_TICKS(kStopWindowMs + 2000)) == pdTRUE; }
  ~Lock() { if (held_) { xSemaphoreGive(h_); } }
  Lock(const Lock&)            = delete;
  Lock& operator=(const Lock&) = delete;
  explicit operator bool() const { return held_; }
  SemaphoreHandle_t h_;
  bool              held_;
};
}  // namespace

esp_err_t Recorder::init()
{
  Lock lock(lock_);
  if (!lock) { return ESP_ERR_TIMEOUT; }
  if (initialized_) { return ESP_OK; }
  const uint32_t gen = ++gen_;
  OBD2::getInstance().subscribe([gen](uint16_t pid) { Recorder::getInstance().onPid(pid, gen); });
  initialized_ = true;
  return ESP_OK;
}

esp_err_t Recorder::deinit()
{
  Lock lock(lock_);
  if (!lock) { return ESP_ERR_TIMEOUT; }
  const esp_err_t err = stopLocked();
  if (err != ESP_OK) { return err; }
  ++gen_;  // the OBD subscription cannot be removed; this makes the old callback a no-op
  initialized_ = false;
  return ESP_OK;
}

// Runs on the OBD callback thread. Takes only the short PID-map lock inside getData(); never waits on the queue.
void Recorder::onPid(uint16_t pid, uint32_t gen)
{
  if (gen != gen_.load() || !gate_.enter()) { return; }
  Session* s = session_;
  PIDData_t d;
  if (s != nullptr && OBD2::getInstance().getData(pid, d) == ESP_OK && s->filter.accept(pid, d.lastUpdated))
  {
    s->queue.push({d.lastUpdated, pid, d.value, d.isValid});
  }
  gate_.leave();
}

esp_err_t Recorder::start(const Config& config)
{
  Lock lock(lock_);
  if (!lock) { return ESP_ERR_TIMEOUT; }
  if (!initialized_) { return ESP_ERR_INVALID_STATE; }
  if (session_ != nullptr) { return ESP_ERR_NOT_FINISHED; }  // live, also when paused in Error: stop first

  PidSel          pids[kMaxPids];
  size_t          n = 0;
  if (config.pidCount == 0)
  {
    if (!defaultPids(pids, n)) { return ESP_ERR_INVALID_ARG; }
  }
  else if (config.pidCount > kMaxPids) { return ESP_ERR_INVALID_ARG; }
  else
  {
    n = config.pidCount;
    memcpy(pids, config.pids, n * sizeof(PidSel));
  }
  if (validatePidList(pids, n, [](uint16_t pid) { return OBD2::getInstance().pidExists(pid); }) != ListError::Ok)
  {
    return ESP_ERR_INVALID_ARG;
  }

  lastError_.store(ESP_OK);
  trip_.store(0);
  rows_.store(0);
  dropped_.store(0);
  fileBytes_.store(0);

  if (!SDCard::getInstance().is_mounted())
  {
    lastError_.store(ESP_ERR_NOT_FOUND);
    if (state_.get() == State::Idle) { state_.move(State::Idle, State::Error); }
    return ESP_ERR_NOT_FOUND;
  }

  Session* s = new (std::nothrow) Session();
  if (s != nullptr) { s->line = new (std::nothrow) char[kBatchBytes]; }
  if (s == nullptr || s->line == nullptr || !s->queue.create(kQueueDepth) || !s->filter.set(pids, n))
  {
    delete s;
    lastError_.store(ESP_ERR_NO_MEM);
    return ESP_ERR_NO_MEM;
  }

  auto& obd = OBD2::getInstance();
  if (!obd.isAutoPoll())
  {
    s->enabledAutoPoll = obd.setAutoPoll(true) == ESP_OK;  // remember: only then do we turn it off again
    if (!s->enabledAutoPoll) { ESP_LOGW(TAG, "Could not enable auto poll; recording only what is polled"); }
  }

  if (!state_.move(State::Idle, State::Recording) && !state_.move(State::Error, State::Recording))
  {
    if (s->enabledAutoPoll) { obd.setAutoPoll(false); }
    delete s;
    return ESP_ERR_INVALID_STATE;
  }

  session_ = s;
  if (xTaskCreate(taskEntry, "recorder", kStackBytes, s, tskIDLE_PRIORITY, nullptr) != pdPASS)
  {
    session_ = nullptr;
    state_.move(State::Recording, State::Error);
    lastError_.store(ESP_ERR_NO_MEM);
    if (s->enabledAutoPoll) { obd.setAutoPoll(false); }
    delete s;
    return ESP_ERR_NO_MEM;
  }
  gate_.open();
  ESP_LOGI(TAG, "Recording %u PIDs", static_cast<unsigned>(n));

  // The writer opens the first file at once: wait briefly so a card that cannot be written is reported, not hidden.
  const uint32_t t0 = nowMs();
  while (s->firstOpen.load() == 0 && static_cast<uint32_t>(nowMs() - t0) < kStartWaitMs) { vTaskDelay(pdMS_TO_TICKS(20)); }
  if (s->firstOpen.load() == 2)
  {
    const esp_err_t cause = lastError_.load();
    if (stopLocked() != ESP_OK) { return ESP_ERR_TIMEOUT; }  // writer stuck on the card: state stays Stopping, stop again
    lastError_.store(cause);
    state_.move(State::Idle, State::Error);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t Recorder::stop()
{
  if (stopInFlight_.exchange(true)) { return ESP_ERR_TIMEOUT; }  // a second caller must not park another worker for 5 s
  esp_err_t err = ESP_ERR_TIMEOUT;
  {
    Lock lock(lock_);
    if (lock) { err = stopLocked(); }
  }
  stopInFlight_.store(false);
  return err;
}

esp_err_t Recorder::stopLocked()
{
  Session* s = session_;
  if (s == nullptr)
  {
    state_.move(State::Error, State::Idle);
    return ESP_OK;
  }
  for (int i = 0; i < 3; ++i)
  {
    if (state_.move(State::Recording, State::Stopping) || state_.move(State::Error, State::Stopping)) { break; }
  }
  gate_.close();
  const uint32_t t0 = nowMs();
  auto           elapsed = [t0] { return static_cast<uint32_t>(nowMs() - t0); };
  while (!gate_.idle() && elapsed() < kStopWindowMs) { vTaskDelay(1); }
  if (!gate_.idle()) { return ESP_ERR_TIMEOUT; }  // a callback still holds the queue: free nothing
  s->stopReq.store(true);
  while (!s->finished.load() && elapsed() < kStopWindowMs) { vTaskDelay(pdMS_TO_TICKS(10)); }
  if (!s->finished.load()) { return ESP_ERR_TIMEOUT; }  // the writer still owns the session: retry later

  // Only undo what we did: if someone turned auto poll off meanwhile there is nothing to restore.
  if (s->enabledAutoPoll && OBD2::getInstance().isAutoPoll()) { OBD2::getInstance().setAutoPoll(false); }
  session_ = nullptr;
  delete s;
  state_.move(State::Stopping, State::Idle);
  return ESP_OK;
}

Recorder::Status Recorder::getStatus() const
{
  return {state_.get(), lastError_.load(), trip_.load(), rows_.load(), dropped_.load(), fileBytes_.load()};
}

void Recorder::taskEntry(void* arg)
{
  Session& s = *static_cast<Session*>(arg);
  getInstance().writerLoop(s);
  s.finished.store(true);  // last access to the session
  vTaskDelete(nullptr);
}

void Recorder::writerLoop(Session& s)
{
  {
    auto op = SDCard::getInstance().acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);
    const esp_err_t err = op ? openFile(s) : (op.status() == ESP_OK ? ESP_ERR_INVALID_STATE : op.status());
    if (err != ESP_OK)
    {
      fail(err);  // Error with the reason; start() tears the session down if it is still waiting, else the loop retries
      s.degraded = true;
      s.firstOpen.store(2);
    }
    else { s.firstOpen.store(1); }
  }
  uint32_t lastFlush = nowMs();
  for (;;)
  {
    const bool stopping = s.stopReq.load();
    Row        row;
    if (s.queue.pop(row, stopping ? 0 : pdMS_TO_TICKS(200)))
    {
      char* at = s.line + s.lineLen;
      const size_t n = s.lineLen + kMaxRowBytes <= kBatchBytes ? formatRow(at, kBatchBytes - s.lineLen, row) : 0;
      if (n == 0) { ++s.writerDropped; }  // the card is stuck and the buffer is full
      else
      {
        s.lineLen += n;
        ++s.pendingRows;
      }
    }
    else if (stopping) { break; }
    dropped_.store(s.writerDropped + s.queue.dropped());

    const uint32_t now = nowMs();
    if (shouldFlush(s.lineLen, now, lastFlush) && mayFlush(s.degraded, now, lastFlush))
    {
      flush(s);
      lastFlush = now;
    }
  }
  if (!flush(s)) { s.writerDropped += s.pendingRows; }  // one last attempt; the rest is lost, not retried
  if (s.fileOpen)
  {
    auto op = SDCard::getInstance().acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);
    if (op) { writeSidecar(s, true); }
    s.fileOpen = false;
  }
  dropped_.store(s.writerDropped + s.queue.dropped());
}

bool Recorder::fail(esp_err_t err)
{
  lastError_.store(err);
  if (state_.move(State::Recording, State::Error)) { ESP_LOGW(TAG, "Recording paused: %s", esp_err_to_name(err)); }
  return false;
}

bool Recorder::flush(Session& s)
{
  if (s.lineLen == 0) { return true; }
  const bool ok = flushBatch(s);
  s.degraded    = !ok;
  return ok;
}

bool Recorder::flushBatch(Session& s)
{
  auto& sd = SDCard::getInstance();
  auto  op = sd.acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);
  if (!op)
  {
    // Lease busy (a listing or another SD user): keep the batch and retry, do not report an error.
    if (op.status() == ESP_ERR_TIMEOUT) { return false; }
    return fail(op.status() == ESP_OK ? ESP_ERR_INVALID_STATE : op.status());
  }

  bool defsChanged = false;
  if (s.fileOpen && static_cast<uint32_t>(nowMs() - s.lastDefCheckMs) >= kDefCheckMs)
  {
    s.lastDefCheckMs = nowMs();
    defsChanged      = defHash(s) != s.hash;
  }
  if (s.fileOpen && (s.reopen || defsChanged || shouldRotate(s.fileBytes, s.lineLen)))
  {
    writeSidecar(s, true);  // best effort for the file being left
    s.fileOpen = false;
  }
  if (!s.fileOpen)
  {
    const esp_err_t err = openFile(s);
    if (err != ESP_OK) { return fail(err); }
  }
  const esp_err_t err = sd.write_file(s.csvPath, s.line, s.lineLen, true);
  if (err != ESP_OK)
  {
    s.reopen = true;  // part of the batch may be on the card
    return fail(err);
  }
  s.fileBytes += s.lineLen;
  s.fileRows += s.pendingRows;
  rows_.fetch_add(s.pendingRows);
  fileBytes_.store(static_cast<uint32_t>(s.fileBytes));
  s.lineLen     = 0;
  s.pendingRows = 0;
  if (state_.move(State::Error, State::Recording)) { ESP_LOGI(TAG, "Recording resumed"); }
  return true;
}

// Caller holds the SD operation.
esp_err_t Recorder::openFile(Session& s)
{
  auto& sd = SDCard::getInstance();
  s.reopen = false;
  esp_err_t err = sd.create_directory("/logs");
  if (err == ESP_OK) { err = sd.create_directory(kTripDir); }
  if (err != ESP_OK) { return err; }

  if (nextTrip_ == 0)  // scan once per boot; later trips just count up
  {
    char dirPath[64];
    err = sd.get_absolute_path(kTripDir, dirPath, sizeof(dirPath));
    if (err != ESP_OK) { return err; }
    DIR* dir = opendir(dirPath);
    if (dir == nullptr) { return ESP_FAIL; }
    TripScanner scanner;
    size_t      seen = 0;
    while (const dirent* entry = readdir(dir))
    {
      scanner.add(entry->d_name);
      if (++seen >= kMaxScanEntries) { break; }
    }
    closedir(dir);
    uint32_t first = 0;
    // A capped scan may have missed the highest index: refuse rather than risk reusing a trip number.
    if (seen >= kMaxScanEntries || !scanner.next(first)) { return ESP_ERR_INVALID_SIZE; }
    nextTrip_ = first;
  }

  uint32_t index = 0;
  for (int attempt = 0;; ++attempt)  // never overwrite: skip any index whose file already exists (card swapped)
  {
    index = nextTrip_;
    if (index > kMaxTripIndex || attempt == 8) { return ESP_ERR_INVALID_SIZE; }
    if (!formatTripName(s.csvName, sizeof(s.csvName), index, false) ||
        !formatTripName(s.jsonName, sizeof(s.jsonName), index, true) ||
        snprintf(s.csvPath, sizeof(s.csvPath), "%s/%s", kTripDir, s.csvName) >= static_cast<int>(sizeof(s.csvPath)) ||
        snprintf(s.jsonPath, sizeof(s.jsonPath), "%s/%s", kTripDir, s.jsonName) >= static_cast<int>(sizeof(s.jsonPath)))
    {
      return ESP_ERR_INVALID_SIZE;
    }
    struct stat     st;
    const esp_err_t probe = sd.get_file_stat(s.csvPath, &st);
    if (probe == ESP_ERR_NOT_FOUND) { break; }       // free
    if (probe != ESP_OK) { return probe; }           // unknown: refuse rather than guess
    ++nextTrip_;
  }
  err = sd.write_file(s.csvPath, kCsvHeader, sizeof(kCsvHeader) - 1, false);
  if (err != ESP_OK) { return err; }
  ++nextTrip_;

  snapshotMeta(s);
  s.fileIndex   = index;
  s.fileRows    = 0;
  s.fileStartMs = nowMs();
  s.fileBytes   = sizeof(kCsvHeader) - 1;
  s.fileOpen    = true;
  trip_.store(index);
  fileBytes_.store(static_cast<uint32_t>(s.fileBytes));
  writeSidecar(s, false);  // a power cut leaves a sidecar with complete=false
  ESP_LOGI(TAG, "Trip file %s", s.csvName);
  return ESP_OK;
}

void Recorder::snapshotMeta(Session& s)
{
  s.metaCount = s.filter.size();
  s.hash      = 0x811C9DC5u;
  for (size_t i = 0; i < s.metaCount; ++i)
  {
    readDef(s.filter.at(i), s.meta[i]);
    s.hash = hashMeta(s.meta[i], s.hash);
  }
}

uint32_t Recorder::defHash(const Session& s) const
{
  uint32_t h = 0x811C9DC5u;
  PidMeta  m;
  for (size_t i = 0; i < s.filter.size(); ++i)
  {
    readDef(s.filter.at(i), m);
    h = hashMeta(m, h);
  }
  return h;
}

// Caller holds the SD operation.
esp_err_t Recorder::writeSidecar(Session& s, bool complete)
{
  cJSON* root = cJSON_CreateObject();
  cJSON* pids = root ? cJSON_AddArrayToObject(root, "pids") : nullptr;
  bool   ok   = pids != nullptr;
  const std::string vin = OBD2::getInstance().getVIN();
  const esp_app_desc_t* app = esp_app_get_description();
  ok = ok && cJSON_AddNumberToObject(root, "version", 1) && cJSON_AddStringToObject(root, "file", s.csvName) &&
       cJSON_AddNumberToObject(root, "trip", s.fileIndex) &&
       cJSON_AddStringToObject(root, "fw", app ? app->version : "") &&
       cJSON_AddNumberToObject(root, "start_ms", s.fileStartMs) && cJSON_AddNumberToObject(root, "end_ms", nowMs()) &&
       cJSON_AddNumberToObject(root, "rows", s.fileRows) &&
       cJSON_AddNumberToObject(root, "dropped", s.writerDropped + s.queue.dropped()) &&
       cJSON_AddBoolToObject(root, "complete", complete);
  if (ok && !vin.empty()) { ok = cJSON_AddStringToObject(root, "vin", vin.c_str()) != nullptr; }
  for (size_t i = 0; ok && i < s.metaCount; ++i)
  {
    const PidMeta& m    = s.meta[i];
    cJSON*         item = cJSON_CreateObject();
    if (item == nullptr) { ok = false; break; }
    cJSON_AddItemToArray(pids, item);
    ok = cJSON_AddNumberToObject(item, "pid", m.pid) && cJSON_AddStringToObject(item, "name", m.name) &&
         cJSON_AddStringToObject(item, "unit", m.unit) && cJSON_AddStringToObject(item, "formula", m.formula) &&
         cJSON_AddNumberToObject(item, "min_interval_ms", m.minSpacingMs);
  }
  char* text = ok ? cJSON_PrintUnformatted(root) : nullptr;
  cJSON_Delete(root);
  if (text == nullptr) { return ESP_ERR_NO_MEM; }
  const esp_err_t err = SDCard::getInstance().write_file(s.jsonPath, text, strlen(text), false);
  cJSON_free(text);
  if (err != ESP_OK) { ESP_LOGW(TAG, "Sidecar write failed: %s", esp_err_to_name(err)); }
  return err;
}

esp_err_t Recorder::listTrips(TripInfo* out, size_t max, size_t& count, bool& truncated)
{
  count     = 0;
  truncated = false;
  if (out == nullptr) { return ESP_ERR_INVALID_ARG; }
  auto&       sd = SDCard::getInstance();
  NewestTrips newest;
  {
    auto op = sd.acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);  // held for the scan only, then per file below
    if (!op) { return op.status() == ESP_OK ? ESP_ERR_INVALID_STATE : op.status(); }
    char      dirPath[64];
    esp_err_t err = sd.get_absolute_path(kTripDir, dirPath, sizeof(dirPath));
    if (err != ESP_OK) { return err; }
    DIR* dir = opendir(dirPath);
    if (dir == nullptr) { return errno == ENOENT ? ESP_OK : ESP_FAIL; }  // no trips yet
    size_t seen = 0;
    while (const dirent* entry = readdir(dir))
    {
      newest.addName(entry->d_name);
      if (++seen >= kMaxListEntries) { truncated = true; break; }
    }
    closedir(dir);
  }
  truncated = truncated || newest.truncated();
  for (size_t i = 0; i < newest.count() && count < max; ++i)
  {
    TripInfo t{newest.at(i), 0, false};
    char     name[24];
    char     path[48];
    struct stat st;
    if (!formatTripName(name, sizeof(name), t.index, false) ||
        snprintf(path, sizeof(path), "%s/%s", kTripDir, name) >= static_cast<int>(sizeof(path)))
    {
      continue;
    }
    if (sd.get_file_stat(path, &st) == ESP_OK) { t.bytes = st.st_size > 0 ? static_cast<uint32_t>(st.st_size) : 0; }
    if (formatTripName(name, sizeof(name), t.index, true) &&
        snprintf(path, sizeof(path), "%s/%s", kTripDir, name) < static_cast<int>(sizeof(path)))
    {
      t.sidecar = sd.get_file_stat(path, &st) == ESP_OK;
    }
    out[count++] = t;
  }
  return ESP_OK;
}
