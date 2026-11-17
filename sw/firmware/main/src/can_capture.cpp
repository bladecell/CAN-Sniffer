#include "can_capture.hpp"

#include <dirent.h>
#include <sys/stat.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include "can_capture_core.hpp"
#include "can_raw.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "sd_card.hpp"

namespace
{
const char* const TAG = "CAN_CAP";

constexpr char     kDir[]          = "/logs/captures";
constexpr uint32_t kSdWaitMs       = 1000;
constexpr size_t   kQueueBytes     = 6144;  // between the drain task and the SD task: about 100 rows
constexpr size_t   kWriteBytes     = 4096;
constexpr size_t   kFlushBytes     = 2048;
constexpr uint32_t kFlushMs        = 1000;
constexpr uint32_t kTaskStopWaitMs = 3000;

enum class State : int { idle, capturing, ending };
enum class End : int { none, stop, stream, limit, noSd, sdError };

SemaphoreHandle_t mutex()  // start/stop/list serialisation; static storage, no allocation that can fail
{
  static StaticSemaphore_t buf;
  static SemaphoreHandle_t h = xSemaphoreCreateMutexStatic(&buf);
  return h;
}

std::atomic<int>      gState{static_cast<int>(State::idle)};
std::atomic<int>      gEnd{static_cast<int>(End::none)};
std::atomic<bool>     gStop{false};
std::atomic<bool>     gTaskRunning{false};
std::atomic<int>      gInObserver{0};
std::atomic<uint32_t> gRows{0};
std::atomic<uint32_t> gBytes{0};
std::atomic<uint32_t> gDropped{0};
char                  gName[16];  // written by start() with the state idle, read by the task and status()
StreamBufferHandle_t  gQueue = nullptr;
char*                 gWrite = nullptr;
can_capture::RowWriter gWriter;     // used by the drain task only
char                   gRow[2304];  // 35 frames at most per batch, 64 bytes a row

// The drain task. Formats the batch into whole rows and queues them; a full queue drops the whole batch and counts its rows.
void onBatch(const uint8_t* data, size_t len, size_t frames)
{
  gInObserver.fetch_add(1);
  if (gState.load() == static_cast<int>(State::capturing) && gQueue != nullptr)
  {
    size_t       rows = 0;
    const size_t w    = gWriter.addBatch(data, len, gRow, sizeof(gRow), rows);
    if (w == 0)
    {
      gDropped.fetch_add(static_cast<uint32_t>(frames > 0 ? frames : 1));
    }
    else if (xStreamBufferSpacesAvailable(gQueue) < w || xStreamBufferSend(gQueue, gRow, w, 0) != w)
    {
      gDropped.fetch_add(static_cast<uint32_t>(rows));
    }
    else
    {
      gRows.fetch_add(static_cast<uint32_t>(rows));
    }
  }
  gInObserver.fetch_sub(1);
}

bool filePath(char* out, size_t cap)
{
  return snprintf(out, cap, "%s/%s", kDir, gName) < static_cast<int>(cap);
}

// Appends gWrite[0..len) to the file. The SD lease is taken with a timeout and held for the write only.
End appendToFile(size_t len)
{
  auto& sd = SDCard::getInstance();
  auto  op = sd.acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);
  if (!op) { return End::noSd; }
  char path[48];
  if (!filePath(path, sizeof(path))) { return End::sdError; }
  if (sd.write_file(path, gWrite, len, true) != ESP_OK) { return End::sdError; }
  gBytes.fetch_add(static_cast<uint32_t>(len));
  return End::none;
}

// Writes the queued text, whole rows only and never past the size cap. False: the capture must end (`end` says why).
bool flush(size_t& len, End& end)
{
  if (len == 0) { return true; }
  const uint32_t room = can_capture::kMaxFileBytes > gBytes.load() ? can_capture::kMaxFileBytes - gBytes.load() : 0;
  size_t         n    = len;
  if (n > room)
  {
    n = room;
    while (n > 0 && gWrite[n - 1] != '\n') { n--; }
    end = End::limit;  // the rows past the cap are not written
  }
  if (n > 0)
  {
    const End e = appendToFile(n);
    if (e != End::none) { end = e; }
  }
  len = 0;
  return end == End::none;
}

void captureTask(void*)
{
  size_t   len       = 0;
  uint32_t lastFlush = static_cast<uint32_t>(pdTICKS_TO_MS(xTaskGetTickCount()));
  End      end       = End::none;
  while (end == End::none)
  {
    if (gStop.load() || !CanRaw::getInstance().isActive())
    {
      end = gStop.load() ? End::stop : End::stream;
      break;
    }
    len += xStreamBufferReceive(gQueue, gWrite + len, kWriteBytes - len, pdMS_TO_TICKS(250));
    const uint32_t now = static_cast<uint32_t>(pdTICKS_TO_MS(xTaskGetTickCount()));
    if (len >= kFlushBytes || (len > 0 && static_cast<uint32_t>(now - lastFlush) >= kFlushMs))
    {
      lastFlush = now;
      (void)flush(len, end);
    }
  }
  // No more rows are queued from here; wait out a batch the drain task is formatting.
  gState.store(static_cast<int>(State::ending));
  CanRaw::getInstance().setBatchObserver(nullptr);
  for (int i = 0; i < 200 && gInObserver.load() != 0; i++) { vTaskDelay(1); }
  if (end == End::stop || end == End::stream)  // an error or the cap leaves the rest unwritten
  {
    for (;;)
    {
      len += xStreamBufferReceive(gQueue, gWrite + len, kWriteBytes - len, 0);
      if (len == 0 || !flush(len, end)) { break; }
    }
  }
  gEnd.store(static_cast<int>(end));
  if (end == End::noSd || end == End::sdError) { ESP_LOGW(TAG, "Capture %s ended on a card error", gName); }
  else
  {
    ESP_LOGI(TAG, "Capture %s ended: %u rows, %u bytes, %u dropped", gName, static_cast<unsigned>(gRows.load()),
             static_cast<unsigned>(gBytes.load()), static_cast<unsigned>(gDropped.load()));
  }
  vStreamBufferDelete(gQueue);
  gQueue = nullptr;
  free(gWrite);
  gWrite = nullptr;
  gState.store(static_cast<int>(State::idle));
  gTaskRunning.store(false);
  vTaskDelete(nullptr);
}

struct Guard
{
  bool held;
  explicit Guard(uint32_t ms) : held(xSemaphoreTake(mutex(), pdMS_TO_TICKS(ms)) == pdTRUE) {}
  ~Guard()
  {
    if (held) { xSemaphoreGive(mutex()); }
  }
};
}  // namespace

namespace can_capture_service
{
bool pathFor(const char* name, char* out, size_t cap)
{
  uint32_t idx = 0;
  if (!can_capture::parseName(name, idx) || out == nullptr) { return false; }
  return snprintf(out, cap, "%s/%s", kDir, name) < static_cast<int>(cap);
}

Start start()
{
  Guard g(kSdWaitMs);
  if (!g.held) { return Start::noCard; }
  if (gTaskRunning.load()) { return Start::running; }
  if (!CanRaw::getInstance().isActive()) { return Start::streamOff; }

  auto& sd = SDCard::getInstance();
  auto  op = sd.acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);
  if (!op) { return Start::noCard; }
  if (sd.create_directory("/logs") != ESP_OK || sd.create_directory(kDir) != ESP_OK) { return Start::cardError; }

  char dirPath[64];
  if (sd.get_absolute_path(kDir, dirPath, sizeof(dirPath)) != ESP_OK) { return Start::cardError; }
  DIR* dir = opendir(dirPath);
  if (dir == nullptr) { return Start::cardError; }
  can_capture::NameScanner scanner;
  size_t                   seen = 0;
  while (const dirent* e = readdir(dir))
  {
    scanner.add(e->d_name);
    if (++seen >= can_capture::kMaxScan) { break; }
  }
  closedir(dir);
  uint32_t index = 0;
  if (seen >= can_capture::kMaxScan || !scanner.next(index)) { return Start::cardError; }  // never risk reusing a number

  char path[48];
  if (!can_capture::formatName(gName, sizeof(gName), index) || snprintf(path, sizeof(path), "%s/%s", kDir, gName) >= static_cast<int>(sizeof(path)))
  {
    return Start::cardError;
  }
  struct stat st;
  if (sd.get_file_stat(path, &st) == ESP_OK) { return Start::cardError; }  // exists after all: do not overwrite
  if (sd.write_file(path, can_capture::kHeader, sizeof(can_capture::kHeader) - 1, false) != ESP_OK) { return Start::cardError; }

  gQueue = xStreamBufferCreate(kQueueBytes, 1);
  gWrite = static_cast<char*>(malloc(kWriteBytes));
  if (gQueue == nullptr || gWrite == nullptr)
  {
    if (gQueue != nullptr) { vStreamBufferDelete(gQueue); gQueue = nullptr; }
    free(gWrite);
    gWrite = nullptr;
    (void)sd.delete_file(path);
    return Start::noMemory;
  }
  gWriter.reset();
  gRows.store(0);
  gBytes.store(static_cast<uint32_t>(sizeof(can_capture::kHeader) - 1));
  gDropped.store(0);
  gEnd.store(static_cast<int>(End::none));
  gStop.store(false);
  gState.store(static_cast<int>(State::capturing));
  gTaskRunning.store(true);
  CanRaw::getInstance().setBatchObserver(onBatch);
  if (xTaskCreate(captureTask, "can_cap", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr) != pdPASS)
  {
    CanRaw::getInstance().setBatchObserver(nullptr);
    for (int i = 0; i < 200 && gInObserver.load() != 0; i++) { vTaskDelay(1); }
    gState.store(static_cast<int>(State::idle));
    gTaskRunning.store(false);
    vStreamBufferDelete(gQueue);
    gQueue = nullptr;
    free(gWrite);
    gWrite = nullptr;
    (void)sd.delete_file(path);
    return Start::noMemory;
  }
  ESP_LOGI(TAG, "Capture %s started", gName);
  return Start::started;
}

esp_err_t stop()
{
  gStop.store(true);
  for (uint32_t waited = 0; gTaskRunning.load() && waited < kTaskStopWaitMs; waited += 10) { vTaskDelay(pdMS_TO_TICKS(10)); }
  return gTaskRunning.load() ? ESP_ERR_TIMEOUT : ESP_OK;
}

Status status()
{
  Status s{};
  s.running = gTaskRunning.load();
  std::memcpy(s.file, gName, sizeof(s.file) - 1);  // a torn read is only possible while start() rewrites it, with the task off
  s.rows    = gRows.load();
  s.bytes   = gBytes.load();
  s.dropped = gDropped.load();
  if (!s.running)
  {
    switch (static_cast<End>(gEnd.load()))
    {
      case End::stop: s.ended = "stop"; break;
      case End::stream: s.ended = "stream"; break;
      case End::limit: s.ended = "limit"; break;
      case End::noSd: s.ended = "error"; s.error = "NO_SD"; break;
      case End::sdError: s.ended = "error"; s.error = "SD_ERROR"; break;
      default: break;
    }
  }
  return s;
}

esp_err_t list(FileInfo* out, size_t max, size_t& count, bool& truncated)
{
  count     = 0;
  truncated = false;
  if (out == nullptr) { return ESP_ERR_INVALID_ARG; }
  auto&                      sd = SDCard::getInstance();
  auto                       op = sd.acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);
  if (!op) { return op.status() == ESP_OK ? ESP_ERR_INVALID_STATE : op.status(); }
  char      dirPath[64];
  esp_err_t err = sd.get_absolute_path(kDir, dirPath, sizeof(dirPath));
  if (err != ESP_OK) { return err; }
  DIR* dir = opendir(dirPath);
  if (dir == nullptr) { return errno == ENOENT ? ESP_OK : ESP_FAIL; }  // no captures yet
  can_capture::NewestNames newest;
  size_t                   seen = 0;
  while (const dirent* e = readdir(dir))
  {
    newest.add(e->d_name);
    if (++seen >= can_capture::kMaxScan) { truncated = true; break; }
  }
  closedir(dir);
  truncated = truncated || newest.truncated();
  for (size_t i = 0; i < newest.count() && count < max; ++i)
  {
    FileInfo fi{};
    char     path[48];
    struct stat st;
    if (!can_capture::formatName(fi.name, sizeof(fi.name), newest.at(i)) || !pathFor(fi.name, path, sizeof(path))) { continue; }
    if (sd.get_file_stat(path, &st) != ESP_OK) { continue; }  // removed meanwhile
    fi.bytes   = st.st_size > 0 ? static_cast<uint32_t>(st.st_size) : 0;
    out[count++] = fi;
  }
  return ESP_OK;
}

Remove remove(const char* name)
{
  char path[48];
  if (!pathFor(name, path, sizeof(path))) { return Remove::badName; }
  if (gTaskRunning.load() && std::strcmp(name, gName) == 0) { return Remove::inUse; }
  auto& sd = SDCard::getInstance();
  auto  op = sd.acquire_operation(pdMS_TO_TICKS(kSdWaitMs), true);
  if (!op) { return Remove::noCard; }
  const esp_err_t err = sd.delete_file(path);
  if (err == ESP_OK) { return Remove::removed; }
  return err == ESP_ERR_NOT_FOUND ? Remove::notFound : Remove::failed;
}
}  // namespace can_capture_service
