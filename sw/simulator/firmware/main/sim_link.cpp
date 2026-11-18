#include "sim_commands.hpp"
#include "sim_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <inttypes.h>
#include "driver/usb_serial_jtag.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace sim {
namespace {
constexpr size_t lineMax = 4096, logMax = 48 * 1024, entryMax = 1280;
constexpr char header[] = "SIM1\n";
constexpr char partition[] = "sim_profile", nameSpace[] = "sim", blob[] = "profile";
constexpr size_t streamDepth = 64;
struct Telemetry { Frame frame; bool tx, unmatched; };
StaticQueue_t streamBuffer;
uint8_t streamStorage[sizeof(Telemetry) * streamDepth];
QueueHandle_t streamQueue;
uint32_t streamRate, drops, dropsReported, streamTokens;
int64_t streamRefill;
uint32_t usbWriteFailures;
bool engineReady, nvsReady;
const char* profileState = "empty";

struct Entry { uint32_t key; uint16_t len; };
char logData[logMax];
Entry entries[entryMax];
size_t logSize = sizeof(header) - 1, entryCount;
size_t reservedIndex;
uint32_t reservedKey;

bool reserveLog(uint32_t key, size_t size) {
  reservedKey = key;
  reservedIndex = entryCount;
  if (key != independent && key != ruleGroup)
    for (size_t i = 0; i < entryCount; ++i) if (entries[i].key == key) { reservedIndex = i; break; }
  const size_t available = logMax - logSize + (reservedIndex == entryCount ? 0 : entries[reservedIndex].len);
  return (entryCount < entryMax || reservedIndex < entryCount) && size + 1 <= available;
}

void removeEntry(size_t index) {
  size_t start = sizeof(header) - 1;
  for (size_t i = 0; i < index; ++i) start += entries[i].len;
  const size_t end = start + entries[index].len;
  std::memmove(logData + start, logData + end, logSize - end);
  logSize -= entries[index].len;
  --entryCount;
  std::memmove(entries + index, entries + index + 1, (entryCount - index) * sizeof(Entry));
}

void stage(const char* line, size_t size) {
  if (reservedIndex < entryCount) removeEntry(reservedIndex);
  std::memcpy(logData + logSize, line, size);
  logData[logSize + size] = '\n';
  entries[entryCount++] = {reservedKey, static_cast<uint16_t>(size + 1)};
  logSize += size + 1;
}

void resetLog() {
  entryCount = 0;
  logSize = sizeof(header) - 1;
  std::memcpy(logData, header, logSize);
}

esp_err_t writeProfile(bool erase) {
  if (!nvsReady) return ESP_ERR_INVALID_STATE;
  nvs_handle_t handle;
  auto err = nvs_open_from_partition(partition, nameSpace, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = erase ? nvs_erase_key(handle, blob) : nvs_set_blob(handle, blob, logData, logSize);
  if (erase && err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  return err;
}

bool space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
bool digit(char c) { return c >= '0' && c <= '9'; }
bool number(const char*& s) {
  const char* start = s;
  if (*s == '-') ++s;
  if (!digit(*s)) return false;
  if (*s++ != '0') while (digit(*s)) ++s;
  if (*s == '.') {
    ++s;
    if (!digit(*s)) return false;
    while (digit(*s)) ++s;
  }
  if (*s == 'e' || *s == 'E') {
    ++s;
    if (*s == '+' || *s == '-') ++s;
    if (!digit(*s)) return false;
    while (digit(*s)) ++s;
  }
  return s - start <= 63 && (*s == ',' || *s == '}' || *s == ']' || space(*s));
}

// Bound parser recursion and allocation before cJSON sees hostile input. cJSON handles JSON structure.
bool prescan(const char* s) {
  unsigned depth = 0, tokens = 0;
  while (*s) {
    const char c = *s++;
    if (space(c) || c == ':' || c == ',') continue;
    if (c == '}' || c == ']') {
      if (!depth) return false;
      --depth;
      continue;
    }
    if (++tokens > 360) return false;
    if (c == '{' || c == '[') {
      if (++depth > 8) return false;
    } else if (c == '"') {
      while (*s && *s != '"') {
        if (static_cast<unsigned char>(*s) < 0x20) return false;
        if (*s++ == '\\') {
          if (!*s) return false;
          // Decoded NUL would truncate values and object keys in cJSON's C strings.
          if (*s == 'u' && std::strncmp(s, "u0000", 5) == 0) return false;
          ++s;
        }
      }
      if (!*s++) return false;
    } else if (digit(c) || c == '-') {
      --s;
      if (!number(s)) return false;
    } else {
      --s;
      const char* literal = c == 't' ? "true" : c == 'f' ? "false" : c == 'n' ? "null" : nullptr;
      if (!literal || std::strncmp(s, literal, std::strlen(literal)) != 0) return false;
      s += std::strlen(literal);
    }
  }
  return depth == 0;
}

bool validTree(Json root) {
  if (cJSON_IsNumber(root)) return std::isfinite(root->valuedouble);
  if (cJSON_IsObject(root)) {
    for (Json a = root->child; a; a = a->next)
      for (Json b = a->next; b; b = b->next)
        if (std::strcmp(a->string, b->string) == 0) return false;
  }
  for (Json child = root->child; child; child = child->next)
    if (!validTree(child)) return false;
  return true;
}

using Document = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
Document parse(const char* line, size_t len) {
  Document doc{nullptr, cJSON_Delete};
  if (std::memchr(line, '\0', len) || !prescan(line)) return doc;
  doc.reset(cJSON_ParseWithLengthOpts(line, len + 1, nullptr, true));
  if (doc && (!cJSON_IsObject(doc.get()) || !validTree(doc.get()))) doc.reset();
  return doc;
}

bool replayLog(size_t length) {
  if (length < sizeof(header) - 1 || std::memcmp(logData, header, sizeof(header) - 1) != 0) return false;
  const int64_t now = esp_timer_get_time();
  while (logSize < length) {
    auto end = static_cast<char*>(std::memchr(logData + logSize, '\n', length - logSize));
    if (!end) return false;
    const size_t size = end - (logData + logSize);
    if (!size || size > lineMax) return false;
    *end = 0;
    const auto doc = parse(logData + logSize, size);
    *end = '\n';
    if (!doc) return false;
    const auto cmd = text(field(doc.get(), "cmd"));
    if (!cmd || edit(doc.get(), cmd, now, true, size, reserveLog) || reservedIndex != entryCount) return false;
    entries[entryCount++] = {reservedKey, static_cast<uint16_t>(size + 1)};
    logSize += size + 1;
    if (entryCount % 16 == 0) vTaskDelay(1);
  }
  return true;
}

void loadProfile() {
  resetLog();
  if (nvs_flash_init_partition(partition) != ESP_OK) {
    profileState = "nvs_error";
    return;
  }
  nvsReady = true;
  nvs_handle_t handle;
  auto err = nvs_open_from_partition(partition, nameSpace, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) return;
  if (err != ESP_OK) { profileState = "nvs_error"; return; }
  size_t count = sizeof(logData);
  err = nvs_get_blob(handle, blob, logData, &count);
  nvs_close(handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) return;
  if (err == ESP_OK && replayLog(count)) { profileState = "loaded"; return; }
  resetModel();
  resetLog();
  profileState = "invalid";
}

bool writeLine(const char* line, int len, size_t cap, bool reply = false) {
  if (len <= 0 || static_cast<size_t>(len) >= cap) return false;
  const bool ok = usb_serial_jtag_write_bytes(line, len, reply ? pdMS_TO_TICKS(30) : 0) == len;
  if (!ok) ++usbWriteFailures;
  return ok;
}

void reply(const char* seq, const char* extra, const char* error) {
  char out[640];
  const int n = error ? std::snprintf(out, sizeof(out), "{\"ok\":false%s,\"err\":\"%s\"}\n", seq, error) :
      std::snprintf(out, sizeof(out), "{\"ok\":true%s%s}\n", seq, extra);
  (void)writeLine(out, n, sizeof(out), true);
}

void command(const char* line, size_t size) {
  const auto doc = parse(line, size);
  if (!doc) { reply("", "", "invalid JSON"); return; }
  const auto root = doc.get();
  char seq[36]{};
  if (const auto value = field(root, "seq")) {
    constexpr double maxSeq = 9007199254740991.0;
    const double n = value->valuedouble;
    if (!cJSON_IsNumber(value) || n < -maxSeq || n > maxSeq || std::trunc(n) != n) {
      reply("", "", "invalid seq");
      return;
    }
    std::snprintf(seq, sizeof(seq), ",\"seq\":%" PRId64, static_cast<int64_t>(n));
  }
  const auto cmd = text(field(root, "cmd"));
  if (!cmd) { reply(seq, "", "missing cmd"); return; }
  char extra[500]{};
  const char* err = nullptr;
  if (std::strcmp(cmd, "hello") == 0 || std::strcmp(cmd, "stats") == 0) {
    lockEngine();
    const auto bus = can::stats();
    const auto work = engineStats();
    const auto bitrate = model.bitrate;
    const bool ack = model.ack;
    const auto lost = drops;
    const size_t ecus = model.ecuCount, rules = model.ruleCount, cyclic = model.cyclicCount;
    unlockEngine();
    std::snprintf(extra, sizeof(extra),
        ",\"protocol\":1,\"bitrate\":%" PRIu32 ",\"ack\":%s,\"canReady\":%s,\"profile\":\"%s\","
        "\"ecus\":%zu,\"rules\":%zu,\"cyclic\":%zu,\"rx\":%" PRIu32 ",\"tx\":%" PRIu32
        ",\"rxDropped\":%" PRIu32 ",\"rxInvalid\":%" PRIu32 ",\"txFailed\":%" PRIu32
        ",\"busResets\":%" PRIu32 ",\"unmatched\":%" PRIu32 ",\"jobsDropped\":%" PRIu32
        ",\"isoTimeouts\":%" PRIu32 ",\"streamDropped\":%" PRIu32 ",\"usbWriteFailures\":%" PRIu32
        ",\"logBytes\":%zu",
        bitrate, ack ? "true" : "false", bus.ready ? "true" : "false", profileState, ecus, rules, cyclic,
        work.rx, bus.tx, bus.rxDropped, bus.rxInvalid, bus.txFailed, bus.busOff, work.unmatched,
        work.jobsDropped, work.isoTimeouts, lost, usbWriteFailures, logSize);
  } else if (std::strcmp(cmd, "save") == 0) {
    const auto result = writeProfile(false);
    if (result == ESP_OK) profileState = "saved";
    else err = esp_err_to_name(result);
  } else if (std::strcmp(cmd, "reset") == 0) {
    const auto result = writeProfile(true);
    if (result != ESP_OK) err = esp_err_to_name(result);
    else {
      resetLog();
      profileState = "empty";
      lockEngine();
      resetModel();
      (void)can::deinit();
      if (engineReady) (void)can::configure(model.bitrate, model.ack);
      unlockEngine();
    }
  } else if (std::strcmp(cmd, "rule.clear") == 0) {
    lockEngine();
    clearRules();
    unlockEngine();
    for (size_t i = entryCount; i > 0; --i) if (entries[i - 1].key == ruleGroup) removeEntry(i - 1);
    profileState = "staged";
  } else if (std::strcmp(cmd, "stream") == 0) {
    const auto on = field(root, "on");
    uint32_t rate = 100;
    if (!cJSON_IsBool(on) || !optionalInt(root, "maxPerSec", 1, 1000, rate)) err = "invalid stream options";
    else {
      lockEngine();
      streamRate = cJSON_IsTrue(on) ? rate : 0;
      streamTokens = std::min<uint32_t>(rate, 32) * 1'000'000;
      streamRefill = esp_timer_get_time();
      xQueueReset(streamQueue);
      unlockEngine();
    }
  } else if (!engineReady) {
    err = "CAN engine unavailable";
  } else {
    char canon[64];
    lockEngine();
    err = edit(root, cmd, esp_timer_get_time(), false, size, reserveLog);
    if (!err && reservedKey == canKey) {
      const int n = std::snprintf(canon, sizeof(canon), "{\"cmd\":\"can.config\",\"bitrate\":%" PRIu32 ",\"ack\":%s}",
          model.bitrate, model.ack ? "true" : "false");
      line = canon;
      size = n;
    }
    unlockEngine();
    if (!err) { stage(line, size); profileState = "staged"; }
  }
  reply(seq, extra, err);
}

void drainStream() {
  Telemetry event;
  for (size_t n = 0; n < 24 && xQueueReceive(streamQueue, &event, 0) == pdTRUE; ++n) {
    const auto& f = event.frame;
    char bytes[33]{};
    size_t pos = 0;
    for (size_t i = 0; i < f.len && i < sizeof(f.data); ++i)
      pos += std::snprintf(bytes + pos, sizeof(bytes) - pos, "%s%u", i ? "," : "", f.data[i]);
    char out[220];
    const int len = std::snprintf(out, sizeof(out),
        "{\"ev\":\"frame\",\"dir\":\"%s\",\"id\":%" PRIu32 ",\"ext\":%s,\"data\":[%s],\"ts\":%" PRId64 "%s%s}\n",
        event.tx ? "tx" : "rx", f.addr.id, f.addr.ext ? "true" : "false", bytes, f.ts,
        f.rtr ? ",\"rtr\":true" : "", event.unmatched ? ",\"unmatched\":true" : "");
    if (!writeLine(out, len, sizeof(out))) {
      lockEngine();
      ++drops;
      unlockEngine();
    }
  }
  static int64_t nextReport;
  const auto now = esp_timer_get_time();
  if (now < nextReport) return;
  nextReport = now + 1'000'000;
  lockEngine();
  const uint32_t count = drops - dropsReported;
  const bool on = streamRate != 0;
  unlockEngine();
  if (!on || !count) return;
  char out[64];
  const int len = std::snprintf(out, sizeof(out), "{\"ev\":\"drop\",\"n\":%" PRIu32 "}\n", count);
  if (writeLine(out, len, sizeof(out))) dropsReported += count;
}

const char* resetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "usb";
    default: return "other";
  }
}

void runLink() {
  usb_serial_jtag_driver_config_t config{.tx_buffer_size = 4096, .rx_buffer_size = 4096};
  if (usb_serial_jtag_driver_install(&config) != ESP_OK) return;
  char boot[120];
  const int n = std::snprintf(boot, sizeof(boot), "{\"ev\":\"boot\",\"reset\":\"%s\",\"profile\":\"%s\"}\n", resetReason(), profileState);
  (void)writeLine(boot, n, sizeof(boot));
  static char line[lineMax + 1];
  size_t used = 0;
  bool oversized = false;
  for (;;) {
    unsigned char input[256];
    const int read = usb_serial_jtag_read_bytes(input, sizeof(input), pdMS_TO_TICKS(5));
    for (int i = 0; i < read; ++i) {
      const char c = input[i];
      if (c == '\n') {
        line[used] = 0;
        if (oversized) reply("", "", "line too long");
        else if (used && !(used == 1 && line[0] == '\r')) command(line, used);
        oversized = false;
        used = 0;
      } else if (!oversized) {
        if (used == lineMax) oversized = true;
        else line[used++] = c;
      }
    }
    drainStream();
    vTaskDelay(1);
  }
}
}

void streamFrame(const Frame& frame, bool tx, bool unmatched) {
  if (!streamRate || !streamQueue) return;
  const int64_t now = std::max(frame.ts, streamRefill);
  const uint32_t cap = std::min<uint32_t>(streamRate, 32) * 1'000'000;
  streamTokens = std::min<int64_t>(cap, streamTokens + std::min<int64_t>(now - streamRefill, 1'000'000) * streamRate);
  streamRefill = now;
  if (streamTokens < 1'000'000) { ++drops; return; }
  streamTokens -= 1'000'000;
  const Telemetry event{frame, tx, unmatched};
  if (xQueueSend(streamQueue, &event, 0) != pdTRUE) ++drops;
}

void run() {
  streamQueue = xQueueCreateStatic(streamDepth, sizeof(Telemetry), streamStorage, &streamBuffer);
  if (!streamQueue) return;
  loadProfile();
  engineReady = startEngine();
  runLink();
}
}

extern "C" void app_main() { sim::run(); }
