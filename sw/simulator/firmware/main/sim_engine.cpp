#include "sim_model.hpp"

#include <algorithm>
#include <cstring>
#include <iterator>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace sim {
Model model;
namespace {
constexpr int64_t flowTimeout = 150'000;
constexpr int64_t sendWindow = 33'000;
struct Packet {
  Address tx, fc;
  uint8_t data[maxPayload];
  uint16_t len = 0, pos = 0;
  uint8_t seq = 1, block = 0, left = 0, waits = 0;
  bool waiting = false, fromRule = false;
  int64_t due = 0, expires = 0, separation = 0, lastSent = 0;
};
struct RuleJob {
  const Rule* rule = nullptr;
  Frame request;
  uint16_t repeats = 0;
  uint8_t next = 0;
  int64_t due = 0, origin = 0;
};
Packet packets[32];
Packet* inFlight;
RuleJob ruleJobs[32];
EngineStats counts{};
StaticSemaphore_t lockStorage;
SemaphoreHandle_t mutex;
int64_t retryAt;
size_t cyclicNext;

bool addPacket(Address tx, Address fc, const uint8_t* data, size_t len, int64_t due, bool fromRule) {
  if (len == 0 || len > maxPayload) return false;
  for (auto& p : packets) {
    if (p.len != 0) continue;
    p = {};
    p.tx = tx;
    p.fc = fc;
    p.due = due;
    p.expires = due + sendWindow;
    p.fromRule = fromRule;
    p.len = static_cast<uint16_t>(len);
    std::memcpy(p.data, data, len);
    return true;
  }
  ++counts.jobsDropped;
  return false;
}

bool flowControl(const Frame& f, int64_t now) {
  if (f.rtr || f.len < 3 || (f.data[0] & 0xf0) != 0x30) return false;
  for (auto& p : packets) {
    if (!p.len || !p.waiting || p.fc != f.addr || p.expires < now) continue;
    const int status = f.data[0] & 0xf;
    if (status == 1 && p.waits < 3) {
      ++p.waits;
      p.expires = now + flowTimeout;
    } else if (status == 0) {
      const uint8_t st = f.data[2];
      p.separation = st <= 0x7f ? st * 1000 : st >= 0xf1 && st <= 0xf9 ? (st - 0xf0) * 100 : 127'000;
      p.block = p.left = f.data[1];
      p.waiting = false;
      p.waits = 0;
      p.due = std::max(now, p.lastSent + p.separation);
      p.expires = now + flowTimeout;
    } else {
      p.len = 0;
      ++counts.isoTimeouts;
    }
    return true;
  }
  return false;
}

bool handleRules(const Frame& f, int64_t now) {
  if (f.rtr) return false;
  bool matched = false;
  for (size_t i = 0; i < model.ruleCount; ++i) {
    const auto& r = model.rules[i];
    if (f.addr.ext != r.match.ext || ((f.addr.id ^ r.match.id) & r.mask) != 0 ||
        (!r.anyDlc && f.len != r.dlc) || r.patternLen > f.len) continue;
    bool valid = true;
    for (size_t n = 0; n < r.patternLen; ++n)
      if (((r.pattern[n] ^ f.data[n]) & r.patternMask[n]) != 0) valid = false;
    for (size_t a = 0; valid && a < r.frameCount; ++a) {
      const auto& action = r.actions[a];
      for (size_t b = 0; b < action.len; ++b)
        if (r.data[action.start + b] < -f.len) valid = false;
    }
    if (!valid) continue;
    auto idle = std::end(ruleJobs);
    if (!r.repeat) idle = std::find_if(std::begin(ruleJobs), std::end(ruleJobs), [&](const auto& j) { return j.rule == &r; });
    if (idle == std::end(ruleJobs))
      idle = std::find_if(std::begin(ruleJobs), std::end(ruleJobs), [](const auto& j) { return !j.rule; });
    if (idle == std::end(ruleJobs)) {
      ++counts.jobsDropped;
      break;
    }
    *idle = {&r, f, r.repeat, 0, now + r.actions[0].delayMs * 1000LL, now};
    matched = true;
  }
  return matched;
}

bool deliverPacket(int64_t now) {
  Packet* chosen = nullptr;
  for (auto& p : packets) {
    if (!p.len || &p == inFlight) continue;
    if (p.expires < now) {
      if (p.pos) ++counts.isoTimeouts;
      else ++counts.jobsDropped;
      p.len = 0;
      continue;
    }
    if (p.waiting || p.due > now) continue;
    if (!p.pos && p.len > 7) {
      const auto active = std::find_if(std::begin(packets), std::end(packets), [&](const auto& other) {
        return other.len && other.pos && other.fc == p.fc;
      });
      if (active != std::end(packets)) continue;
    }
    if (chosen && (p.fromRule > chosen->fromRule || (p.fromRule == chosen->fromRule && p.due >= chosen->due))) continue;
    chosen = &p;
  }
  if (!chosen || !can::canTransmit()) return false;
  auto& p = *chosen;
  Frame f;
  f.addr = p.tx;
  f.len = 8;
  if (p.len <= 7) {
    f.data[0] = p.len;
    std::memcpy(f.data + 1, p.data, p.len);
  } else if (!p.pos) {
    f.data[0] = 0x10 | (p.len >> 8);
    f.data[1] = p.len & 0xff;
    std::memcpy(f.data + 2, p.data, 6);
  } else {
    const size_t n = std::min<size_t>(7, p.len - p.pos);
    f.data[0] = 0x20 | (p.seq & 0xf);
    std::memcpy(f.data + 1, p.data + p.pos, n);
  }
  if (can::transmit(f, now)) inFlight = &p;
  return true;
}

void completePacket(can::TxResult result, int64_t ts) {
  if (!inFlight) return;
  auto& p = *inFlight;
  inFlight = nullptr;
  if (result == can::TxResult::failed || p.len <= 7) {
    p.len = 0;
    return;
  }
  p.lastSent = ts;
  p.expires = ts + flowTimeout;
  if (!p.pos) {
    p.pos = 6;
    p.waiting = true;
  } else {
    ++p.seq;
    p.pos = std::min<size_t>(p.len, p.pos + 7);
    if (p.pos == p.len) p.len = 0;
    else if (p.block && --p.left == 0) p.waiting = true;
    else p.due = ts + p.separation;
  }
}

void advanceRule(RuleJob& j, int64_t now) {
  const auto& r = *j.rule;
  if (++j.next == r.frameCount) {
    if (j.repeats && --j.repeats == 0) {
      j.rule = nullptr;
      return;
    }
    j.next = 0;
    j.origin = std::max<int64_t>(now, j.origin + r.periodMs * 1000LL);
    j.due = j.origin + r.actions[0].delayMs * 1000LL;
  } else {
    // Each delay is relative to the previous action; don't catch up missed delays in a burst.
    j.due = std::max<int64_t>(now, j.due) + r.actions[j.next].delayMs * 1000LL;
  }
}

bool deliverRule(int64_t now) {
  RuleJob* chosen = nullptr;
  for (auto& j : ruleJobs) {
    if (!j.rule || j.due > now) continue;
    if (j.due + sendWindow < now) {
      j.rule = nullptr;
      ++counts.jobsDropped;
    } else if (!chosen || j.due < chosen->due) chosen = &j;
  }
  if (!chosen || !can::canTransmit()) return false;
  auto& j = *chosen;
  const auto& r = *j.rule;
  const auto& a = r.actions[j.next];
  uint8_t bytes[maxPayload];
  for (size_t i = 0; i < a.len; ++i) {
    const int n = r.data[a.start + i];
    bytes[i] = n < 0 ? j.request.data[-n - 1] : static_cast<uint8_t>(n);
  }
  bool accepted;
  if (r.isotp) {
    accepted = addPacket(a.addr, j.request.addr, bytes, a.len, now, true);
    if (!accepted) j.rule = nullptr;
  } else {
    Frame f;
    f.addr = a.addr;
    f.len = a.len;
    std::memcpy(f.data, bytes, f.len);
    accepted = can::transmit(f, now);
  }
  if (accepted) advanceRule(j, now);
  return true;
}

void deliverCyclic(int64_t now) {
  if (!can::canTransmit() || !model.cyclicCount) return;
  for (size_t n = 0; n < model.cyclicCount; ++n) {
    const size_t i = (cyclicNext + n) % model.cyclicCount;
    auto& c = model.cyclic[i];
    if (c.due > now) continue;
    if (can::transmit(c.frame, now)) {
      // Skip missed periods instead of trying to catch up after a busy bus.
      const int64_t period = c.periodMs * 1000LL;
      c.due += ((now - c.due) / period + 1) * period;
    }
    cyclicNext = (i + 1) % model.cyclicCount;
    return;
  }
}

void engine(void*) {
  for (;;) {
    lockEngine();
    const int64_t now = esp_timer_get_time();
    Frame f;
    const auto result = can::service(now, f);
    if (result != can::TxResult::none) completePacket(result, f.ts);
    if (result == can::TxResult::success) streamFrame(f, true, false);
    if (!can::stats().ready && now >= retryAt) {
      retryAt = now + 1'000'000;
      cancelJobs();
      (void)can::configure(model.bitrate, model.ack);
    }
    for (size_t n = 0; n < 24 && can::receive(f); ++n) {
      ++counts.rx;
      const bool fresh = now - f.ts < flowTimeout;
      bool matched = model.ack && fresh && flowControl(f, now);
      if (!matched && model.ack && fresh) {
        const bool obd = handleObd(f, now);
        matched = handleRules(f, now) || obd;
      }
      if (!matched) ++counts.unmatched;
      streamFrame(f, false, !matched);
    }
    if (model.ack && !deliverPacket(now) && !deliverRule(now)) deliverCyclic(now);
    unlockEngine();
    vTaskDelay(1);
  }
}
}

bool startEngine() {
  mutex = xSemaphoreCreateMutexStatic(&lockStorage);
  if (!mutex) return false;
  (void)can::configure(model.bitrate, model.ack);
  if (xTaskCreatePinnedToCore(engine, "sim-can", 4096, nullptr, 4, nullptr, xPortGetCoreID()) == pdPASS) return true;
  (void)can::deinit();
  return false;
}

void lockEngine() { if (mutex) xSemaphoreTake(mutex, portMAX_DELAY); }
void unlockEngine() { if (mutex) xSemaphoreGive(mutex); }
EngineStats engineStats() { return counts; }

void cancelJobs() {
  inFlight = nullptr;
  for (auto& p : packets) p.len = 0;
  for (auto& j : ruleJobs) j.rule = nullptr;
}

void clearRules() {
  model.ruleCount = model.cyclicCount = cyclicNext = 0;
  if (inFlight && inFlight->fromRule) inFlight = nullptr;
  for (auto& p : packets) if (p.fromRule) p.len = 0;
  for (auto& j : ruleJobs) j.rule = nullptr;
}

void resetModel() {
  model.ecuCount = 0;
  model.ack = true;
  model.bitrate = defaultRate;
  clearRules();
  cancelJobs();
}

bool queuePayload(Address tx, Address fc, const uint8_t* data, size_t len, int64_t due) {
  return addPacket(tx, fc, data, len, due, false);
}
}
