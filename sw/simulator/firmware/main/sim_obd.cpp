#include "sim_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include "esp_random.h"

namespace sim {
namespace {
double randomUnit() { return esp_random() / 4294967296.0; }

double sample(Generator& g, int64_t now) {
  const double width = g.max - g.min;
  switch (g.type) {
    case GenType::constant: return g.min;
    case GenType::ramp:
    case GenType::sine: {
      const int64_t period = g.periodMs * 1000LL;
      double phase = static_cast<double>((now - g.start) % period) / static_cast<double>(period);
      if (g.type == GenType::sine) phase = (1.0 - std::cos(6.283185307179586 * phase)) * 0.5;
      return g.min + width * phase;
    }
    case GenType::noise:
    case GenType::walk: {
      const int64_t sampleIndex = g.periodMs ? (now - g.start) / (g.periodMs * 1000LL) : g.lastSample + 1;
      if (sampleIndex != g.lastSample) {
        g.lastSample = sampleIndex;
        g.previous = g.type == GenType::noise ? g.min + width * randomUnit() :
            std::clamp(g.previous + (2 * randomUnit() - 1) * g.step, g.min, g.max);
      }
      return g.previous;
    }
  }
  return 0;
}

size_t configuredPid(Ecu& ecu, uint8_t mode, uint16_t pid, uint8_t* out, int64_t now) {
  for (size_t i = 0; i < ecu.pidCount; ++i) {
    auto& p = ecu.pids[i];
    if (p.mode != mode || p.id != pid) continue;
    if (p.raw) std::memcpy(out, p.data, p.len);
    else {
      const double limit = std::ldexp(1.0, 8 * p.len);
      const double rounded = std::round((sample(p.gen, now) - p.offset) / p.scale);
      if (rounded >= limit) std::memset(out, 0xff, p.len);
      else {
        auto raw = static_cast<uint64_t>(std::max(0.0, rounded));
        for (size_t b = p.len; b > 0; --b, raw >>= 8) out[b - 1] = raw & 0xff;
      }
    }
    return p.len;
  }
  return 0;
}

size_t modeOne(Ecu& ecu, uint8_t pid, uint8_t* out, int64_t now) {
  out[0] = 0x41;
  out[1] = pid;
  if (pid % 32 == 0 && pid <= 0xc0) {
    uint32_t bits = pid == 0 ? 0x80000000U : 0;
    for (size_t i = 0; i < ecu.pidCount; ++i) {
      const auto& p = ecu.pids[i];
      if (p.mode != 1 || p.id <= pid) continue;
      const int distance = p.id - pid;
      if (distance <= 32) bits |= 1U << (32 - distance);
      else if (pid < 0xc0) bits |= 1;
    }
    for (size_t i = 0; i < 4; ++i) out[2 + i] = bits >> (24 - 8 * i);
    return 6;
  }
  if (pid == 1) {
    const auto count = ecu.dtcs[0].len;
    out[2] = count | (count ? 0x80 : 0);
    out[3] = out[4] = out[5] = 0;
    return 6;
  }
  const size_t len = configuredPid(ecu, 1, pid, out + 2, now);
  return len ? len + 2 : 0;
}

bool respond(Ecu& ecu, const uint8_t* request, size_t len, int64_t now) {
  const uint8_t mode = request[0];
  const auto& fault = ecu.fault;
  if (fault.silent) return false;
  const int64_t due = now + fault.delayMs * 1000LL;
  uint8_t out[maxPayload]{};
  if (fault.nrc && fault.nrc != 0x78) {
    const uint8_t nrc[] = {0x7f, mode, fault.nrc};
    return queuePayload(ecu.resp, ecu.req, nrc, sizeof(nrc), due);
  }
  size_t n = 0;
  uint8_t error = 0x12;
  switch (mode) {
    case 0x01:
      if (len == 2) n = modeOne(ecu, request[1], out, now);
      break;
    case 0x03:
    case 0x07:
    case 0x0a: {
      if (len != 1) break;
      const auto& dtcs = ecu.dtcs[mode == 0x03 ? 0 : mode == 0x07 ? 1 : 2];
      out[0] = mode + 0x40;
      out[1] = dtcs.len;
      n = 2;
      for (size_t i = 0; i < dtcs.len; ++i) {
        out[n++] = dtcs.codes[i] >> 8;
        out[n++] = dtcs.codes[i] & 0xff;
      }
      break;
    }
    case 0x04:
      if (len != 1) break;
      if (!ecu.clearable) {
        out[0] = 0x7f;
        out[1] = mode;
        out[2] = 0x22;
        n = 3;
      } else {
        for (auto& d : ecu.dtcs) d.len = 0;
        out[0] = 0x44;
        n = 1;
      }
      break;
    case 0x09:
      if (len != 2) break;
      out[0] = 0x49;
      out[1] = request[1];
      if (request[1] == 0) {
        out[2] = ecu.vin[0] ? 0x40 : 0;
        n = 6;
      } else if (request[1] == 2 && ecu.vin[0]) {
        out[2] = 1;
        std::memcpy(out + 3, ecu.vin, 17);
        n = 20;
      }
      break;
    case 0x10:
      if (len != 2 || !(request[1] & 0x7f)) break;
      ecu.session = request[1] & 0x7f;
      if (request[1] & 0x80) return true;
      out[0] = 0x50;
      out[1] = ecu.session;
      out[3] = 50;  // P2: 50 ms
      out[4] = 1;
      out[5] = 244; // P2*: 500 * 10 ms
      n = 6;
      break;
    case 0x22:
      if (len == 3) {
        const uint16_t did = (request[1] << 8) | request[2];
        out[0] = 0x62;
        out[1] = request[1];
        out[2] = request[2];
        for (size_t i = 0; i < ecu.didCount; ++i) {
          const auto& d = ecu.dids[i];
          if (d.id != did) continue;
          std::memcpy(out + 3, d.data, d.len);
          n = 3 + d.len;
          break;
        }
        if (!n) {
          const size_t sized = configuredPid(ecu, 0x22, did, out + 3, now);
          if (sized) n = 3 + sized;
        }
      }
      break;
    case 0x3e:
      if (len != 2 || (request[1] & 0x7f) != 0) break;
      if (request[1] & 0x80) return true;
      out[0] = 0x7e;
      n = 2;
      break;
    default: error = 0x11;
  }
  if (!n) {
    if (!ecu.unknownNrc) return false;
    out[0] = 0x7f;
    out[1] = mode;
    out[2] = error;
    n = 3;
  }
  if (fault.nrc == 0x78) {
    const uint8_t pending[] = {0x7f, mode, 0x78};
    if (!queuePayload(ecu.resp, ecu.req, pending, sizeof(pending), due)) return false;
    (void)queuePayload(ecu.resp, ecu.req, out, n, due + fault.pendingMs * 1000LL);
    return true;
  }
  return queuePayload(ecu.resp, ecu.req, out, n, due);
}
}

bool handleObd(const Frame& request, int64_t now) {
  if (request.addr.ext || request.rtr || request.len < 2 || request.data[0] == 0 ||
      request.data[0] > request.len - 1) return false;
  bool matched = false;
  for (size_t i = 0; i < model.ecuCount; ++i) {
    auto& ecu = model.ecus[i];
    if (request.addr.id == 0x7df || request.addr == ecu.req)
      matched = respond(ecu, request.data + 1, request.data[0], now) || matched;
  }
  return matched;
}
}
