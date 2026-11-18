#include "sim_commands.hpp"
#include "sim_model.hpp"

#include <cmath>
#include <cstring>
#include <new>

namespace sim {
Json field(Json root, const char* name) { return cJSON_GetObjectItemCaseSensitive(root, name); }
const char* text(Json item) { return cJSON_IsString(item) ? item->valuestring : nullptr; }
bool integer(Json item, uint32_t min, uint32_t max, uint32_t& out) {
  if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) return false;
  const double val = item->valuedouble;
  if (val < min || val > max || std::trunc(val) != val) return false;
  out = static_cast<uint32_t>(val);
  return true;
}
bool optionalInt(Json root, const char* name, uint32_t min, uint32_t max, uint32_t& inOut) {
  const auto item = field(root, name);
  return !item || integer(item, min, max, inOut);
}
bool optionalBool(Json root, const char* name, bool& inOut) {
  const auto item = field(root, name);
  if (!item) return true;
  if (!cJSON_IsBool(item)) return false;
  inOut = cJSON_IsTrue(item);
  return true;
}

namespace {
constexpr const char* journalFull = "profile command log full";
bool equals(const char* x, const char* y) { return x && std::strcmp(x, y) == 0; }
bool real(Json item, double& out) {
  if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) || std::abs(item->valuedouble) > 1e30) return false;
  out = item->valuedouble;
  return true;
}
bool optionalReal(Json item, const char* name, double& out) {
  const auto val = field(item, name);
  return !val || real(val, out);
}

bool address(Json root, Address& a) {
  return cJSON_IsObject(root) && optionalBool(root, "ext", a.ext) &&
      integer(field(root, "id"), 0, a.ext ? 0x1fffffff : 0x7ff, a.id);
}

bool bytes(Json item, uint8_t* data, size_t cap, size_t& len) {
  if (!cJSON_IsArray(item)) return false;
  len = 0;
  Json val;
  cJSON_ArrayForEach(val, item) {
    uint32_t n;
    if (len == cap || !integer(val, 0, 255, n)) return false;
    data[len++] = n;
  }
  return true;
}

bool actions(Json array, Rule& r) {
  if (!cJSON_IsArray(array)) return false;
  uint16_t offset = 0;
  Json frame;
  cJSON_ArrayForEach(frame, array) {
    if (r.frameCount >= (r.isotp ? 1 : maxActions)) return false;
    auto& a = r.actions[r.frameCount++];
    if (!address(frame, a.addr) || !optionalInt(frame, "delayMs", 0, maxDelayMs, a.delayMs)) return false;
    const auto arrayData = field(frame, "data");
    if (!cJSON_IsArray(arrayData)) return false;
    a.start = offset;
    Json value;
    cJSON_ArrayForEach(value, arrayData) {
      if (offset == maxPayload || (!r.isotp && a.len == 8)) return false;
      uint32_t n;
      if (integer(value, 0, 255, n)) r.data[offset] = static_cast<int16_t>(n);
      else {
        const char* s = text(value);
        if (!s || s[0] != '$' || s[1] < '0' || s[1] > '7' || s[2]) return false;
        const auto index = s[1] - '0';
        if (!r.anyDlc && index >= r.dlc) return false;
        r.data[offset] = -1 - index;
      }
      ++a.len;
      ++offset;
    }
    if (r.isotp && !a.len) return false;
  }
  return r.frameCount != 0;
}

bool generator(Json item, Generator& g, int64_t now) {
  if (!cJSON_IsObject(item)) return false;
  const auto type = text(field(item, "type"));
  g.start = now;
  if (equals(type, "const")) {
    const auto value = field(item, "value");
    if (value) return real(value, g.min);
    return real(field(item, "min"), g.min);
  }
  if (equals(type, "ramp")) g.type = GenType::ramp;
  else if (equals(type, "sine")) g.type = GenType::sine;
  else if (equals(type, "noise")) g.type = GenType::noise;
  else if (equals(type, "walk")) g.type = GenType::walk;
  else return false;
  if (!real(field(item, "min"), g.min) || !real(field(item, "max"), g.max) || g.min > g.max ||
      !optionalInt(item, "periodMs", 0, maxPeriodMs, g.periodMs)) return false;
  if ((g.type == GenType::ramp || g.type == GenType::sine) && !g.periodMs) return false;
  g.previous = g.min;
  if (g.type != GenType::walk) return true;
  return real(field(item, "step"), g.step) && g.step > 0;
}

bool pidSource(Json root, Pid& p, int64_t now) {
  const auto raw = field(root, "raw"), value = field(root, "value"), gen = field(root, "gen");
  if ((raw != nullptr) + (value != nullptr) + (gen != nullptr) != 1) return false;
  p.raw = raw != nullptr;
  if (raw) {
    size_t len;
    return bytes(raw, p.data, p.len, len) && len == p.len;
  }
  if (p.len > 8 || p.scale == 0 || std::abs(p.scale) < 1e-30) return false;
  p.gen = {};
  return value ? real(value, p.gen.min) : generator(gen, p.gen, now);
}

int hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + c - 'a';
  if (c >= 'A' && c <= 'F') return 10 + c - 'A';
  return -1;
}
bool dtcs(Json array, DtcList& d) {
  if (!cJSON_IsArray(array)) return false;
  Json item;
  cJSON_ArrayForEach(item, array) {
    if (d.len == maxDtcs) return false;
    const auto s = text(item);
    if (!s || std::strlen(s) != 5 || s[1] < '0' || s[1] > '3') return false;
    int system;
    switch (s[0]) {
      case 'P': case 'p': system = 0; break;
      case 'C': case 'c': system = 1; break;
      case 'B': case 'b': system = 2; break;
      case 'U': case 'u': system = 3; break;
      default: return false;
    }
    uint16_t code = system;
    for (size_t i = 1; i < 5; ++i) {
      int v = hex(s[i]);
      if (v < 0) return false;
      code = (code << 4) | v;
    }
    d.codes[d.len++] = code;
  }
  return true;
}

uint32_t updateKey(uint32_t group, size_t ecu, uint32_t id) {
  return (group << 24) | (static_cast<uint32_t>(ecu) << 16) | id;
}
}

const char* edit(Json root, const char* cmd, int64_t now, bool replay, size_t lineSize, Reserve reserve) {
  const auto keep = [&](uint32_t key) { return reserve(key, lineSize); };
  if (equals(cmd, "can.config")) {
    auto rate = model.bitrate;
    auto ack = model.ack;
    if ((!field(root, "bitrate") && !field(root, "ack")) || !optionalInt(root, "bitrate", 0, 1'000'000, rate) ||
        (rate != 125'000 && rate != 250'000 && rate != 500'000 && rate != 1'000'000) ||
        !optionalBool(root, "ack", ack)) return "invalid CAN bitrate or ack";
    if (!reserve(canKey, 64)) return journalFull;
    if (!replay) {
      const auto err = can::configure(rate, ack);
      if (err != ESP_OK) return esp_err_to_name(err);
    }
    if (rate != model.bitrate || ack != model.ack) cancelJobs();
    model.bitrate = rate;
    model.ack = ack;
    return nullptr;
  }
  if (equals(cmd, "ecu.add")) {
    const auto name = text(field(root, "name"));
    const auto onUnknown = field(root, "onUnknown");
    Address req, resp;
    bool ext = false, clearable = true;
    if (!name || !*name || std::strlen(name) > maxName ||
        !integer(field(root, "reqId"), 0, 0x7ff, req.id) || req.id == 0x7df ||
        !integer(field(root, "respId"), 0, 0x7ff, resp.id) || req == resp ||
        !optionalBool(root, "clearable", clearable) || !optionalBool(root, "ext", ext) || ext ||
        (onUnknown && !equals(text(onUnknown), "silent") && !equals(text(onUnknown), "nrc"))) return "invalid ECU";
    if (model.ecuCount == maxEcus) return "ECU limit";
    for (size_t i = 0; i < model.ecuCount; ++i) {
      const auto& e = model.ecus[i];
      if (equals(e.name, name) || e.req == req || e.resp == resp) return "duplicate ECU name or CAN ID";
    }
    if (!keep(independent)) return journalFull;
    auto& e = model.ecus[model.ecuCount++];
    new (&e) Ecu{};
    std::memcpy(e.name, name, std::strlen(name) + 1);
    e.req = req;
    e.resp = resp;
    e.unknownNrc = equals(text(onUnknown), "nrc");
    e.clearable = clearable;
    return nullptr;
  }
  if (equals(cmd, "rule.add")) {
    if (model.ruleCount == maxRules) return "rule limit";
    Rule r;
    const auto match = field(root, "match"), action = field(root, "action");
    if (!integer(field(root, "id"), 0, UINT32_MAX, r.id) || !address(match, r.match) || !cJSON_IsObject(action)) return "invalid rule";
    const uint32_t fullMask = r.match.ext ? 0x1fffffff : 0x7ff;
    r.mask = fullMask;
    uint32_t dlc = 0, repeat = 1;
    size_t n = 0;
    if (!optionalInt(match, "mask", 0, fullMask, r.mask) || !optionalInt(match, "dlc", 0, 8, dlc) ||
        (field(match, "pattern") && !bytes(field(match, "pattern"), r.pattern, 8, n))) return "invalid rule match";
    r.anyDlc = !field(match, "dlc");
    r.dlc = dlc;
    r.patternLen = n;
    if (!r.anyDlc && n > dlc) return "rule pattern exceeds DLC";
    const auto mask = field(match, "patternMask");
    size_t maskLen;
    if (mask && (!bytes(mask, r.patternMask, n, maskLen) || maskLen != n)) return "invalid patternMask";
    if (!mask) std::memset(r.patternMask, 0xff, n);
    if (!optionalBool(action, "isotp", r.isotp) || !optionalInt(action, "periodMs", 0, maxPeriodMs, r.periodMs))
      return "invalid rule action";
    repeat = r.periodMs ? 0 : 1;
    if (!optionalInt(action, "repeat", 0, 1000, repeat) || (!repeat && !r.periodMs) || !actions(field(action, "frames"), r))
      return "invalid rule action";
    r.repeat = repeat;
    for (size_t i = 0; i < model.ruleCount; ++i) if (model.rules[i].id == r.id) return "duplicate rule ID";
    if (!keep(ruleGroup)) return journalFull;
    model.rules[model.ruleCount++] = r;
    return nullptr;
  }
  if (equals(cmd, "cyclic.add")) {
    if (model.cyclicCount == maxCyclic) return "cyclic frame limit";
    Cyclic c;
    size_t n;
    if (!address(root, c.frame.addr) || !integer(field(root, "periodMs"), 1, maxPeriodMs, c.periodMs) ||
        !bytes(field(root, "data"), c.frame.data, 8, n)) return "invalid cyclic frame";
    c.frame.len = n;
    c.due = now + c.periodMs * 1000LL;
    if (!keep(ruleGroup)) return journalFull;
    model.cyclic[model.cyclicCount++] = c;
    return nullptr;
  }

  const bool faultCommand = equals(cmd, "ecu.fault");
  const bool pidAdd = equals(cmd, "pid.add"), pidSet = equals(cmd, "pid.set");
  if (!faultCommand && !pidAdd && !pidSet && !equals(cmd, "dtc.set") && !equals(cmd, "did.set") && !equals(cmd, "vin.set"))
    return "unknown cmd";
  const auto name = text(field(root, faultCommand ? "name" : "ecu"));
  size_t index = 0;
  while (index < model.ecuCount && !equals(name, model.ecus[index].name)) ++index;
  if (index == model.ecuCount) return "unknown ECU";
  auto& ecu = model.ecus[index];
  if (faultCommand) {
    Fault f;
    uint32_t nrc = 0;
    if (!optionalBool(root, "silent", f.silent) || !optionalInt(root, "delayMs", 0, maxDelayMs, f.delayMs) ||
        !optionalInt(root, "nrc", 0, 255, nrc) || !optionalInt(root, "pendingMs", 0, 4999, f.pendingMs) ||
        (nrc != 0x78 && f.pendingMs)) return "invalid fault";
    f.nrc = nrc;
    if (nrc == 0x78 && !field(root, "pendingMs")) f.pendingMs = 100;
    if (!keep(updateKey(1, index, 0))) return journalFull;
    ecu.fault = f;
    return nullptr;
  }
  if (pidAdd || pidSet) {
    uint32_t mode, id, len;
    if (!integer(field(root, "mode"), 1, 0x22, mode) || (mode != 0x01 && mode != 0x22) ||
        !integer(field(root, "pid"), 0, mode == 0x01 ? 0xff : 0xffff, id)) return "invalid PID mode or ID";
    size_t i = 0;
    while (i < ecu.pidCount && !(ecu.pids[i].mode == mode && ecu.pids[i].id == id)) ++i;
    Pid p;
    if (pidAdd) {
      if (i != ecu.pidCount) return "duplicate PID";
      if (i == maxPids) return "PID limit";
      if (!integer(field(root, "len"), 1, maxRawPid, len) || !optionalReal(root, "scale", p.scale) ||
          !optionalReal(root, "offset", p.offset)) return "invalid PID encoding";
      p.mode = mode;
      p.id = id;
      p.len = len;
    } else {
      if (i == ecu.pidCount) return "unknown PID";
      p = ecu.pids[i];
    }
    if (!pidSource(root, p, now)) return "invalid PID source";
    if (!keep(pidAdd ? independent : updateKey(mode == 1 ? 2 : 3, index, id))) return journalFull;
    ecu.pids[i] = p;
    if (pidAdd) ++ecu.pidCount;
    return nullptr;
  }
  if (equals(cmd, "dtc.set")) {
    const auto kind = text(field(root, "kind"));
    const int k = equals(kind, "stored") ? 0 : equals(kind, "pending") ? 1 : equals(kind, "permanent") ? 2 : -1;
    DtcList d;
    if (k < 0 || !dtcs(field(root, "codes"), d)) return "invalid DTC list";
    if (!keep(updateKey(4, index, k))) return journalFull;
    ecu.dtcs[k] = d;
    return nullptr;
  }
  if (equals(cmd, "vin.set")) {
    const auto vin = text(field(root, "vin"));
    if (!vin || std::strlen(vin) != 17 || std::strspn(vin, "0123456789ABCDEFGHJKLMNPRSTUVWXYZ") != 17)
      return "invalid VIN";
    if (!keep(updateKey(5, index, 0))) return journalFull;
    std::memcpy(ecu.vin, vin, sizeof(ecu.vin));
    return nullptr;
  }
  uint32_t id;
  size_t n;
  Did d;
  if (!integer(field(root, "did"), 0, 0xffff, id) || !bytes(field(root, "bytes"), d.data, sizeof(d.data), n) || !n)
    return "invalid DID";
  d.id = id;
  d.len = n;
  size_t i = 0;
  while (i < ecu.didCount && ecu.dids[i].id != id) ++i;
  if (i == maxDids) return "DID limit";
  if (!keep(updateKey(6, index, id))) return journalFull;
  ecu.dids[i] = d;
  if (i == ecu.didCount) ++ecu.didCount;
  return nullptr;
}
}
