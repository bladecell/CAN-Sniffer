#pragma once

#include "sim_can.hpp"
#include <cstddef>

namespace sim {
constexpr size_t maxPayload = 256;
constexpr size_t maxRawPid = 32;
constexpr uint32_t defaultRate = 500'000;
constexpr uint32_t maxDelayMs = 60'000;
constexpr uint32_t maxPeriodMs = 3'600'000;
constexpr size_t maxEcus = 8, maxPids = 64, maxDids = 16, maxDtcs = 32;
constexpr size_t maxRules = 24, maxCyclic = 24, maxActions = 8, maxName = 31;

enum class GenType : uint8_t { constant, ramp, sine, noise, walk };
struct Generator {
  GenType type = GenType::constant;
  double min = 0, max = 0, step = 0, previous = 0;
  uint32_t periodMs = 0;
  int64_t start = 0, lastSample = -1;
};

struct Pid {
  uint16_t id = 0;
  uint8_t mode = 0, len = 0;
  bool raw = false;
  double scale = 1, offset = 0;
  Generator gen;
  uint8_t data[maxRawPid]{};
};
struct Did {
  uint16_t id = 0;
  uint16_t len = 0;
  uint8_t data[maxPayload - 3]{};
};
struct DtcList {
  uint8_t len = 0;
  uint16_t codes[maxDtcs]{};
};
struct Fault {
  bool silent = false;
  uint8_t nrc = 0;
  uint32_t delayMs = 0, pendingMs = 0;
};
struct Ecu {
  char name[maxName + 1]{};
  Address req, resp;
  bool unknownNrc = false, clearable = true;
  Fault fault;
  uint8_t session = 1;
  DtcList dtcs[3];
  Pid pids[maxPids];
  Did dids[maxDids];
  char vin[18]{};
  size_t pidCount = 0, didCount = 0;
};
struct RuleAction {
  Address addr;
  uint32_t delayMs = 0;
  uint16_t len = 0, start = 0;
};
struct Rule {
  uint32_t id = 0;
  Address match;
  uint32_t mask = 0;
  uint8_t dlc = 0, patternLen = 0;
  bool anyDlc = true, isotp = false;
  uint8_t pattern[8]{}, patternMask[8]{};
  uint8_t frameCount = 0;
  uint16_t repeat = 1;
  uint32_t periodMs = 0;
  RuleAction actions[maxActions];
  // Negative entries encode request byte indexes: -1 is $0.
  int16_t data[maxPayload]{};
};
struct Cyclic {
  Frame frame;
  uint32_t periodMs = 0;
  int64_t due = 0;
};
struct Model {
  Ecu ecus[maxEcus];
  Rule rules[maxRules];
  Cyclic cyclic[maxCyclic];
  size_t ecuCount = 0, ruleCount = 0, cyclicCount = 0;
  uint32_t bitrate = defaultRate;
  bool ack = true;
};
extern Model model;

struct EngineStats {
  uint32_t rx, unmatched, jobsDropped, isoTimeouts;
};

bool startEngine();
void lockEngine();
void unlockEngine();
EngineStats engineStats();
void resetModel();
void clearRules();
void cancelJobs();
bool queuePayload(Address tx, Address fc, const uint8_t* data, size_t len, int64_t due);
bool handleObd(const Frame& request, int64_t now);
void streamFrame(const Frame& frame, bool tx, bool unmatched);
}
