#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>

namespace p28 {
enum class Action { Pass, Fail, Hold };
class TaskInjection {
 public:
  void arm(Action action) { hits_.store(0); action_.store(action); }
  Action take(const char* name, bool expectedEntry) {
    if (!expectedEntry || !name || std::strcmp(name, "DataSimTask") != 0) return Action::Pass;
    const Action action = action_.exchange(Action::Pass);
    if (action != Action::Pass) hits_.fetch_add(1);
    return action;
  }
  unsigned hits() const { return hits_.load(); }
 private:
  std::atomic<Action> action_{Action::Pass};
  std::atomic<unsigned> hits_{0};
};
struct SavedKey {
  bool present = false;
  uint32_t value = 0;
  bool operator==(const SavedKey& other) const {
    return present == other.present && (!present || value == other.value);
  }
};
inline bool parseKey(const char* text, SavedKey& out) {
  if (std::strcmp(text, "absent") == 0) { out = {}; return true; }
  if (std::strncmp(text, "u32:", 4) != 0 || !text[4]) return false;
  uint32_t value = 0;
  for (const char* p = text + 4; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    const uint32_t digit = static_cast<uint32_t>(*p - '0');
    if (value > (UINT32_MAX - digit) / 10) return false;
    value = value * 10 + digit;
  }
  out = {true, value};
  return true;
}
struct RestoredResult { bool scenario; bool restored; };
template <class Body, class Restore>
RestoredResult withRestoration(Body body, Restore restore) {
  const bool scenario = body();
  return {scenario, restore()};
}
}  // namespace p28
