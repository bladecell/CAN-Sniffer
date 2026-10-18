#pragma once

// PATCH /settings outcome rules (P2.12 BR3). No ESP-IDF includes, so host tests include it directly.
// The body is an object keyed by section name ({"wifi":{...},"can":{...}}); each section is applied on its own and
// reports one Outcome. A section that is not valid saves nothing, the other sections are still saved (docs/decisions.md).

#include <cstddef>
#include <string_view>

namespace settings_patch
{
enum class Section { Wifi, Can, System, Unknown };
enum class Outcome { Saved, Invalid, Unknown, Failed };

constexpr size_t kMaxSections = 3;

inline Section sectionOf(std::string_view name)
{
  if (name == "wifi") { return Section::Wifi; }
  if (name == "can") { return Section::Can; }
  if (name == "system") { return Section::System; }
  return Section::Unknown;
}

// A body with no section or with more keys than there are sections cannot be valid.
inline bool sectionCountOk(size_t n) { return n >= 1 && n <= kMaxSections; }

// Index of the first name that repeats an earlier one, or n when all are distinct. cJSON keeps duplicate keys as separate
// children, so without this a body could apply the same section twice.
inline size_t firstDuplicate(const std::string_view* names, size_t n)
{
  for (size_t i = 1; i < n; i++)
  {
    for (size_t j = 0; j < i; j++)
    {
      if (names[i] == names[j]) { return i; }
    }
  }
  return n;
}

// Failed (NVS error) 500 > Invalid 400 > Unknown section 422; nothing to apply is a 400.
inline int httpStatus(const Outcome* o, size_t n)
{
  if (n == 0) { return 400; }
  bool invalid = false, unknown = false;
  for (size_t i = 0; i < n; i++)
  {
    if (o[i] == Outcome::Failed) { return 500; }
    invalid = invalid || o[i] == Outcome::Invalid;
    unknown = unknown || o[i] == Outcome::Unknown;
  }
  return invalid ? 400 : (unknown ? 422 : 200);
}

// Top-level `reason` of a failed PATCH: the first error entry's reason.
inline const char* topReason(const char* firstErrorReason)
{
  return (firstErrorReason != nullptr && firstErrorReason[0] != '\0') ? firstErrorReason : "settings not saved";
}

// Saved values reach the drivers only after a reboot.
inline bool restartRequired(const Outcome* o, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    if (o[i] == Outcome::Saved) { return true; }
  }
  return false;
}
}  // namespace settings_patch
