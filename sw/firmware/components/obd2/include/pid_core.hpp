// pid_core.hpp: the built-in core PID set (mode 0x01). Always part of the PID database, with or without an SD card or
// a pid_def.json. No hardware, builds for the linux host tests.
#pragma once

#include <vector>

#include "esp_err.h"
#include "pid_def.hpp"

namespace pidcore
{

// The core definitions: 0x04 load, 0x05 coolant, 0x0C RPM, 0x0D speed, 0x0F intake air temp, 0x10 MAF.
const std::vector<PIDDefinitionData>& definitions();

// True when `pid` belongs to the core set.
bool isCore(uint16_t pid);

// Core PIDs are immutable: remove, edit and replace-with-a-different-definition are refused with this code
// (HTTP 403, code PID_BUILTIN).
constexpr esp_err_t kBuiltin = ESP_ERR_NOT_SUPPORTED;

// True when `d` is field for field the built-in definition of its pid.
bool isIdentical(const PIDDefinitionData& d);

// Makes `defs` a legal full set: an item for a core pid must be identical to the built-in (else kBuiltin, `defs`
// unchanged); core definitions that are absent are added. ESP_ERR_INVALID_SIZE when the result would exceed
// `maxItems`; `defs` is then unchanged.
esp_err_t fill(std::vector<PIDDefinitionData>& defs, size_t maxItems);

// Copy of `requested` made a legal full set by fill(). This is what OBD2::replacePIDDefinitions applies.
esp_err_t merged(const std::vector<PIDDefinitionData>& requested, std::vector<PIDDefinitionData>& out, size_t maxItems);

// Boot-file load only: replaces every item for a core pid that differs from the built-in with the built-in one and
// appends the replaced pids to `replaced`. API writes never call this; they reject such items.
void overrideChanged(std::vector<PIDDefinitionData>& defs, std::vector<uint16_t>& replaced);

// Remove and add of a core pid are refused: kBuiltin for a core pid, else ESP_OK.
esp_err_t guardChange(uint16_t pid);

// HTTP status for a refused change: 403 for remove / whole-set write / load, 422 for one item of POST add.
enum class Op
{
    Write,
    Add
};
inline int statusFor(Op op)
{
    return op == Op::Add ? 422 : 403;
}
inline constexpr const char* kCode = "PID_BUILTIN";

// The seed runs only when the load found the file confirmed missing; never for a file that exists (any other result).
inline bool shouldSeed(esp_err_t loadResult)
{
    return loadResult == ESP_ERR_NOT_FOUND;
}

}  // namespace pidcore
