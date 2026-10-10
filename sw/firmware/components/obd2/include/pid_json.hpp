// pid_json.hpp: PID definition JSON <-> PIDDefinitionData. No hardware, builds for the linux host tests.
// The input is hostile (REST body or a file on the SD card): every field is range-checked and nothing here aborts.
#pragma once

#include <cstdint>
#include <vector>

#include "cJSON.h"
#include "pid_def.hpp"

namespace pidjson
{

struct ParseResult
{
    bool        ok;
    int         httpStatus;  // 400 / 422 when !ok
    const char* reason;      // static string, valid when !ok
};

// Validates a JSON array of PID definitions (schema in docs/pid-config.md) into `out`.
ParseResult parse(const cJSON* array, std::vector<PIDDefinitionData>& out);

// "length" is the canonical key, "len" is accepted as an alias. Both present must agree.
// Returns false if the value is out of range or the aliases disagree; absent is ok unless `required`.
bool lengthFromJson(const cJSON* object, uint8_t& output, bool required);

// Serializes definitions to a JSON array in the same schema. Returns nullptr on allocation failure.
cJSON* serialize(const std::vector<PIDDefinitionData>& definitions);

}  // namespace pidjson
