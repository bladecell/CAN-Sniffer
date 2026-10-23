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

// Validates one item of `POST /pid_def` (add) into `out`, with the same field ranges, string length caps, non-empty
// formula and (mode, PID, length) rule as parse(). Looser only where docs/pid-config.md says so: required are id, mode,
// pid, name, formula, interval; priority, color, length/len, unit, desc, icon, minV, maxV are optional (null = absent);
// an interval of 1..3 is accepted here and raised by the poll queue. Keys read here are matched case-insensitively, as POST always did; length/len go through lengthFromJson and are case-sensitive. A present optional string of the wrong type is rejected; absent or null keeps "".
// Does not check for an existing PID: the data model does that. False leaves `out` unspecified.
bool parseAddItem(const cJSON* item, PIDDefinitionData& out);

// "length" is the canonical key, "len" is accepted as an alias. Both present must agree.
// Returns false if the value is out of range or the aliases disagree; absent is ok unless `required`.
bool lengthFromJson(const cJSON* object, uint8_t& output, bool required);

// One JSON number to an unsigned integer in [minValue, maxValue]. False (output untouched) for null, non-numbers, NaN,
// infinity, fractions and anything out of range. Never wraps and never casts an out-of-range double.
bool integerFromJson(const cJSON* value, uint32_t minValue, uint32_t maxValue, uint32_t& output);

// One JSON number to a finite float. False (output untouched) for null, non-numbers, NaN, infinity and values a float
// cannot hold.
bool floatFromJson(const cJSON* value, float& output);

// Serializes definitions to a JSON array in the same schema. Returns nullptr on allocation failure.
cJSON* serialize(const std::vector<PIDDefinitionData>& definitions);

}  // namespace pidjson
