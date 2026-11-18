#pragma once

#include <cstddef>
#include <cstdint>
#include "cJSON.h"

namespace sim {
using Json = const cJSON*;
constexpr uint32_t independent = 0;
constexpr uint32_t ruleGroup = 1;
constexpr uint32_t canKey = 2;
using Reserve = bool (*)(uint32_t key, size_t size);

const char* edit(Json root, const char* cmd, int64_t now, bool replay, size_t lineSize, Reserve reserve);
Json field(Json root, const char* name);
bool integer(Json item, uint32_t min, uint32_t max, uint32_t& out);
bool optionalInt(Json root, const char* name, uint32_t min, uint32_t max, uint32_t& inOut);
bool optionalBool(Json root, const char* name, bool& inOut);
const char* text(Json item);
}
