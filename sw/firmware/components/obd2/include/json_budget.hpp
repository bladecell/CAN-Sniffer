// Pre-scan for untrusted JSON (B56 / HEAP-1): a byte cap does not bound cJSON's heap use, because every value costs a
// ~64 B node ("[[],[],..." is 0.5 nodes per byte). check() counts the nodes cJSON would allocate and refuses the parse
// when the tree is implausible for the byte size, too deep, or would not fit in the free heap. Pure, no allocation.
#pragma once
#include <cstddef>
#include <cstdint>

namespace json_budget
{
constexpr size_t NODE_BYTES   = 64;          // one cJSON node plus allocator overhead
constexpr size_t MAX_NODES    = 8192;        // a 256-item PID set is about 5000 nodes
constexpr size_t MAX_DEPTH    = 32;          // same as CONFIG_CJSON_NESTING_LIMIT
constexpr size_t HEAP_RESERVE = size_t{32} * 1024;  // left free for the rest of the firmware after the parse

enum class Verdict : uint8_t
{
    Ok,
    TooManyNodes,
    TooDeep,
    NoMemory,
};

// Heap needed to parse: nodes plus the text copied into string values. Saturates instead of wrapping.
constexpr size_t estimateBytes(size_t nodes, size_t len)
{
    return nodes > (SIZE_MAX - len) / NODE_BYTES ? SIZE_MAX : nodes * NODE_BYTES + len;
}

namespace detail
{
constexpr bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }
constexpr bool isDelimiter(char c)
{
    return isSpace(c) || c == ',' || c == ':' || c == '[' || c == ']' || c == '{' || c == '}' || c == '"';
}
}  // namespace detail

// Object keys are not nodes (they are strings owned by their value). Bytes are never read past `len`.
inline Verdict check(const char* text, size_t len, size_t freeHeap)
{
    if (text == nullptr) { len = 0; }
    size_t nodes = 0;
    size_t depth = 0;
    size_t i     = 0;
    while (i < len)
    {
        const char c = text[i];
        if (c == '[' || c == '{')
        {
            ++nodes;
            if (++depth > MAX_DEPTH) { return Verdict::TooDeep; }
            ++i;
        }
        else if (c == ']' || c == '}')
        {
            if (depth > 0) { --depth; }
            ++i;
        }
        else if (c == '"')
        {
            ++i;
            while (i < len && text[i] != '"') { i += (text[i] == '\\' && i + 1 < len) ? 2 : 1; }
            if (i < len) { ++i; }  // closing quote
            size_t j = i;
            while (j < len && detail::isSpace(text[j])) { ++j; }
            if (j >= len || text[j] != ':') { ++nodes; }
        }
        else if (detail::isSpace(c) || c == ',' || c == ':')
        {
            ++i;
        }
        else
        {
            ++nodes;  // number, true, false, null
            while (i < len && !detail::isDelimiter(text[i])) { ++i; }
        }
        if (nodes > MAX_NODES) { return Verdict::TooManyNodes; }
    }
    if (nodes > len / 3 + 16) { return Verdict::TooManyNodes; }
    const size_t need = estimateBytes(nodes, len);
    if (need > freeHeap || freeHeap - need < HEAP_RESERVE) { return Verdict::NoMemory; }
    return Verdict::Ok;
}

constexpr const char* reason(Verdict v)
{
    switch (v)
    {
        case Verdict::TooManyNodes: return "JSON has too many values for its size";
        case Verdict::TooDeep:      return "JSON nested too deeply";
        case Verdict::NoMemory:     return "Not enough memory to parse JSON";
        case Verdict::Ok:           break;
    }
    return "";
}
}  // namespace json_budget
