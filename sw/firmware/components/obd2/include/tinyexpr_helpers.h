// tinyexpr_helpers.h
#pragma once

#include <cstdint>

// A double to uint32 cast is undefined for NaN, negatives and values above UINT32_MAX; saturate instead.
inline uint32_t te_to_u32(double v)
{
    if (!(v > 0.0))
    {
        return 0;
    }
    return v >= 4294967295.0 ? 0xFFFFFFFFu : (uint32_t)v;
}

// tinyexpr custom functions
inline double get_bit(double val, double bit_idx)
{
    uint32_t byte = te_to_u32(val);
    uint32_t bit  = te_to_u32(bit_idx);
    if (bit > 31)
    {
        return 0.0;
    }
    return (byte & (1UL << bit)) ? 1.0 : 0.0;
}

inline double bit_mask(double val, double start, double len)
{
    uint32_t v = te_to_u32(val);
    uint32_t s = te_to_u32(start);
    uint32_t l = te_to_u32(len);
    if (s > 31 || l == 0)
    {
        return 0.0;
    }

    uint32_t mask = l >= 32 ? 0xFFFFFFFFu : ((1u << l) - 1u);
    return (double)((v >> s) & mask);
}

// Defined in obd2.cpp (need access to the OBD2 data model).
double get_pid_value(double id);
double get_pid_raw(double id, double byte_idx);