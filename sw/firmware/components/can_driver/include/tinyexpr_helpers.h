// tinyexpr_helpers.h
#pragma once
#include "obd2.hpp"

// tinyexpr custom functions
inline double get_bit(double val, double bit_idx)
{
    uint32_t byte = (uint32_t)val;
    uint32_t bit  = (uint32_t)bit_idx;
    if (bit > 31)
        return 0.0;
    return (byte & (1UL << bit)) ? 1.0 : 0.0;
}

inline double bit_mask(double val, double start, double len)
{
    uint32_t v = (uint32_t)val;
    uint32_t s = (uint32_t)start;
    uint32_t l = (uint32_t)len;
    if (s > 31 || l == 0)
        return 0.0;

    uint32_t mask = (1UL << l) - 1;
    return (double)((v >> s) & mask);
}

inline double get_pid_value(double id)
{
    return (double)OBD2::getInstance().getValue((uint16_t)id);
}

inline double get_pid_raw(double id, double byte_idx)
{
    return (double)OBD2::getInstance().getRawDataByte((uint16_t)id, (uint8_t)byte_idx);
}