// Host stub of can_driver.hpp: same names and fields the OBD2 data model uses, no hardware.
#pragma once

#include <cstddef>
#include <cstdint>

#define MIN_TRANSMIT_PERIOD_MS 4

class CanDriver
{
public:
    struct Header
    {
        uint32_t id;
    };

    struct CanFrame
    {
        Header  header;
        uint8_t data[8];
        size_t  length;
    };
};
