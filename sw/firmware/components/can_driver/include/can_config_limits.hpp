#pragma once

// Pure bounds for the CAN queue sizes (B48). No ESP-IDF includes, so host tests can include it directly.
// Taking double lets one check cover a JSON number (negative, fractional, NaN, huge) and any integer type.
namespace can_limits
{
constexpr double kRxQueueMin = 1;
constexpr double kRxQueueMax = 256;
constexpr double kTxQueueMin = 1;
constexpr double kTxQueueMax = 64;

inline bool rxQueueSizeValid(double v)
{
    return v >= kRxQueueMin && v <= kRxQueueMax && v == static_cast<double>(static_cast<unsigned>(v));
}

inline bool txQueueDepthValid(double v)
{
    return v >= kTxQueueMin && v <= kTxQueueMax && v == static_cast<double>(static_cast<unsigned>(v));
}

inline bool canConfigValid(double rxQueueSize, double txQueueDepth)
{
    return rxQueueSizeValid(rxQueueSize) && txQueueDepthValid(txQueueDepth);
}
}  // namespace can_limits
