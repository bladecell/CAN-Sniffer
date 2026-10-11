// Upper bounds on sizes taken from outside the device. Hardware-free so host tests can check them.
#pragma once
#include <cstddef>

namespace input_limits
{
// PID JSON file on the SD card; the whole file is read into one buffer.
constexpr size_t PID_FILE_MAX = 256 * 1024;
// Largest legitimate client frame is a 1-byte command; 125 is the WebSocket control frame payload maximum (ping).
constexpr size_t WS_FRAME_MAX = 125;
// SD upload. Streamed to the card, so this bounds time and card use, not RAM. Covers the DTC bin and one OTA image.
constexpr size_t UPLOAD_MAX = 4 * 1024 * 1024;

constexpr bool pidFileSizeOk(size_t n) { return n > 0 && n <= PID_FILE_MAX; }
constexpr bool wsFrameLenOk(size_t n) { return n <= WS_FRAME_MAX; }
constexpr bool uploadLenOk(size_t n) { return n <= UPLOAD_MAX; }
}  // namespace input_limits
