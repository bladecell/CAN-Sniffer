// obd2_frame.hpp: pure OBD-II / ISO-TP frame parsing. No hardware, no RTOS, so it builds for the linux host tests.
// Every length and index comes from the bus and is treated as hostile.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "esp_err.h"

namespace obd2frame
{

constexpr size_t   CAN_DATA_MAX     = 8;    // classic CAN
constexpr uint16_t MAX_ISOTP_LENGTH = 512;  // cap on a reassembled payload (VIN is 20, DTC lists are far smaller)

enum class IsoTp : uint8_t
{
    Single      = 0,
    First       = 1,
    Consecutive = 2,
    FlowControl = 3,
};

struct RawFrame
{
    uint8_t data[CAN_DATA_MAX];
    size_t  length;
};

struct SingleFrameInfo
{
    uint8_t  mode;           // response SID: 0x41, 0x62, ...
    uint16_t pid;            // 8-bit for 0x41 / 0x42, 16-bit for 0x62 / derived
    uint8_t  payloadOffset;  // index of the first data byte in the frame
    uint8_t  payloadLength;  // data bytes that belong to the message (padding excluded)
};

// ISO-TP frame type from the high nibble of byte 0. Needs at least one byte.
esp_err_t frameType(const uint8_t* data, size_t length, IsoTp& type);

// Validates a single frame (length 2..8, PCI length 1..7 and within the frame, known SID) and extracts mode/PID.
// Modes without a PID (e.g. 0x43, 0x7F) report pid 0 and payloadOffset 2.
esp_err_t parseSingleFrame(const uint8_t* data, size_t length, SingleFrameInfo& out);

// First frame: total payload length and number of consecutive frames still expected.
esp_err_t parseFirstFrame(const uint8_t* data, size_t length, uint16_t& totalLength, uint16_t& framesNeeded);

// True if this is a consecutive frame whose 4-bit sequence matches the 1-based index (wraps 15 -> 0).
bool consecutiveSequenceOk(const uint8_t* data, size_t length, uint16_t index);

// Joins one single frame, or a first frame plus consecutive frames, into the ISO-TP payload (SID first).
// Checks frame types, sequence numbers, total length and that all frames are present. Returns the payload.
esp_err_t reassemble(const RawFrame* frames, size_t count, std::vector<uint8_t>& payload);

// Mode 03/07/0A payload (0x43 / 0x47 / 0x4A, then 2 bytes per DTC) to raw 16-bit DTCs. A trailing odd byte is dropped.
esp_err_t extractDtcs(const std::vector<uint8_t>& payload, std::vector<uint16_t>& rawDtcs);

// Mode 09 PID 02 payload (0x49 0x02 count, then 17 ASCII chars) to the VIN.
esp_err_t extractVin(const std::vector<uint8_t>& payload, char (&vin)[17]);

// Reassembles `count` frames and checks the response SID equals `mode` (0x43 / 0x47 / 0x4A), then extracts raw DTCs.
// ESP_ERR_INVALID_RESPONSE on a wrong SID. Clears `rawDtcs` on failure.
esp_err_t decodeDtcResponse(const RawFrame* frames, size_t count, uint8_t mode, std::vector<uint16_t>& rawDtcs);

// Reassembles `count` frames and checks the header is 0x49 0x02, then extracts the VIN.
// ESP_ERR_INVALID_RESPONSE on a wrong header.
esp_err_t decodeVinResponse(const RawFrame* frames, size_t count, char (&vin)[17]);

}  // namespace obd2frame
