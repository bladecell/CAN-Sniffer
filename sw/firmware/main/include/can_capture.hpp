#pragma once

// SD capture of the raw CAN stream (UI3e-8): CSV rows from the 0x10 batches the drain task delivers, written to
// /logs/captures/cap_NNNNNN.csv by its own short-lived task. Needs the stream on (a WebSocket subscribed with 0xB0) and a
// mounted card; ends on stop(), when the stream ends (last socket closed), at the size cap or on a write error. A full or
// missing card ends the capture with an error code; nothing else in the device is affected. Callable from REST workers.
// Format and layout: docs/sd-card.md.
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace can_capture_service
{
enum class Start
{
  started,
  streamOff,  // 409 STREAM_OFF: the raw stream is not on
  running,    // 409 CAPTURE_RUNNING
  noCard,     // 503 NO_SD: no mounted card, or the card is busy
  cardError,  // 503 SD_ERROR: folder, name or first write failed (also a folder with too many files)
  noMemory    // 503 NO_MEMORY
};

struct Status
{
  bool        running;
  char        file[16];  // current or last capture, empty before the first
  uint32_t    rows;
  uint32_t    bytes;     // written to the card, header included
  uint32_t    dropped;   // rows the writer could not take (queue full) or batches it could not decode
  const char* ended;     // why the last capture ended: "stop", "stream", "limit", "error"; nullptr while running or before the first
  const char* error;     // set when ended is "error": "NO_SD" or "SD_ERROR"
};

Start  start();
// Asks the task to finish and waits for it (at most about 3 s). Idempotent; ESP_ERR_TIMEOUT when the card did not answer in time
// (the task ends later by itself).
esp_err_t stop();
Status status();

// Names of the newest captures, with sizes. truncated: more files than `max`.
struct FileInfo
{
  char     name[16];
  uint32_t bytes;
};
esp_err_t list(FileInfo* out, size_t max, size_t& count, bool& truncated);
enum class Remove
{
  removed,
  badName,   // 400 BAD_NAME
  notFound,  // 404
  inUse,     // 409 CAPTURE_RUNNING: the file being written
  noCard,    // 503
  failed     // 500
};
Remove remove(const char* name);
// "/logs/captures/<name>" for a valid name; false (nothing written) for anything else.
bool pathFor(const char* name, char* out, size_t cap);
}  // namespace can_capture_service
