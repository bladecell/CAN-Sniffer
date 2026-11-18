#pragma once

#include <cstdint>
#include "esp_err.h"

namespace sim {
struct Address {
  uint32_t id = 0;
  bool ext = false;
  bool operator==(const Address&) const = default;
};

struct Frame {
  Address addr;
  uint8_t len = 0;
  bool rtr = false;
  uint8_t data[8]{};
  int64_t ts = 0;
};

struct CanStats {
  uint32_t rxDropped = 0, rxInvalid = 0, tx = 0, txFailed = 0, busOff = 0;
  bool ready = false;
};

namespace can {
enum class TxResult { none, success, failed };
// All controller lifecycle and TX functions run on the same core, under the model lock.
esp_err_t configure(uint32_t bitrate, bool ack);
esp_err_t deinit();
bool receive(Frame& frame);
bool transmit(const Frame& frame, int64_t now);
bool canTransmit();
TxResult service(int64_t now, Frame& completed);
CanStats stats();
}
}
