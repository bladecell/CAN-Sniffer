#pragma once

// Bounded ISO-TP flow-control responder for raw sends with fc=true (POST /can/mode {"mode":"raw","fc":true}). A raw send arms
// it for a short window; the first frame of a multi-frame message from an ECU (0x7E8-0x7EF) is then answered with the usual
// 30 00 00 to the physical request ID (response ID minus 8), once per ECU per arming, and only for the ECUs the arming
// send addressed (physical send: that ECU; functional 0x7DF: all). The first frame an addressed ECU sends after the arming
// uses up that ECU's share, so only the reply to the arming request can be answered, and the window (1.5 s) bounds the
// rest: an ECU that stays silent can still be answered for an unrelated first frame until the window ends. It keeps no payload; the consecutive
// frames are only seen in the raw stream. No ESP-IDF includes; host-tested. onFrame is called by the receive task only,
// arm and disarm from any task.
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace obd2frame
{
class RawFlowControl
{
public:
  static constexpr uint32_t kDefaultWindowMs = 1500;
  static constexpr uint32_t kMaxWindowMs     = 2000;

  // `requestId` is the id of the raw send that arms it: a physical request to 0x7E0+k answers only 0x7E8+k, the functional
  // 0x7DF answers every ECU, anything else arms nothing.
  void arm(uint32_t nowMs, uint32_t windowMs, uint32_t requestId)
  {
    const uint8_t mask = requestId == 0x7DF ? 0xFF : (requestId >= 0x7E0 && requestId <= 0x7E7 ? static_cast<uint8_t>(1u << (requestId - 0x7E0)) : 0);
    armed_.store(false);
    allowed_.store(mask);
    answered_.store(0);
    deadline_.store(nowMs + (windowMs > kMaxWindowMs ? kMaxWindowMs : windowMs));
    armed_.store(mask != 0);
  }
  void disarm() { armed_.store(false); }
  // Every raw send goes through here: with fc off it disarms, so a send without flow control never leaves an older arming
  // active.
  void apply(bool enable, uint32_t nowMs, uint32_t windowMs, uint32_t requestId)
  {
    if (enable) { arm(nowMs, windowMs, requestId); }
    else { disarm(); }
  }
  bool armed() const { return armed_.load(); }

  // True: send 30 00 00 to `fcId` now.
  bool onFrame(uint32_t nowMs, uint32_t id, bool ext, const uint8_t* data, size_t len, uint32_t& fcId)
  {
    if (!armed_.load() || ext || data == nullptr || len < 1 || id < 0x7E8 || id > 0x7EF) { return false; }
    if (static_cast<int32_t>(deadline_.load() - nowMs) <= 0)
    {
      armed_.store(false);
      return false;
    }
    const uint8_t bit = static_cast<uint8_t>(1u << (id - 0x7E8));
    if (!(allowed_.load() & bit)) { return false; }
    // The first frame an addressed ECU sends after the arming decides for that ECU, whatever it is: a single-frame reply
    // (the request is complete) uses the arming up, so a first frame of a later, unrelated message is not answered.
    if (answered_.fetch_or(bit) & bit) { return false; }
    if ((answered_.load() & allowed_.load()) == allowed_.load()) { armed_.store(false); }  // every addressed ECU is done
    if (len < 2 || (data[0] & 0xF0) != 0x10) { return false; }
    const unsigned total = (static_cast<unsigned>(data[0] & 0x0F) << 8) | data[1];
    if (total < 8) { return false; }  // a first frame always announces more than a single frame can carry
    fcId = id - 8;
    return true;
  }

private:
  std::atomic<bool>     armed_{false};
  std::atomic<uint32_t> deadline_{0};
  std::atomic<uint8_t>  allowed_{0};
  std::atomic<uint8_t>  answered_{0};
};
}  // namespace obd2frame
