#pragma once

// SD capture of the raw CAN stream (UI3e-8, UI3e-16): file names, the CSV row writer and the size cap. No ESP-IDF includes,
// so host tests compile it directly. The writer turns one 0x10 stream batch (can_raw_core.hpp) into CSV text; the SD
// task and the REST routes live in main/src/can_capture.cpp. Layout and columns: docs/sd-card.md.
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace can_capture
{
constexpr size_t   kNameBytes    = 14;  // "cap_000001.csv"
constexpr uint32_t kMaxIndex     = 999999;
constexpr size_t   kMaxScan      = 2000;     // directory entries looked at; a larger folder refuses a new capture
constexpr size_t   kMaxListed    = 100;      // newest files in GET /can/captures
constexpr uint32_t kMaxFileBytes = 8u << 20;  // a capture ends when the file would pass this
constexpr char     kHeader[]     = "t_us,id,ext,rtr,dir,dlc,data\n";
constexpr size_t   kMaxRowBytes  = 64;  // 20 digit time + id + flags + 8 data bytes + newline, with room to spare

// "cap_NNNNNN.csv" and nothing else: no path part, no other digits count, no other suffix. Used for every name that reaches the
// SD card from a URL, so a hostile name ("../x", "cap_1.csv", "cap_000001.csv/") never becomes a path.
inline bool parseName(const char* name, uint32_t& index)
{
  if (name == nullptr || std::strlen(name) != kNameBytes || std::strncmp(name, "cap_", 4) != 0 ||
      std::strcmp(name + 10, ".csv") != 0)
  {
    return false;
  }
  uint32_t v = 0;
  for (size_t i = 4; i < 10; i++)
  {
    if (name[i] < '0' || name[i] > '9') { return false; }
    v = v * 10 + static_cast<uint32_t>(name[i] - '0');
  }
  if (v == 0) { return false; }
  index = v;
  return true;
}

inline bool formatName(char* out, size_t cap, uint32_t index)
{
  if (out == nullptr || cap < kNameBytes + 1 || index == 0 || index > kMaxIndex) { return false; }
  out[0] = 'c';
  out[1] = 'a';
  out[2] = 'p';
  out[3] = '_';
  for (int i = 9; i >= 4; i--)
  {
    out[i] = static_cast<char>('0' + index % 10);
    index /= 10;
  }
  out[10] = '.';
  out[11] = 'c';
  out[12] = 's';
  out[13] = 'v';
  out[14] = '\0';
  return true;
}

// The highest index among the names seen, so the next capture takes max + 1 and never overwrites.
class NameScanner
{
public:
  void add(const char* name)
  {
    uint32_t i = 0;
    if (parseName(name, i) && i > max_) { max_ = i; }
  }
  bool next(uint32_t& index) const
  {
    if (max_ >= kMaxIndex) { return false; }
    index = max_ + 1;
    return true;
  }

private:
  uint32_t max_ = 0;
};

// The newest kMaxListed capture indexes of a directory scan, highest first.
class NewestNames
{
public:
  void add(const char* name)
  {
    uint32_t i = 0;
    if (!parseName(name, i)) { return; }
    ++total_;
    size_t at = n_;
    while (at > 0 && idx_[at - 1] < i) { --at; }
    if (at >= kMaxListed) { return; }
    const size_t last = n_ < kMaxListed ? n_ : kMaxListed - 1;
    for (size_t k = last; k > at; --k) { idx_[k] = idx_[k - 1]; }
    idx_[at] = i;
    if (n_ < kMaxListed) { ++n_; }
  }
  size_t   count() const { return n_; }
  uint32_t at(size_t i) const { return i < n_ ? idx_[i] : 0; }
  bool     truncated() const { return total_ > n_; }

private:
  uint32_t idx_[kMaxListed]{};
  size_t   n_     = 0;
  size_t   total_ = 0;
};

namespace detail
{
inline size_t putDec(char* out, uint64_t v)
{
  char   tmp[20];
  size_t n = 0;
  do
  {
    tmp[n++] = static_cast<char>('0' + v % 10);
    v /= 10;
  } while (v != 0);
  for (size_t i = 0; i < n; i++) { out[i] = tmp[n - 1 - i]; }
  return n;
}
inline void putHex(char* out, uint32_t v, size_t digits)
{
  static const char kHex[] = "0123456789ABCDEF";
  for (size_t i = 0; i < digits; i++) { out[digits - 1 - i] = kHex[(v >> (4 * i)) & 0xF]; }
}
inline uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t get32(const uint8_t* p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
}  // namespace detail

// Turns 0x10 batches into CSV rows: `t_us,id,ext,rtr,dir,dlc,data`. t_us is microseconds since the first frame of the capture
// (the 32-bit stream clock is unwrapped here); id is upper-case hex, 3 digits standard and 8 extended; data is contiguous hex.
class RowWriter
{
public:
  void reset()
  {
    started_ = false;
    last32_  = 0;
    abs_     = 0;
    zero_    = 0;
  }

  // Appends the rows of one batch to `out`. Returns the bytes written, 0 when the batch is malformed or `cap` cannot hold all
  // of it (nothing is then written). `frames` gets the rows. A hostile or corrupt batch never reads past `len`.
  size_t addBatch(const uint8_t* batch, size_t len, char* out, size_t cap, size_t& frames)
  {
    frames = 0;
    if (batch == nullptr || out == nullptr || len < 8 || batch[0] != 0x10 || static_cast<size_t>(batch[1]) + 2 != len) { return 0; }
    uint32_t last32 = last32_;
    uint64_t abs    = abs_;
    uint64_t zero   = zero_;
    bool     first  = started_;
    const uint32_t base = detail::get32(batch + 4);
    if (!first) { abs = base; zero = base; last32 = base; first = true; }
    else { abs = static_cast<uint64_t>(static_cast<int64_t>(abs) + static_cast<int32_t>(base - last32)); last32 = base; }  // signed: a TX-done stamp can be earlier
    // The batch base is the first frame's time; every record adds a signed dt to the previous frame's time.
    size_t   pos = 8;
    size_t   w   = 0;
    uint32_t t32 = base;
    bool     firstRec = true;
    while (pos < len)
    {
      if (pos + 7 > len) { return 0; }
      const int16_t  dt  = static_cast<int16_t>(detail::get16(batch + pos));
      const uint32_t id  = detail::get32(batch + pos + 2);
      const uint8_t  ctl = batch[pos + 6];
      const size_t   n   = ctl & 0x0F;
      if (n > 8 || pos + 7 + n > len) { return 0; }
      if (!firstRec)
      {
        t32 += static_cast<uint32_t>(static_cast<int32_t>(dt));
        abs = static_cast<uint64_t>(static_cast<int64_t>(abs) + dt);
        last32 = t32;
      }
      firstRec = false;
      if (w + kMaxRowBytes > cap) { return 0; }
      const bool ext = (id & (1u << 31)) != 0;
      const bool rtr = (id & (1u << 30)) != 0;
      w += detail::putDec(out + w, abs >= zero ? abs - zero : 0);
      out[w++] = ',';
      detail::putHex(out + w, id & 0x1FFFFFFFu, ext ? 8 : 3);
      w += ext ? 8 : 3;
      out[w++] = ',';
      out[w++] = ext ? '1' : '0';
      out[w++] = ',';
      out[w++] = rtr ? '1' : '0';
      out[w++] = ',';
      out[w++] = (ctl & 0x80) != 0 ? 'T' : 'R';
      out[w++] = 'X';
      out[w++] = ',';
      out[w++] = static_cast<char>('0' + n);
      out[w++] = ',';
      for (size_t k = 0; k < n; k++)
      {
        detail::putHex(out + w, batch[pos + 7 + k], 2);
        w += 2;
      }
      out[w++] = '\n';
      pos += 7 + n;
      frames++;
    }
    started_ = true;
    last32_  = last32;
    abs_     = abs;
    zero_    = zero;
    return w;
  }

private:
  bool     started_ = false;
  uint32_t last32_  = 0;
  uint64_t abs_     = 0;
  uint64_t zero_    = 0;
};
}  // namespace can_capture
