// can_capture core: file names, scanner, CSV row writer and its bounds (no ESP-IDF).
#include <cstring>
#include <string>

#include "can_capture_core.hpp"
#include "can_raw_core.hpp"
#include "unity.h"

using namespace can_capture;

static can_raw::Frame mk(uint32_t id, uint32_t ts, std::initializer_list<uint8_t> d, bool tx = false, bool ext = false, bool rtr = false)
{
  uint8_t buf[8] = {};
  size_t  n      = 0;
  for (uint8_t b : d) { buf[n++] = b; }
  return can_raw::makeFrame(id, ext, rtr, n, buf, ts, tx);
}

static std::string rows(const can_raw::Frame* f, size_t n, RowWriter& w)
{
  uint8_t buf[can_raw::kMaxBatchBytes];
  size_t  used = 0;
  const size_t bytes = can_raw::encodeBatch(f, n, 0, buf, sizeof(buf), used);
  char   out[4096];
  size_t frames = 0;
  const size_t wr = w.addBatch(buf, bytes, out, sizeof(out), frames);
  TEST_ASSERT_EQUAL(used, frames);
  return std::string(out, wr);
}

static void test_names_are_strict(void)
{
  uint32_t i = 0;
  TEST_ASSERT_TRUE(parseName("cap_000001.csv", i));
  TEST_ASSERT_EQUAL_UINT32(1, i);
  TEST_ASSERT_TRUE(parseName("cap_999999.csv", i));
  TEST_ASSERT_EQUAL_UINT32(999999, i);
  const char* bad[] = {"", "../cap_000001.csv", "cap_000001.csv/", "/cap_000001.csv", "cap_1.csv", "cap_0000001.csv", "cap_000000.csv",
                       "cap_00000a.csv", "cap_000001.CSV", "cap_000001.csv ", " cap_000001.csv", "trip_000001.csv", "cap_000001.json",
                       "cap_000001.csv\n", "cap_00-001.csv", "cap_000001.csv%00", "..", ".", "cap_000001/../x.csv", "cap_\xC3\xA9"};
  for (const char* b : bad) { TEST_ASSERT_FALSE_MESSAGE(parseName(b, i), b); }
  TEST_ASSERT_FALSE(parseName(nullptr, i));
}

static void test_format_name_round_trips_and_checks_bounds(void)
{
  char n[32];
  TEST_ASSERT_TRUE(formatName(n, sizeof(n), 42));
  TEST_ASSERT_EQUAL_STRING("cap_000042.csv", n);
  TEST_ASSERT_FALSE(formatName(n, sizeof(n), 0));
  TEST_ASSERT_FALSE(formatName(n, sizeof(n), 1000000));
  TEST_ASSERT_FALSE(formatName(n, 14, 1));  // no room for the NUL
  TEST_ASSERT_FALSE(formatName(nullptr, 32, 1));
}

static void test_scanner_picks_next_index_and_lists_newest_first(void)
{
  NameScanner s;
  s.add("cap_000003.csv");
  s.add("cap_000010.csv");
  s.add("notes.txt");
  s.add("cap_bad.csv");
  uint32_t next = 0;
  TEST_ASSERT_TRUE(s.next(next));
  TEST_ASSERT_EQUAL_UINT32(11, next);
  NameScanner full;
  full.add("cap_999999.csv");
  TEST_ASSERT_FALSE(full.next(next));  // never reuses a number
  NewestNames n;
  for (uint32_t i = 1; i <= kMaxListed + 5; i++)
  {
    char nm[24];
    formatName(nm, sizeof(nm), i);
    n.add(nm);
  }
  TEST_ASSERT_EQUAL(kMaxListed, n.count());
  TEST_ASSERT_EQUAL_UINT32(kMaxListed + 5, n.at(0));
  TEST_ASSERT_TRUE(n.truncated());
}

static void test_csv_rows(void)
{
  RowWriter w;
  can_raw::Frame f[3] = {mk(0x7E8, 1000, {0x41, 0x0C, 0x1A, 0xF8}), mk(0x7DF, 1250, {0x02, 0x01, 0x0C}, true),
                         mk(0x18DAF110, 1300, {}, false, true, true)};
  TEST_ASSERT_EQUAL_STRING("0,7E8,0,0,RX,4,410C1AF8\n250,7DF,0,0,TX,3,02010C\n300,18DAF110,1,1,RX,0,\n", rows(f, 3, w).c_str());
}

static void test_time_is_relative_across_batches_and_wraps(void)
{
  RowWriter w;
  can_raw::Frame a[1] = {mk(1, 0xFFFFFF00u, {1})};
  TEST_ASSERT_EQUAL_STRING("0,001,0,0,RX,1,01\n", rows(a, 1, w).c_str());
  can_raw::Frame b[1] = {mk(1, 0x00000100u, {2})};  // the 32-bit clock wrapped: 0x200 us later
  TEST_ASSERT_EQUAL_STRING("512,001,0,0,RX,1,02\n", rows(b, 1, w).c_str());
  can_raw::Frame c[1] = {mk(1, 0x000000F0u, {3})};  // 16 us earlier than the last frame (a TX-done stamp)
  const std::string r = rows(c, 1, w);
  TEST_ASSERT_EQUAL_STRING("496,001,0,0,RX,1,03\n", r.c_str());
  w.reset();
  TEST_ASSERT_EQUAL_STRING("0,001,0,0,RX,1,01\n", rows(a, 1, w).c_str());
}

static void test_a_full_batch_of_empty_frames_fits_one_buffer(void)
{
  RowWriter      w;
  can_raw::Frame f[35];
  for (int i = 0; i < 35; i++) { f[i] = mk(0x1FFFFFFF, 4000000000u + i * 3, {}, true, true, true); }
  uint8_t buf[can_raw::kMaxBatchBytes];
  size_t  used  = 0;
  const size_t bytes = can_raw::encodeBatch(f, 35, 0, buf, sizeof(buf), used);
  TEST_ASSERT_GREATER_THAN(0, used);
  char   out[2304];
  size_t frames = 0;
  TEST_ASSERT_GREATER_THAN(0, w.addBatch(buf, bytes, out, sizeof(out), frames));
  TEST_ASSERT_EQUAL(used, frames);
}

static void test_hostile_batches_are_refused(void)
{
  RowWriter w;
  char      out[256];
  size_t    frames = 9;
  uint8_t   ok[] = {0x10, 0x0F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02, 0x03, 0x02, 0xAA};  // len byte says 15 but this is a record of 2 bytes
  TEST_ASSERT_EQUAL(0, w.addBatch(nullptr, 10, out, sizeof(out), frames));
  TEST_ASSERT_EQUAL(0, frames);
  TEST_ASSERT_EQUAL(0, w.addBatch(ok, 3, out, sizeof(out), frames));                       // too short
  uint8_t wrongType[16];
  std::memcpy(wrongType, ok, sizeof(ok));
  wrongType[0] = 0x11;
  TEST_ASSERT_EQUAL(0, w.addBatch(wrongType, sizeof(ok), out, sizeof(out), frames));
  uint8_t badLen[16];
  std::memcpy(badLen, ok, sizeof(ok));
  badLen[1] = 0x20;  // length byte disagrees with the buffer
  TEST_ASSERT_EQUAL(0, w.addBatch(badLen, sizeof(ok), out, sizeof(out), frames));
  uint8_t longData[16] = {0x10, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0F, 0xAA, 0xAA};  // data nibble 15 (> 8), cut short
  longData[1] = static_cast<uint8_t>(sizeof(longData) - 2);
  TEST_ASSERT_EQUAL(0, w.addBatch(longData, sizeof(longData), out, sizeof(out), frames));
  uint8_t trunc[12] = {0x10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x00};  // record header cut off
  TEST_ASSERT_EQUAL(0, w.addBatch(trunc, sizeof(trunc), out, sizeof(out), frames));
  // output buffer too small: nothing is written and the writer state does not advance
  can_raw::Frame f[2] = {mk(1, 10, {1}), mk(2, 20, {2})};
  uint8_t        buf[can_raw::kMaxBatchBytes];
  size_t         used  = 0;
  const size_t   bytes = can_raw::encodeBatch(f, 2, 0, buf, sizeof(buf), used);
  char           tiny[kMaxRowBytes + 8];
  TEST_ASSERT_EQUAL(0, w.addBatch(buf, bytes, tiny, sizeof(tiny), frames));
  char big[256];
  TEST_ASSERT_GREATER_THAN(0, w.addBatch(buf, bytes, big, sizeof(big), frames));
  TEST_ASSERT_EQUAL_STRING_LEN("0,001", big, 5);
}

static void test_cap_constants(void)
{
  TEST_ASSERT_EQUAL(14, kNameBytes);
  TEST_ASSERT_EQUAL_UINT32(8u << 20, kMaxFileBytes);
  TEST_ASSERT_EQUAL_STRING("t_us,id,ext,rtr,dir,dlc,data\n", kHeader);
}

extern "C" void run_can_capture_tests(void)
{
  RUN_TEST(test_names_are_strict);
  RUN_TEST(test_format_name_round_trips_and_checks_bounds);
  RUN_TEST(test_scanner_picks_next_index_and_lists_newest_first);
  RUN_TEST(test_csv_rows);
  RUN_TEST(test_time_is_relative_across_batches_and_wraps);
  RUN_TEST(test_a_full_batch_of_empty_frames_fits_one_buffer);
  RUN_TEST(test_hostile_batches_are_refused);
  RUN_TEST(test_cap_constants);
}
