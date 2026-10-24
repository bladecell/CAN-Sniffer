// Recorder pure core: state machine, CSV rows, trip names, PID list, decimation, flush/rotate policy.
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "recorder_core.hpp"
#include "recorder_queue.hpp"
#include "unity.h"

using namespace recorder_core;

static std::string fmt(const Row& r)
{
  char buf[kMaxRowBytes] = {};
  const size_t n = formatRow(buf, sizeof(buf), r);
  return std::string(buf, n);
}

static void test_row_valid_value(void)
{
  TEST_ASSERT_EQUAL_STRING("1500,12,2500.5,1\n", fmt({1500, 12, 2500.5f, true}).c_str());
  TEST_ASSERT_EQUAL_STRING("0,0,0,1\n", fmt({0, 0, 0.0f, true}).c_str());
  TEST_ASSERT_EQUAL_STRING("4294967295,65535,-40,1\n", fmt({4294967295u, 65535, -40.0f, true}).c_str());
  TEST_ASSERT_EQUAL_STRING("7,5,0.1234567,1\n", fmt({7, 5, 0.1234567f, true}).c_str());
  TEST_ASSERT_EQUAL_STRING("7,5,1234567.8,1\n", fmt({7, 5, 1234567.75f, true}).c_str());  // 8 digits keep a float exact enough
}

static void test_row_nan_inf_invalid_give_empty_value(void)
{
  const float inf = std::numeric_limits<float>::infinity();
  TEST_ASSERT_EQUAL_STRING("10,12,,0\n", fmt({10, 12, NAN, true}).c_str());
  TEST_ASSERT_EQUAL_STRING("10,12,,0\n", fmt({10, 12, inf, true}).c_str());
  TEST_ASSERT_EQUAL_STRING("10,12,,0\n", fmt({10, 12, -inf, true}).c_str());
  TEST_ASSERT_EQUAL_STRING("10,12,,0\n", fmt({10, 12, 55.0f, false}).c_str());
}

static void test_row_worst_case_fits_and_small_buffers_are_refused(void)
{
  const std::string worst = fmt({4294967295u, 65535, -std::numeric_limits<float>::max(), true});
  TEST_ASSERT_TRUE(worst.size() > 0 && worst.size() <= kMaxRowBytes);
  const std::string tiny = fmt({4294967295u, 65535, -1.17549435e-38f, true});
  TEST_ASSERT_TRUE(tiny.size() > 0 && tiny.size() <= kMaxRowBytes);
  char small[8];
  TEST_ASSERT_EQUAL(0, formatRow(small, sizeof(small), {1500, 12, 2500.5f, true}));
  TEST_ASSERT_EQUAL(0, formatRow(nullptr, 64, {1, 1, 1.0f, true}));
  TEST_ASSERT_EQUAL(0, formatRow(small, 0, {1, 1, 1.0f, true}));
  char exact[18];  // "1500,12,2500.5,1\n" is 17 bytes + NUL
  TEST_ASSERT_EQUAL(17, formatRow(exact, sizeof(exact), {1500, 12, 2500.5f, true}));
  TEST_ASSERT_EQUAL(0, formatRow(exact, 17, {1500, 12, 2500.5f, true}));
}

static void test_state_machine_legal_and_illegal_moves(void)
{
  StateMachine sm;
  TEST_ASSERT_EQUAL(State::Idle, sm.get());
  TEST_ASSERT_FALSE(sm.move(State::Idle, State::Stopping));
  TEST_ASSERT_FALSE(sm.move(State::Recording, State::Stopping));  // wrong "from"
  TEST_ASSERT_TRUE(sm.move(State::Idle, State::Recording));
  TEST_ASSERT_FALSE(sm.move(State::Recording, State::Recording));
  TEST_ASSERT_FALSE(sm.move(State::Recording, State::Idle));
  TEST_ASSERT_TRUE(sm.move(State::Recording, State::Error));
  TEST_ASSERT_TRUE(sm.move(State::Error, State::Recording));
  TEST_ASSERT_TRUE(sm.move(State::Recording, State::Stopping));
  TEST_ASSERT_FALSE(sm.move(State::Stopping, State::Error));
  TEST_ASSERT_FALSE(sm.move(State::Stopping, State::Recording));
  TEST_ASSERT_TRUE(sm.move(State::Stopping, State::Idle));
  TEST_ASSERT_TRUE(sm.move(State::Idle, State::Error));
  TEST_ASSERT_TRUE(sm.move(State::Error, State::Idle));
  TEST_ASSERT_TRUE(sm.move(State::Idle, State::Error));
  TEST_ASSERT_TRUE(sm.move(State::Error, State::Stopping));
}

static void test_callback_gate_refuses_after_close_and_waits_for_in_flight(void)
{
  CallbackGate g;
  TEST_ASSERT_FALSE(g.enter());  // closed by default
  TEST_ASSERT_TRUE(g.idle());    // the refused enter left nothing in flight
  g.open();
  TEST_ASSERT_TRUE(g.enter());
  TEST_ASSERT_FALSE(g.idle());
  g.close();  // stop arrives while a callback is running
  TEST_ASSERT_FALSE(g.enter());
  TEST_ASSERT_FALSE(g.idle());  // the running callback still holds the gate
  g.leave();
  TEST_ASSERT_TRUE(g.idle());
  TEST_ASSERT_FALSE(g.enter());
}

static void test_trip_names(void)
{
  char b[32];
  TEST_ASSERT_TRUE(formatTripName(b, sizeof(b), 42, false));
  TEST_ASSERT_EQUAL_STRING("trip_000042.csv", b);
  TEST_ASSERT_TRUE(formatTripName(b, sizeof(b), 42, true));
  TEST_ASSERT_EQUAL_STRING("trip_000042.json", b);
  TEST_ASSERT_TRUE(formatTripName(b, sizeof(b), kMaxTripIndex, false));
  TEST_ASSERT_EQUAL_STRING("trip_999999.csv", b);
  TEST_ASSERT_FALSE(formatTripName(b, sizeof(b), kMaxTripIndex + 1, false));
  TEST_ASSERT_FALSE(formatTripName(b, 15, 1, false));  // needs 16 with NUL
  TEST_ASSERT_FALSE(formatTripName(nullptr, 32, 1, false));
}

static void test_trip_index_parse_is_strict(void)
{
  uint32_t i = 7;
  TEST_ASSERT_TRUE(parseTripIndex("trip_000123.csv", i));
  TEST_ASSERT_EQUAL_UINT32(123, i);
  TEST_ASSERT_TRUE(parseTripIndex("trip_000124.json", i));
  TEST_ASSERT_EQUAL_UINT32(124, i);
  const char* bad[] = {"", "trip_1.csv", "trip_0000123.csv", "trip_00012a.csv", "trip_000123.txt", "trip_000123.csv.tmp",
                       "Trip_000123.csv", "trip_-00123.csv", "xtrip_000123.csv", "trip_000123", "trip_+00123.csv"};
  for (const char* s : bad)
  {
    i = 7;
    TEST_ASSERT_FALSE(parseTripIndex(s, i));
    TEST_ASSERT_EQUAL_UINT32(7, i);
  }
  TEST_ASSERT_FALSE(parseTripIndex(nullptr, i));
}

static void test_trip_scanner_picks_max_plus_one_and_ignores_garbage(void)
{
  TripScanner s;
  uint32_t n = 99;
  TEST_ASSERT_TRUE(s.next(n));
  TEST_ASSERT_EQUAL_UINT32(1, n);  // empty directory starts at 1
  s.add("trip_000003.csv");
  s.add("notes.txt");
  s.add("trip_000009.json");  // orphan sidecar still reserves its index
  s.add("trip_000004.csv");
  s.add(nullptr);
  TEST_ASSERT_TRUE(s.next(n));
  TEST_ASSERT_EQUAL_UINT32(10, n);
  s.add("trip_999999.csv");
  TEST_ASSERT_FALSE(s.next(n));  // exhausted: never wraps onto an old trip
}

static bool existsBelow100(uint16_t pid) { return pid < 100; }

static void test_pid_list_validation(void)
{
  const PidSel ok[] = {{12, 0}, {13, 500}, {5, kMaxSpacingMs}};
  TEST_ASSERT_EQUAL(ListError::Ok, validatePidList(ok, 3, existsBelow100));
  TEST_ASSERT_EQUAL(ListError::Empty, validatePidList(ok, 0, existsBelow100));
  TEST_ASSERT_EQUAL(ListError::Empty, validatePidList(nullptr, 3, existsBelow100));
  const PidSel dup[] = {{12, 0}, {13, 0}, {12, 100}};
  TEST_ASSERT_EQUAL(ListError::Duplicate, validatePidList(dup, 3, existsBelow100));
  const PidSel spacing[] = {{12, kMaxSpacingMs + 1}};
  TEST_ASSERT_EQUAL(ListError::BadSpacing, validatePidList(spacing, 1, existsBelow100));
  const PidSel unknown[] = {{12, 0}, {500, 0}};
  TEST_ASSERT_EQUAL(ListError::Unknown, validatePidList(unknown, 2, existsBelow100));
  PidSel many[kMaxPids + 1];
  for (size_t i = 0; i < kMaxPids + 1; ++i) many[i] = {static_cast<uint16_t>(i), 0};
  TEST_ASSERT_EQUAL(ListError::Ok, validatePidList(many, kMaxPids, existsBelow100));
  TEST_ASSERT_EQUAL(ListError::TooMany, validatePidList(many, kMaxPids + 1, existsBelow100));
  TEST_ASSERT_EQUAL(ListError::TooMany, validatePidList(many, static_cast<size_t>(-1), existsBelow100));
}

static void test_pid_filter_selects_and_decimates(void)
{
  PidFilter f;
  const PidSel sel[] = {{12, 0}, {13, 100}};
  TEST_ASSERT_TRUE(f.set(sel, 2));
  TEST_ASSERT_EQUAL(2, f.size());
  TEST_ASSERT_FALSE(f.accept(99, 1000));  // not selected
  TEST_ASSERT_TRUE(f.accept(12, 1000));
  TEST_ASSERT_FALSE(f.accept(12, 1000));  // same RX time again: a duplicate callback, not a new sample
  TEST_ASSERT_TRUE(f.accept(12, 1001));   // spacing 0 keeps every distinct update
  TEST_ASSERT_TRUE(f.accept(13, 1000));  // first sample always passes
  TEST_ASSERT_FALSE(f.accept(13, 1099));
  TEST_ASSERT_TRUE(f.accept(13, 1100));
  TEST_ASSERT_FALSE(f.accept(13, 1150));
  TEST_ASSERT_TRUE(f.accept(13, 1200));
}

static void test_pid_filter_decimation_survives_millisecond_wrap(void)
{
  PidFilter f;
  const PidSel sel[] = {{13, 100}};
  TEST_ASSERT_TRUE(f.set(sel, 1));
  TEST_ASSERT_TRUE(f.accept(13, 0xFFFFFFF0u));
  TEST_ASSERT_FALSE(f.accept(13, 0x00000010u));  // 32 ms later, across the wrap
  TEST_ASSERT_TRUE(f.accept(13, 0x00000054u));   // 100 ms after the first
}

static void test_pid_filter_rejects_oversized_or_null_lists(void)
{
  PidFilter f;
  PidSel many[kMaxPids + 1] = {};
  TEST_ASSERT_FALSE(f.set(many, kMaxPids + 1));
  TEST_ASSERT_FALSE(f.set(nullptr, 1));
  TEST_ASSERT_EQUAL(0, f.size());
  TEST_ASSERT_FALSE(f.accept(0, 5));
}

static void test_flush_policy(void)
{
  TEST_ASSERT_FALSE(shouldFlush(0, 100000, 0));  // nothing pending: never flush
  TEST_ASSERT_EQUAL_UINT32(10000, kFlushMs);  // data-loss window (docs/sd-card.md)
  TEST_ASSERT_EQUAL_UINT32(8192, kBatchBytes);
  TEST_ASSERT_FALSE(shouldFlush(100, 9999, 0));
  TEST_ASSERT_TRUE(shouldFlush(100, 10000, 0));
  TEST_ASSERT_FALSE(shouldFlush(100, 5, 0xFFFFFFF0u));  // wrap: 21 ms elapsed, not due
  TEST_ASSERT_FALSE(shouldFlush(kBatchBytes - kMaxRowBytes, 10, 0));
  TEST_ASSERT_TRUE(shouldFlush(kBatchBytes - kMaxRowBytes + 1, 10, 0));  // next row might not fit
}

static void test_flush_retry_is_throttled_only_when_degraded(void)
{
  TEST_ASSERT_TRUE(mayFlush(false, 10, 10));  // healthy: flush whenever shouldFlush says so
  TEST_ASSERT_FALSE(mayFlush(true, 9999, 0));
  TEST_ASSERT_TRUE(mayFlush(true, 10000, 0));
  TEST_ASSERT_FALSE(mayFlush(true, 5, 0xFFFFFFF0u));  // wrap safe
}

static void test_rotate_policy(void)
{
  TEST_ASSERT_FALSE(shouldRotate(0, 100));
  TEST_ASSERT_FALSE(shouldRotate(kRotateBytes - 101, 100));
  TEST_ASSERT_TRUE(shouldRotate(kRotateBytes - 100, 100));
  TEST_ASSERT_TRUE(shouldRotate(kRotateBytes * 2, 0));
}

static void test_fnv1a_known_vectors_and_chaining(void)
{
  TEST_ASSERT_EQUAL_HEX32(0x811C9DC5u, fnv1a32("", 0));
  TEST_ASSERT_EQUAL_HEX32(0xE40C292Cu, fnv1a32("a", 1));
  TEST_ASSERT_EQUAL_HEX32(0xBF9CF968u, fnv1a32("foobar", 6));
  TEST_ASSERT_EQUAL_HEX32(fnv1a32("foobar", 6), fnv1a32("bar", 3, fnv1a32("foo", 3)));
}

static void test_row_queue_drops_oldest_and_counts(void)
{
  RowQueue q;
  TEST_ASSERT_TRUE(q.create(4));
  for (uint16_t i = 0; i < 6; ++i) q.push({i, i, 1.0f, true});
  TEST_ASSERT_EQUAL_UINT32(2, q.dropped());
  Row r{};
  for (uint16_t want = 2; want < 6; ++want)
  {
    TEST_ASSERT_TRUE(q.pop(r, 0));
    TEST_ASSERT_EQUAL_UINT16(want, r.pid);
  }
  TEST_ASSERT_FALSE(q.pop(r, 0));
  q.push({9, 9, 2.0f, true});  // usable after draining
  TEST_ASSERT_TRUE(q.pop(r, 0));
  TEST_ASSERT_EQUAL_UINT16(9, r.pid);
  TEST_ASSERT_EQUAL_UINT32(2, q.dropped());
  q.destroy();
  q.push({1, 1, 1.0f, true});  // after destroy: ignored, no crash
  TEST_ASSERT_FALSE(q.pop(r, 0));
  q.destroy();
}

static void test_row_queue_create_rejects_zero_depth_and_recreates(void)
{
  RowQueue q;
  TEST_ASSERT_FALSE(q.create(0));
  TEST_ASSERT_TRUE(q.create(2));
  q.push({1, 1, 1.0f, true});
  q.push({2, 2, 1.0f, true});
  q.push({3, 3, 1.0f, true});
  TEST_ASSERT_EQUAL_UINT32(1, q.dropped());
  q.destroy();
  TEST_ASSERT_TRUE(q.create(2));
  TEST_ASSERT_EQUAL_UINT32(0, q.dropped());  // a new session starts from zero
  q.destroy();
}

static void test_newest_trips_keeps_highest_indices_descending(void)
{
  NewestTrips t;
  t.addName("trip_000003.csv");
  t.addName("trip_000003.json");  // sidecars and garbage are not listed
  t.addName("junk.csv");
  t.addName("trip_000010.csv");
  t.addName("trip_000001.csv");
  TEST_ASSERT_EQUAL(3, t.count());
  TEST_ASSERT_EQUAL_UINT32(10, t.at(0));
  TEST_ASSERT_EQUAL_UINT32(3, t.at(1));
  TEST_ASSERT_EQUAL_UINT32(1, t.at(2));
  TEST_ASSERT_FALSE(t.truncated());
  TEST_ASSERT_EQUAL_UINT32(0, t.at(3));  // out of range reads are harmless
}

static void test_newest_trips_is_bounded_and_reports_truncation(void)
{
  NewestTrips t;
  char name[24];
  for (uint32_t i = 1; i <= kMaxListedTrips + 20; ++i)
  {
    TEST_ASSERT_TRUE(formatTripName(name, sizeof(name), i, false));
    t.addName(name);
  }
  TEST_ASSERT_EQUAL(kMaxListedTrips, t.count());
  TEST_ASSERT_TRUE(t.truncated());
  TEST_ASSERT_EQUAL_UINT32(kMaxListedTrips + 20, t.at(0));
  TEST_ASSERT_EQUAL_UINT32(21, t.at(kMaxListedTrips - 1));
}

static void test_start_cooldown_window(void)
{
  TEST_ASSERT_EQUAL_UINT32(10000, kStartCooldownMs);
  TEST_ASSERT_EQUAL_UINT32(0, startCooldownRemainingMs(false, 5, 0));  // first start after boot
  TEST_ASSERT_EQUAL_UINT32(0, startCooldownRemainingMs(false, 0, 123));
  TEST_ASSERT_EQUAL_UINT32(10000, startCooldownRemainingMs(true, 1000, 1000));  // same tick as the mark
  TEST_ASSERT_EQUAL_UINT32(1, startCooldownRemainingMs(true, 10999, 1000));
  TEST_ASSERT_EQUAL_UINT32(0, startCooldownRemainingMs(true, 11000, 1000));  // exactly at the boundary
  TEST_ASSERT_EQUAL_UINT32(0, startCooldownRemainingMs(true, 500000, 1000));
}

static void test_start_cooldown_survives_tick_wrap(void)
{
  const uint32_t mark = 0xFFFFFFFFu - 2999u;  // 3 s before the wrap
  TEST_ASSERT_EQUAL_UINT32(3000, startCooldownRemainingMs(true, 4000, mark));   // 7 s elapsed across the wrap
  TEST_ASSERT_EQUAL_UINT32(1, startCooldownRemainingMs(true, 6999, mark));
  TEST_ASSERT_EQUAL_UINT32(0, startCooldownRemainingMs(true, 7000, mark));
}

extern "C" void run_recorder_core_tests(void)
{
  RUN_TEST(test_row_valid_value);
  RUN_TEST(test_row_nan_inf_invalid_give_empty_value);
  RUN_TEST(test_row_worst_case_fits_and_small_buffers_are_refused);
  RUN_TEST(test_state_machine_legal_and_illegal_moves);
  RUN_TEST(test_callback_gate_refuses_after_close_and_waits_for_in_flight);
  RUN_TEST(test_trip_names);
  RUN_TEST(test_trip_index_parse_is_strict);
  RUN_TEST(test_trip_scanner_picks_max_plus_one_and_ignores_garbage);
  RUN_TEST(test_pid_list_validation);
  RUN_TEST(test_pid_filter_selects_and_decimates);
  RUN_TEST(test_pid_filter_decimation_survives_millisecond_wrap);
  RUN_TEST(test_pid_filter_rejects_oversized_or_null_lists);
  RUN_TEST(test_flush_policy);
  RUN_TEST(test_flush_retry_is_throttled_only_when_degraded);
  RUN_TEST(test_rotate_policy);
  RUN_TEST(test_fnv1a_known_vectors_and_chaining);
  RUN_TEST(test_row_queue_drops_oldest_and_counts);
  RUN_TEST(test_row_queue_create_rejects_zero_depth_and_recreates);
  RUN_TEST(test_newest_trips_keeps_highest_indices_descending);
  RUN_TEST(test_newest_trips_is_bounded_and_reports_truncation);
  RUN_TEST(test_start_cooldown_window);
  RUN_TEST(test_start_cooldown_survives_tick_wrap);
}
