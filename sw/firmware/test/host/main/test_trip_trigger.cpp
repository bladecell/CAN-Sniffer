// TripTrigger: pure start/stop decisions for automatic trips (P3-R6). No clock, no IDF.
#include <cmath>
#include <limits>

#include "trip_trigger.hpp"
#include "unity.h"

using namespace trip_trigger;

namespace
{
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

Config autoCfg(bool cont = false) { return {Mode::Auto, cont}; }

Input in(uint32_t t, float rpm, float speed, bool rec = false, bool bus = true, bool canStart = true)
{
  return {bus, rpm, speed, kNan, rec, canStart, t};
}
}  // namespace

static void test_manual_mode_never_acts(void)
{
  TripTrigger t;
  const Config c{Mode::Manual, false};
  TEST_ASSERT_EQUAL(Action::None, t.step(c, in(0, 2000, 50)));
  TEST_ASSERT_EQUAL(Action::None, t.step(c, in(1000, 2000, 50, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(c, in(200000, 0, 0, true, false)));
}

static void test_auto_starts_on_rpm_or_speed_with_bus_up(void)
{
  TripTrigger a;
  TEST_ASSERT_EQUAL(Action::Start, a.step(autoCfg(), in(0, 800, 0)));
  TripTrigger b;
  TEST_ASSERT_EQUAL(Action::Start, b.step(autoCfg(), in(0, 0, 12.5f)));
}

static void test_auto_does_not_start_without_evidence(void)
{
  TripTrigger t;
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(0, 0, 0)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(1, 2000, 50, false, false)));  // bus down
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(2, kNan, kNan)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(3, kInf, kInf)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(4, -5, -1)));
}

static void test_start_waits_for_canStart(void)
{
  TripTrigger t;
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(0, 800, 0, false, true, false)));
  TEST_ASSERT_EQUAL(Action::Start, t.step(autoCfg(), in(1000, 800, 0, false, true, true)));
}

static void test_ends_after_60s_of_rpm_zero(void)
{
  TripTrigger t;
  TEST_ASSERT_EQUAL(Action::Start, t.step(autoCfg(), in(0, 800, 0)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(1000, 800, 0, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(10000, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(10000 + kEndIdleMs - 1, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(10000 + kEndIdleMs, 0, 0, true)));
}

static void test_running_sample_resets_idle_timer(void)
{
  TripTrigger t;
  t.step(autoCfg(), in(0, 800, 0));
  t.step(autoCfg(), in(1000, 0, 0, true));
  t.step(autoCfg(), in(50000, 900, 0, true));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(60000, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(60000 + kEndIdleMs - 1, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(60000 + kEndIdleMs, 0, 0, true)));
}

static void test_ends_after_60s_of_bus_lost(void)
{
  TripTrigger t;
  t.step(autoCfg(), in(0, 800, 0));
  t.step(autoCfg(), in(1000, 800, 0, true));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(2000, 800, 30, true, false)));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(2000 + kEndIdleMs, 800, 30, true, false)));
}

static void test_invalid_inputs_count_as_not_running(void)
{
  TripTrigger t;
  t.step(autoCfg(), in(0, 800, 0));
  t.step(autoCfg(), in(1000, kNan, kNan, true));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(1000 + kEndIdleMs, kInf, kNan, true)));
}

static void test_never_stops_a_manual_session(void)
{
  TripTrigger t;
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(0, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(500000, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(500001, 800, 0, true)));  // and does not start a second one
}

static void test_manual_stop_during_drive_does_not_restart_until_rearmed(void)
{
  TripTrigger t;
  t.step(autoCfg(), in(0, 800, 0));
  t.step(autoCfg(), in(1000, 800, 0, true));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(2000, 800, 40, false)));  // the user stopped it
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(3000, 800, 40, false)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(4000, 0, 0, false)));  // engine off: re-arm after 60 s
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(4000 + kEndIdleMs, 0, 0, false)));
  TEST_ASSERT_EQUAL(Action::Start, t.step(autoCfg(), in(4000 + kEndIdleMs + 1000, 800, 0, false)));
}

static void test_failed_start_is_not_retried_every_tick(void)
{
  TripTrigger t;
  TEST_ASSERT_EQUAL(Action::Start, t.step(autoCfg(), in(0, 800, 0)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(1000, 800, 0, false)));  // start did not produce a recording
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(2000, 800, 0, false)));
}

static void test_next_trip_starts_after_a_clean_stop(void)
{
  TripTrigger t;
  t.step(autoCfg(), in(0, 800, 0));
  t.step(autoCfg(), in(1000, 0, 0, true));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(1000 + kEndIdleMs, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(2000 + kEndIdleMs, 0, 0, false)));  // stopped
  TEST_ASSERT_EQUAL(Action::Start, t.step(autoCfg(), in(3000 + kEndIdleMs, 700, 0, false)));
}

static void test_stop_is_repeated_while_recording_persists(void)
{
  TripTrigger t;
  t.step(autoCfg(), in(0, 800, 0));
  t.step(autoCfg(), in(1000, 0, 0, true));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(1000 + kEndIdleMs, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(2000 + kEndIdleMs, 0, 0, true)));  // stop timed out
}

static void test_millis_wraparound(void)
{
  TripTrigger t;
  const uint32_t t0 = 0xFFFFFFFFu - 10000u;
  t.step(autoCfg(), in(t0, 800, 0));
  t.step(autoCfg(), in(t0 + 1000, 0, 0, true));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(t0 + 1000 + kEndIdleMs - 1, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::Stop, t.step(autoCfg(), in(t0 + 1000 + kEndIdleMs, 0, 0, true)));
}

static void test_imu_input_is_reserved_and_ignored(void)
{
  TripTrigger t;
  Input i = in(0, 0, 0);
  i.accelG = 2.5f;
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), i));  // motion alone does not start a trip yet
  i.accelG = kNan;
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), i));
}

static void test_switching_to_manual_forgets_ownership(void)
{
  TripTrigger t;
  t.step(autoCfg(), in(0, 800, 0));
  t.step(autoCfg(), in(1000, 0, 0, true));
  const Config m{Mode::Manual, false};
  TEST_ASSERT_EQUAL(Action::None, t.step(m, in(1000 + kEndIdleMs, 0, 0, true)));
  TEST_ASSERT_EQUAL(Action::None, t.step(autoCfg(), in(2000 + kEndIdleMs, 0, 0, true)));  // no longer ours
}

static void test_wantPolling_needs_opt_in_and_bus(void)
{
  TEST_ASSERT_TRUE(wantPolling({Mode::Manual, true}, true));
  TEST_ASSERT_TRUE(wantPolling({Mode::Auto, true}, true));
  TEST_ASSERT_FALSE(wantPolling({Mode::Auto, true}, false));
  TEST_ASSERT_FALSE(wantPolling({Mode::Auto, false}, true));
}

static void test_mode_names(void)
{
  Mode m = Mode::Manual;
  TEST_ASSERT_TRUE(parseMode("auto", m));
  TEST_ASSERT_EQUAL(Mode::Auto, m);
  TEST_ASSERT_TRUE(parseMode("manual", m));
  TEST_ASSERT_EQUAL(Mode::Manual, m);
  TEST_ASSERT_FALSE(parseMode("Auto", m));
  TEST_ASSERT_FALSE(parseMode("", m));
  TEST_ASSERT_FALSE(parseMode(nullptr, m));
  TEST_ASSERT_EQUAL_STRING("auto", modeName(Mode::Auto));
  TEST_ASSERT_EQUAL_STRING("manual", modeName(static_cast<Mode>(200)));  // unknown values read as manual
}

extern "C" void run_trip_trigger_tests(void)
{
  RUN_TEST(test_manual_mode_never_acts);
  RUN_TEST(test_auto_starts_on_rpm_or_speed_with_bus_up);
  RUN_TEST(test_auto_does_not_start_without_evidence);
  RUN_TEST(test_start_waits_for_canStart);
  RUN_TEST(test_ends_after_60s_of_rpm_zero);
  RUN_TEST(test_running_sample_resets_idle_timer);
  RUN_TEST(test_ends_after_60s_of_bus_lost);
  RUN_TEST(test_invalid_inputs_count_as_not_running);
  RUN_TEST(test_never_stops_a_manual_session);
  RUN_TEST(test_manual_stop_during_drive_does_not_restart_until_rearmed);
  RUN_TEST(test_failed_start_is_not_retried_every_tick);
  RUN_TEST(test_next_trip_starts_after_a_clean_stop);
  RUN_TEST(test_stop_is_repeated_while_recording_persists);
  RUN_TEST(test_millis_wraparound);
  RUN_TEST(test_imu_input_is_reserved_and_ignored);
  RUN_TEST(test_switching_to_manual_forgets_ownership);
  RUN_TEST(test_wantPolling_needs_opt_in_and_bus);
  RUN_TEST(test_mode_names);
}
