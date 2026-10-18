// P2.10a: debounced press/hold state machine of the BOOT button driver.
#include "../../../components/button/include/button_logic.h"
#include "unity.h"

namespace
{
constexpr int64_t kHold = 10LL * 1000 * 1000;

void test_press_release_reports_both_and_stamps_the_press(void)
{
  BtnState s;
  btnInit(&s, false);
  TEST_ASSERT_EQUAL_UINT32(BTN_EVENT_PRESS, btnSample(&s, true, 5000, kHold));
  TEST_ASSERT_EQUAL_INT64(5000, s.lastPressUs);
  TEST_ASSERT_EQUAL_UINT32(0, btnSample(&s, true, 6000, kHold));  // still down: nothing new
  TEST_ASSERT_EQUAL_UINT32(BTN_EVENT_RELEASE, btnSample(&s, false, 90000, kHold));
  TEST_ASSERT_EQUAL_INT64(5000, s.lastPressUs);  // release keeps the press time
}

void test_glitch_that_settled_high_is_no_press(void)
{
  BtnState s;
  btnInit(&s, false);
  TEST_ASSERT_EQUAL_UINT32(0, btnSample(&s, false, 1000, kHold));
  TEST_ASSERT_EQUAL_INT64(0, s.lastPressUs);
}

void test_held_at_init_is_ignored_until_released(void)
{
  BtnState s;
  btnInit(&s, true);
  TEST_ASSERT_EQUAL_UINT32(0, btnSample(&s, true, 1000, kHold));
  TEST_ASSERT_EQUAL_UINT32(0, btnSample(&s, true, kHold + 5000, kHold));  // not even a hold
  TEST_ASSERT_EQUAL_INT64(BTN_WAIT_FOREVER, btnWaitUs(&s, kHold + 5000, kHold));
  TEST_ASSERT_EQUAL_UINT32(0, btnSample(&s, false, kHold + 9000, kHold));  // release: armed now, no event
  TEST_ASSERT_EQUAL_INT64(0, s.lastPressUs);
  TEST_ASSERT_EQUAL_UINT32(BTN_EVENT_PRESS, btnSample(&s, true, kHold + 20000, kHold));
}

void test_hold_fires_once_per_press(void)
{
  BtnState s;
  btnInit(&s, false);
  btnSample(&s, true, 1000, kHold);
  TEST_ASSERT_EQUAL_INT64(kHold - 4000, btnWaitUs(&s, 5000, kHold));
  TEST_ASSERT_EQUAL_UINT32(0, btnSample(&s, true, 1000 + kHold - 1, kHold));
  TEST_ASSERT_EQUAL_UINT32(BTN_EVENT_HOLD, btnSample(&s, true, 1000 + kHold, kHold));
  TEST_ASSERT_EQUAL_UINT32(0, btnSample(&s, true, 1000 + 2 * kHold, kHold));
  TEST_ASSERT_EQUAL_INT64(BTN_WAIT_FOREVER, btnWaitUs(&s, 1000 + 2 * kHold, kHold));
  // the next press can fire again
  btnSample(&s, false, 1000 + 3 * kHold, kHold);
  btnSample(&s, true, 1000 + 4 * kHold, kHold);
  TEST_ASSERT_EQUAL_UINT32(BTN_EVENT_HOLD, btnSample(&s, true, 1000 + 5 * kHold, kHold));
}

void test_idle_button_waits_forever(void)
{
  BtnState s;
  btnInit(&s, false);
  TEST_ASSERT_EQUAL_INT64(BTN_WAIT_FOREVER, btnWaitUs(&s, 0, kHold));
}

void test_late_wake_reports_hold_and_zero_wait(void)
{
  BtnState s;
  btnInit(&s, false);
  btnSample(&s, true, 1000, kHold);
  TEST_ASSERT_EQUAL_INT64(0, btnWaitUs(&s, 1000 + kHold + 50, kHold));
}

void test_press_at_time_zero_still_counts(void)
{
  BtnState s;
  btnInit(&s, false);
  btnSample(&s, true, 0, kHold);
  TEST_ASSERT_TRUE(s.lastPressUs != 0);  // 0 means "no press"
}
}  // namespace

extern "C" void run_button_logic_tests(void)
{
  RUN_TEST(test_press_release_reports_both_and_stamps_the_press);
  RUN_TEST(test_glitch_that_settled_high_is_no_press);
  RUN_TEST(test_held_at_init_is_ignored_until_released);
  RUN_TEST(test_hold_fires_once_per_press);
  RUN_TEST(test_idle_button_waits_forever);
  RUN_TEST(test_late_wake_reports_hold_and_zero_wait);
  RUN_TEST(test_press_at_time_zero_still_counts);
}
