// SD health: consecutive I/O timeouts mark the card unavailable; the state frame is 0x06 (docs/api.md).
#include <cerrno>
#include <cstring>

#include "sd_health.hpp"
#include "unity.h"

using namespace sd_health;

static void test_three_consecutive_timeouts_trip(void)
{
  Tracker t;
  t.noteMounted();
  TEST_ASSERT_FALSE(t.reportIo(ESP_ERR_TIMEOUT));
  TEST_ASSERT_FALSE(t.reportIo(ESP_ERR_TIMEOUT));
  TEST_ASSERT_FALSE(t.unavailable());
  TEST_ASSERT_TRUE(t.reportIo(ESP_ERR_TIMEOUT));
  TEST_ASSERT_TRUE(t.unavailable());
  TEST_ASSERT_FALSE(t.reportIo(ESP_ERR_TIMEOUT));  // trips once
}

static void test_success_resets_the_count(void)
{
  Tracker t;
  t.noteMounted();
  t.reportIo(ESP_ERR_TIMEOUT);
  t.reportIo(ESP_ERR_TIMEOUT);
  t.reportIo(ESP_OK);
  t.reportIo(ESP_ERR_TIMEOUT);
  TEST_ASSERT_FALSE(t.reportIo(ESP_ERR_TIMEOUT));
  TEST_ASSERT_FALSE(t.unavailable());
}

static void test_other_errors_do_not_count(void)
{
  Tracker t;
  t.noteMounted();
  for (int i = 0; i < 10; ++i) TEST_ASSERT_FALSE(t.reportIo(ESP_FAIL));
  TEST_ASSERT_FALSE(t.unavailable());
}

static void test_trip_publishes_unmounted_unavailable(void)
{
  Tracker t;
  State   s;
  t.noteMounted();
  TEST_ASSERT_TRUE(t.takeChange(s));
  TEST_ASSERT_TRUE(s.mounted);
  TEST_ASSERT_FALSE(s.unavailable);
  t.reportIo(ESP_ERR_TIMEOUT);
  t.reportIo(ESP_ERR_TIMEOUT);
  TEST_ASSERT_FALSE(t.takeChange(s));
  t.reportIo(ESP_ERR_TIMEOUT);
  t.noteUnmounted(Reason::Unmounted);  // the unmount that follows must keep the reason
  TEST_ASSERT_TRUE(t.takeChange(s));
  TEST_ASSERT_FALSE(s.mounted);
  TEST_ASSERT_TRUE(s.unavailable);
  TEST_ASSERT_EQUAL(Reason::IoTimeouts, s.reason);
  TEST_ASSERT_FALSE(t.takeChange(s));
}

static void test_mount_and_removal_clear_the_flag(void)
{
  Tracker t;
  State   s;
  t.noteMounted();
  for (int i = 0; i < 3; ++i) t.reportIo(ESP_ERR_TIMEOUT);
  t.noteMounted();
  TEST_ASSERT_FALSE(t.unavailable());
  TEST_ASSERT_TRUE(t.takeChange(s));
  TEST_ASSERT_TRUE(s.mounted);
  TEST_ASSERT_EQUAL(Reason::Mounted, s.reason);
  for (int i = 0; i < 3; ++i) t.reportIo(ESP_ERR_TIMEOUT);
  t.noteRemoved();
  TEST_ASSERT_FALSE(t.unavailable());
  TEST_ASSERT_TRUE(t.takeChange(s));
  TEST_ASSERT_FALSE(s.mounted);
  TEST_ASSERT_EQUAL(Reason::Removed, s.reason);
}

static void test_trip_publishes_without_waiting_for_the_unmount(void)
{
  Tracker t;
  State   s;
  t.noteMounted();
  t.takeChange(s);
  for (int i = 0; i < 3; ++i) t.reportIo(ESP_ERR_TIMEOUT);
  TEST_ASSERT_TRUE(t.takeChange(s));  // the frame never says mounted while unavailable, even if the unmount is still pending
  TEST_ASSERT_FALSE(s.mounted);
  TEST_ASSERT_TRUE(s.unavailable);
  TEST_ASSERT_EQUAL(Reason::IoTimeouts, s.reason);
}

static void test_deinit_clears_unavailable_with_reason_unmounted(void)
{
  Tracker t;
  State   s;
  t.noteMounted();
  for (int i = 0; i < 3; ++i) t.reportIo(ESP_ERR_TIMEOUT);
  t.takeChange(s);
  t.noteDeinit();
  TEST_ASSERT_FALSE(t.unavailable());
  TEST_ASSERT_TRUE(t.takeChange(s));
  TEST_ASSERT_FALSE(s.mounted);
  TEST_ASSERT_FALSE(s.unavailable);
  TEST_ASSERT_EQUAL(Reason::Unmounted, s.reason);
  TEST_ASSERT_FALSE(t.retryDue(100000));  // no retry for a card that was shut down on purpose
}

static void test_retry_backoff(void)
{
  Tracker t;
  TEST_ASSERT_FALSE(t.retryDue(1000));  // not unavailable
  t.noteMounted();
  for (int i = 0; i < 3; ++i) t.reportIo(ESP_ERR_TIMEOUT);
  TEST_ASSERT_FALSE(t.retryDue(5000));  // arms the timer
  TEST_ASSERT_FALSE(t.retryDue(5000 + kRetryBackoffMs - 1));
  TEST_ASSERT_TRUE(t.retryDue(5000 + kRetryBackoffMs));
  TEST_ASSERT_FALSE(t.retryDue(5000 + kRetryBackoffMs + 1));  // one attempt per period
}

static void test_errno_classification(void)
{
  TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, ioError(EIO));
  TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, ioError(ETIMEDOUT));
  TEST_ASSERT_EQUAL(ESP_FAIL, ioError(ENOSPC));
  TEST_ASSERT_EQUAL(ESP_FAIL, ioError(0));
}

static void test_state_frame(void)
{
  uint8_t out[kStateFrameBytes + 2];
  memset(out, 0xEE, sizeof(out));
  const State s{false, true, Reason::IoTimeouts};
  TEST_ASSERT_EQUAL(kStateFrameBytes, encodeState(s, out, sizeof(out)));
  const uint8_t want[kStateFrameBytes] = {0x06, 3, 0, 1, 2};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, kStateFrameBytes);
  TEST_ASSERT_EQUAL_UINT8(0xEE, out[kStateFrameBytes]);  // nothing past the frame
  const State m{true, false, Reason::Mounted};
  TEST_ASSERT_EQUAL(kStateFrameBytes, encodeState(m, out, sizeof(out)));
  const uint8_t want2[kStateFrameBytes] = {0x06, 3, 1, 0, 0};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(want2, out, kStateFrameBytes);
}

static void test_state_frame_rejects_small_buffer(void)
{
  uint8_t out[kStateFrameBytes];
  TEST_ASSERT_EQUAL(0, encodeState(State{}, out, kStateFrameBytes - 1));
  TEST_ASSERT_EQUAL(0, encodeState(State{}, nullptr, 8));
}

extern "C" void run_sd_health_tests(void)
{
  RUN_TEST(test_three_consecutive_timeouts_trip);
  RUN_TEST(test_success_resets_the_count);
  RUN_TEST(test_other_errors_do_not_count);
  RUN_TEST(test_trip_publishes_unmounted_unavailable);
  RUN_TEST(test_mount_and_removal_clear_the_flag);
  RUN_TEST(test_trip_publishes_without_waiting_for_the_unmount);
  RUN_TEST(test_deinit_clears_unavailable_with_reason_unmounted);
  RUN_TEST(test_retry_backoff);
  RUN_TEST(test_errno_classification);
  RUN_TEST(test_state_frame);
  RUN_TEST(test_state_frame_rejects_small_buffer);
}
