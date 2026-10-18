// WEB-7: POST /req/dtc without a mode may have one request in flight; a second is rejected at once.
#include "../../../main/include/single_flight.hpp"
#include "unity.h"

static bool earlyReturn(SingleFlight& f, bool fail)
{
  SingleFlight::Guard g(f);
  if (!g.held()) { return false; }
  if (fail) { return false; }  // error path: the guard must still release
  return true;
}

static void test_second_acquire_is_rejected_while_the_first_is_held(void)
{
  SingleFlight       f;
  SingleFlight::Guard a(f);
  TEST_ASSERT_TRUE(a.held());
  SingleFlight::Guard b(f);
  TEST_ASSERT_FALSE(b.held());
}

static void test_release_allows_the_next_request(void)
{
  SingleFlight f;
  {
    SingleFlight::Guard a(f);
    TEST_ASSERT_TRUE(a.held());
  }
  SingleFlight::Guard b(f);
  TEST_ASSERT_TRUE(b.held());
}

static void test_a_rejected_guard_does_not_release_the_holder(void)
{
  SingleFlight       f;
  SingleFlight::Guard a(f);
  {
    SingleFlight::Guard b(f);
    TEST_ASSERT_FALSE(b.held());
  }
  SingleFlight::Guard c(f);
  TEST_ASSERT_FALSE(c.held());
}

static void test_slot_is_released_on_the_error_path(void)
{
  SingleFlight f;
  TEST_ASSERT_FALSE(earlyReturn(f, true));
  TEST_ASSERT_TRUE(earlyReturn(f, false));
  TEST_ASSERT_TRUE(earlyReturn(f, false));
}

extern "C" void run_single_flight_tests(void)
{
  RUN_TEST(test_second_acquire_is_rejected_while_the_first_is_held);
  RUN_TEST(test_release_allows_the_next_request);
  RUN_TEST(test_a_rejected_guard_does_not_release_the_holder);
  RUN_TEST(test_slot_is_released_on_the_error_path);
}
