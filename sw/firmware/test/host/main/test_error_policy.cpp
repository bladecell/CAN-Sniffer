// P2.9e: the supervisor ERROR state must end (restart after a bounded, growing back-off), never stay awake forever.
#include <cstdint>

#include "../../../main/include/error_policy.hpp"
#include "unity.h"

namespace
{
namespace ep = error_policy;

void test_backoff_doubles_from_the_base(void)
{
  TEST_ASSERT_EQUAL_UINT32(ep::kBaseBackoffS, ep::backoffS(0));
  TEST_ASSERT_EQUAL_UINT32(ep::kBaseBackoffS * 2, ep::backoffS(1));
  TEST_ASSERT_EQUAL_UINT32(ep::kBaseBackoffS * 4, ep::backoffS(2));
}

void test_backoff_is_capped(void)
{
  TEST_ASSERT_EQUAL_UINT32(ep::kMaxBackoffS, ep::backoffS(20));
  TEST_ASSERT_EQUAL_UINT32(ep::kMaxBackoffS, ep::backoffS(UINT32_MAX));  // garbage RTC value: no shift overflow
  for (uint32_t i = 0; i < 100; i++)
  {
    TEST_ASSERT_TRUE(ep::backoffS(i) >= ep::kBaseBackoffS && ep::backoffS(i) <= ep::kMaxBackoffS);
  }
}

void test_backoff_never_shrinks(void)
{
  for (uint32_t i = 1; i < 100; i++)
  {
    TEST_ASSERT_TRUE(ep::backoffS(i) >= ep::backoffS(i - 1));
  }
}

void test_stored_count_needs_magic_and_sane_value(void)
{
  TEST_ASSERT_EQUAL_UINT32(3, ep::validCount(ep::kMagic, 3));
  TEST_ASSERT_EQUAL_UINT32(0, ep::validCount(0xDEADBEEF, 3));  // power-on garbage
  TEST_ASSERT_EQUAL_UINT32(ep::kMaxCount, ep::validCount(ep::kMagic, 1000000));
}
void test_nvs_reset_is_bounded(void)
{
  TEST_ASSERT_TRUE(ep::nvsResetAllowed(0));
  TEST_ASSERT_TRUE(ep::nvsResetAllowed(ep::kMaxNvsResets - 1));
  TEST_ASSERT_FALSE(ep::nvsResetAllowed(ep::kMaxNvsResets));
  TEST_ASSERT_FALSE(ep::nvsResetAllowed(ep::validCount(ep::kMagic, 1000)));
  TEST_ASSERT_TRUE(ep::nvsResetAllowed(ep::validCount(0xDEADBEEF, 99)));  // garbage RTC reads as 0
}
}  // namespace

extern "C" void run_error_policy_tests(void)
{
  RUN_TEST(test_backoff_doubles_from_the_base);
  RUN_TEST(test_backoff_is_capped);
  RUN_TEST(test_backoff_never_shrinks);
  RUN_TEST(test_stored_count_needs_magic_and_sane_value);
  RUN_TEST(test_nvs_reset_is_bounded);
}
