// Follow-up to P2.9: ESP_RST_SW (3) had no text and ESP_RST_EXT (2) was labelled "Software reset".
#include <cstring>

#include "../../../main/include/reset_reason.hpp"
#include "unity.h"

namespace
{
void test_sw_and_ext_are_distinct(void)
{
  TEST_ASSERT_EQUAL_STRING("External reset", reset_reason::text(2));
  TEST_ASSERT_EQUAL_STRING("Software reset", reset_reason::text(3));
  TEST_ASSERT_EQUAL_STRING("Deep sleep", reset_reason::text(8));
  TEST_ASSERT_EQUAL_STRING("CPU lockup", reset_reason::text(15));
}

void test_out_of_range(void)
{
  TEST_ASSERT_EQUAL_STRING("invalid reason", reset_reason::text(-1));
  TEST_ASSERT_EQUAL_STRING("invalid reason", reset_reason::text(16));
}
}  // namespace

extern "C" void run_reset_reason_tests(void)
{
  RUN_TEST(test_sw_and_ext_are_distinct);
  RUN_TEST(test_out_of_range);
}
