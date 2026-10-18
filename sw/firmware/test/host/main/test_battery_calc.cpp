// P2.9h: battery_read() used `int sum = 0, raw;` and ignored adc_oneshot_read errors, so a failed read added garbage.
#include <cmath>

#include "../../../main/include/battery_calc.hpp"
#include "unity.h"

namespace
{
constexpr float kRatio = (1000000.0f + 270000.0f) / 270000.0f;

void test_full_scale_and_mid_scale(void)
{
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 15.5f, battery_calc::averageVolts([](int& r) { r = 4095; return true; }, 16, kRatio));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, battery_calc::averageVolts([](int& r) { r = 0; return true; }, 16, kRatio));
}

void test_one_failed_read_makes_the_result_unknown(void)
{
  int n = 0;
  const float v = battery_calc::averageVolts([&](int& r) { r = 4000; return ++n != 7; }, 16, kRatio);
  TEST_ASSERT_TRUE(std::isnan(v));
}

void test_reader_that_never_sets_raw_is_unknown(void)
{
  // adc_oneshot_read on a null handle fails without touching raw; the old code summed the uninitialised value
  TEST_ASSERT_TRUE(std::isnan(battery_calc::averageVolts([](int&) { return false; }, 16, kRatio)));
}

void test_out_of_range_sample_and_bad_count(void)
{
  TEST_ASSERT_TRUE(std::isnan(battery_calc::averageVolts([](int& r) { r = 5000; return true; }, 16, kRatio)));
  TEST_ASSERT_TRUE(std::isnan(battery_calc::averageVolts([](int& r) { r = -1; return true; }, 16, kRatio)));
  TEST_ASSERT_TRUE(std::isnan(battery_calc::averageVolts([](int& r) { r = 100; return true; }, 0, kRatio)));
}
}  // namespace

extern "C" void run_battery_calc_tests(void)
{
  RUN_TEST(test_full_scale_and_mid_scale);
  RUN_TEST(test_one_failed_read_makes_the_result_unknown);
  RUN_TEST(test_reader_that_never_sets_raw_is_unknown);
  RUN_TEST(test_out_of_range_sample_and_bad_count);
}
