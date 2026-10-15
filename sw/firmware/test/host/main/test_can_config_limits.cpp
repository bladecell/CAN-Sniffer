// B48: CAN queue size bounds. A zero rx queue aborts xQueueCreate and a persisted bad value would boot-loop.
#include <cstddef>
#include <cstdint>
#include <limits>

#include "../../../components/can_driver/include/can_config_limits.hpp"
#include "unity.h"

namespace
{
void test_defaults_valid(void)
{
  TEST_ASSERT_TRUE(can_limits::canConfigValid(20, 20));
}

void test_rx_bounds(void)
{
  TEST_ASSERT_FALSE(can_limits::rxQueueSizeValid(0));
  TEST_ASSERT_TRUE(can_limits::rxQueueSizeValid(1));
  TEST_ASSERT_TRUE(can_limits::rxQueueSizeValid(256));
  TEST_ASSERT_FALSE(can_limits::rxQueueSizeValid(257));
  TEST_ASSERT_FALSE(can_limits::rxQueueSizeValid(static_cast<double>(SIZE_MAX)));
}

void test_tx_bounds(void)
{
  TEST_ASSERT_FALSE(can_limits::txQueueDepthValid(0));
  TEST_ASSERT_TRUE(can_limits::txQueueDepthValid(1));
  TEST_ASSERT_TRUE(can_limits::txQueueDepthValid(64));
  TEST_ASSERT_FALSE(can_limits::txQueueDepthValid(65));
  TEST_ASSERT_FALSE(can_limits::txQueueDepthValid(static_cast<double>(UINT32_MAX)));
}

void test_hostile_json_numbers(void)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  TEST_ASSERT_FALSE(can_limits::rxQueueSizeValid(-1));
  TEST_ASSERT_FALSE(can_limits::rxQueueSizeValid(nan));
  TEST_ASSERT_FALSE(can_limits::rxQueueSizeValid(inf));
  TEST_ASSERT_FALSE(can_limits::rxQueueSizeValid(20.5));
  TEST_ASSERT_FALSE(can_limits::txQueueDepthValid(-inf));
  TEST_ASSERT_FALSE(can_limits::txQueueDepthValid(1e30));
}

void test_pair(void)
{
  TEST_ASSERT_FALSE(can_limits::canConfigValid(0, 20));
  TEST_ASSERT_FALSE(can_limits::canConfigValid(20, 0));
  TEST_ASSERT_FALSE(can_limits::canConfigValid(20, 4096));
  TEST_ASSERT_TRUE(can_limits::canConfigValid(256, 64));
}
}  // namespace

extern "C" void run_can_config_limits_tests(void)
{
  RUN_TEST(test_defaults_valid);
  RUN_TEST(test_rx_bounds);
  RUN_TEST(test_tx_bounds);
  RUN_TEST(test_hostile_json_numbers);
  RUN_TEST(test_pair);
}
