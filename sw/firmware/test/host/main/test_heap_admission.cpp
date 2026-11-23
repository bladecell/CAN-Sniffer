// B-0230: internal-RAM admission and NULL thread-semaphore handling, hardware-free.
#include "../../../components/async_web_server/include/heap_admission.hpp"
#include "unity.h"

namespace
{
namespace ha = heap_admission;

void test_admit_at_threshold()
{
  TEST_ASSERT_TRUE(ha::admit(ha::kMinInternalFreeBytes, "/api/v1/vin"));
  TEST_ASSERT_TRUE(ha::admit(ha::kMinInternalFreeBytes + 1, "/api/v1/vin"));
  TEST_ASSERT_FALSE(ha::admit(ha::kMinInternalFreeBytes - 1, "/api/v1/vin"));
  TEST_ASSERT_FALSE(ha::admit(0, "/api/v1/sd_card/tree/a"));
  TEST_ASSERT_FALSE(ha::admit(0, nullptr));
}

void test_threshold_is_8_kib()
{
  TEST_ASSERT_EQUAL_UINT32(8192, ha::kMinInternalFreeBytes);
  TEST_ASSERT_TRUE(ha::admit(8192, "/api/v1/vin"));
  TEST_ASSERT_FALSE(ha::admit(8191, "/api/v1/vin"));
  TEST_ASSERT_TRUE(ha::admit(26 * 1024, "/api/v1/vin"));
}

void test_critical_routes_stay_open()
{
  TEST_ASSERT_TRUE(ha::admit(0, "/api/v1/system"));
  TEST_ASSERT_TRUE(ha::admit(0, "/api/v1/system?x=1"));
  TEST_ASSERT_TRUE(ha::admit(0, "/api/v1/auth"));
  TEST_ASSERT_TRUE(ha::admit(0, "/api/v1/auth/login"));
}

void test_critical_is_exact()
{
  TEST_ASSERT_FALSE(ha::admit(0, "/api/v1/system/reboot"));
  TEST_ASSERT_FALSE(ha::admit(0, "/api/v1/systemx"));
  TEST_ASSERT_FALSE(ha::admit(0, "/api/v1/authx"));
  TEST_ASSERT_FALSE(ha::admit(0, "/api/v1/vin?u=/api/v1/system"));
  TEST_ASSERT_FALSE(ha::admit(0, ""));
}

void test_null_thread_sem_is_not_ready()
{
  TEST_ASSERT_FALSE(ha::threadSemReady([]() -> void* { return nullptr; }));
  static int sem;
  TEST_ASSERT_TRUE(ha::threadSemReady([]() -> void* { return &sem; }));
}
void test_all_workers_down()
{
    TEST_ASSERT_FALSE(ha::allWorkersDown(0, 0));
    TEST_ASSERT_FALSE(ha::allWorkersDown(0, 6));
    TEST_ASSERT_FALSE(ha::allWorkersDown(5, 6));
    TEST_ASSERT_TRUE(ha::allWorkersDown(6, 6));
}
}  // namespace

extern "C" void run_heap_admission_tests(void)
{
  RUN_TEST(test_admit_at_threshold);
  RUN_TEST(test_threshold_is_8_kib);
  RUN_TEST(test_critical_routes_stay_open);
  RUN_TEST(test_critical_is_exact);
  RUN_TEST(test_null_thread_sem_is_not_ready);
  RUN_TEST(test_all_workers_down);
}
