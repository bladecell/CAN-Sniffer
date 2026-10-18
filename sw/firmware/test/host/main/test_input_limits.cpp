// Input size caps (R1): the pure checks behind the PID file read, /ws frame and upload limits.
#include "input_limits.hpp"
#include "unity.h"

static void test_pid_file_size(void)
{
    TEST_ASSERT_TRUE(input_limits::pidFileSizeOk(1));
    TEST_ASSERT_TRUE(input_limits::pidFileSizeOk(262144));
    TEST_ASSERT_FALSE(input_limits::pidFileSizeOk(262145));
    TEST_ASSERT_FALSE(input_limits::pidFileSizeOk(0));
    TEST_ASSERT_FALSE(input_limits::pidFileSizeOk(static_cast<size_t>(-1)));
}

static void test_ws_frame_length(void)
{
    TEST_ASSERT_TRUE(input_limits::wsFrameLenOk(0));
    TEST_ASSERT_TRUE(input_limits::wsFrameLenOk(1));
    TEST_ASSERT_TRUE(input_limits::wsFrameLenOk(125));
    TEST_ASSERT_FALSE(input_limits::wsFrameLenOk(126));
    TEST_ASSERT_FALSE(input_limits::wsFrameLenOk(4096));
    TEST_ASSERT_FALSE(input_limits::wsFrameLenOk(static_cast<size_t>(-1)));
}

static void test_upload_length(void)
{
    TEST_ASSERT_TRUE(input_limits::uploadLenOk(0));
    TEST_ASSERT_TRUE(input_limits::uploadLenOk(4u * 1024 * 1024));
    TEST_ASSERT_FALSE(input_limits::uploadLenOk(4u * 1024 * 1024 + 1));
    TEST_ASSERT_FALSE(input_limits::uploadLenOk(16u * 1024 * 1024));
}

extern "C" void run_input_limits_tests(void)
{
  RUN_TEST(test_pid_file_size);
  RUN_TEST(test_ws_frame_length);
  RUN_TEST(test_upload_length);
}
