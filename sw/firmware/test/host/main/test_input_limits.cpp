// Input size caps (R1): the pure checks behind the PID file read, /ws frame and upload limits.
#include "input_limits.hpp"
#include "unity.h"

TEST_CASE("PID file size: at the cap passes, above or empty is rejected", "[limits]")
{
    TEST_ASSERT_TRUE(input_limits::pidFileSizeOk(1));
    TEST_ASSERT_TRUE(input_limits::pidFileSizeOk(262144));
    TEST_ASSERT_FALSE(input_limits::pidFileSizeOk(262145));
    TEST_ASSERT_FALSE(input_limits::pidFileSizeOk(0));
    TEST_ASSERT_FALSE(input_limits::pidFileSizeOk(static_cast<size_t>(-1)));
}

TEST_CASE("WS frame length: command and control frames pass, larger is rejected", "[limits]")
{
    TEST_ASSERT_TRUE(input_limits::wsFrameLenOk(0));
    TEST_ASSERT_TRUE(input_limits::wsFrameLenOk(1));
    TEST_ASSERT_TRUE(input_limits::wsFrameLenOk(125));
    TEST_ASSERT_FALSE(input_limits::wsFrameLenOk(126));
    TEST_ASSERT_FALSE(input_limits::wsFrameLenOk(4096));
    TEST_ASSERT_FALSE(input_limits::wsFrameLenOk(static_cast<size_t>(-1)));
}

TEST_CASE("Upload length: at the cap passes, above is rejected", "[limits]")
{
    TEST_ASSERT_TRUE(input_limits::uploadLenOk(0));
    TEST_ASSERT_TRUE(input_limits::uploadLenOk(4u * 1024 * 1024));
    TEST_ASSERT_FALSE(input_limits::uploadLenOk(4u * 1024 * 1024 + 1));
    TEST_ASSERT_FALSE(input_limits::uploadLenOk(16u * 1024 * 1024));
}
