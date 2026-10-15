// Range-checked JSON numbers for the REST add-PID path (B40): nothing wraps, nothing casts out of range.
#include <cmath>
#include <limits>

#include "cJSON.h"
#include "pid_json.hpp"
#include "unity.h"

using namespace pidjson;

namespace
{
bool intOf(const char* text, uint32_t lo, uint32_t hi, uint32_t* out = nullptr)
{
    cJSON*   v = cJSON_Parse(text);
    uint32_t r = 0xDEADBEEF;
    bool     ok = v != nullptr && integerFromJson(v, lo, hi, r);
    if (!ok)
    {
        TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, r);  // output untouched on failure
    }
    if (out != nullptr)
    {
        *out = r;
    }
    cJSON_Delete(v);
    return ok;
}

bool intOfNumber(double d, uint32_t lo, uint32_t hi)
{
    cJSON*   v = cJSON_CreateNumber(d);
    uint32_t r = 0;
    bool     ok = integerFromJson(v, lo, hi, r);
    cJSON_Delete(v);
    return ok;
}
}  // namespace

static void test_integer_rejects_wrapping_and_hostile_values(void)
{
    // mode: uint8
    TEST_ASSERT_FALSE(intOf("257", 0, 255));   // would wrap to 1 and pass the mode rule
    TEST_ASSERT_FALSE(intOf("256", 0, 255));
    TEST_ASSERT_FALSE(intOf("-1", 0, 255));
    TEST_ASSERT_FALSE(intOf("0.5", 0, 255));
    TEST_ASSERT_FALSE(intOf("1.0000001", 0, 255));
    TEST_ASSERT_FALSE(intOf("1e10", 0, 255));
    TEST_ASSERT_FALSE(intOf("1e300", 0, 255));
    TEST_ASSERT_FALSE(intOf("-1e300", 0, 255));
    TEST_ASSERT_FALSE(intOf("\"1\"", 0, 255));  // a string is not a number
    TEST_ASSERT_FALSE(intOf("null", 0, 255));
    TEST_ASSERT_FALSE(intOf("true", 0, 255));
    TEST_ASSERT_FALSE(intOf("[1]", 0, 255));
    TEST_ASSERT_FALSE(intOfNumber(std::nan(""), 0, 255));
    TEST_ASSERT_FALSE(intOfNumber(INFINITY, 0, 255));
    TEST_ASSERT_FALSE(intOfNumber(-INFINITY, 0, 255));
    // pid: uint16
    TEST_ASSERT_FALSE(intOf("65536", 0, 65535));
    TEST_ASSERT_FALSE(intOf("-1", 0, 65535));
    // id: 0..0x7FF
    TEST_ASSERT_FALSE(intOf("2048", 0, 0x7FF));
    TEST_ASSERT_FALSE(intOf("4294967296", 0, 0x7FF));
    // interval: uint16
    TEST_ASSERT_FALSE(intOf("65536", 0, 65535));
    // priority: uint8
    TEST_ASSERT_FALSE(intOf("256", 0, 255));
    // color: 0..0xFFFFFF
    TEST_ASSERT_FALSE(intOf("16777216", 0, 0xFFFFFF));
    TEST_ASSERT_FALSE(intOf("-5", 0, 0xFFFFFF));
    TEST_ASSERT_FALSE(intOf("4294967296", 0, 0xFFFFFFFFu));  // above the widest range
}

static void test_integer_accepts_the_real_boundaries(void)
{
    uint32_t v = 99;
    TEST_ASSERT_TRUE(intOf("0", 0, 255, &v));
    TEST_ASSERT_EQUAL_UINT32(0, v);
    TEST_ASSERT_TRUE(intOf("255", 0, 255, &v));
    TEST_ASSERT_EQUAL_UINT32(255, v);
    TEST_ASSERT_TRUE(intOf("1", 0, 255, &v));
    TEST_ASSERT_EQUAL_UINT32(1, v);
    TEST_ASSERT_TRUE(intOf("65535", 0, 65535, &v));
    TEST_ASSERT_EQUAL_UINT32(65535, v);
    TEST_ASSERT_TRUE(intOf("2047", 0, 0x7FF, &v));
    TEST_ASSERT_EQUAL_UINT32(2015, 2015);
    TEST_ASSERT_TRUE(intOf("16777215", 0, 0xFFFFFF, &v));
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFF, v);
    TEST_ASSERT_TRUE(intOf("4294967295", 0, 0xFFFFFFFFu, &v));
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, v);
    TEST_ASSERT_TRUE(intOf("12.0", 0, 255, &v));  // an integral value written with a fraction part is fine
    TEST_ASSERT_EQUAL_UINT32(12, v);
    TEST_ASSERT_TRUE(intOf("-0", 0, 255, &v));
    TEST_ASSERT_EQUAL_UINT32(0, v);
}

static void test_float_rejects_what_a_float_cannot_hold(void)
{
    float f = 7.f;
    cJSON* v = cJSON_Parse("1e300");
    TEST_ASSERT_FALSE(floatFromJson(v, f));
    cJSON_Delete(v);
    v = cJSON_Parse("-1e300");
    TEST_ASSERT_FALSE(floatFromJson(v, f));
    cJSON_Delete(v);
    v = cJSON_Parse("\"1\"");
    TEST_ASSERT_FALSE(floatFromJson(v, f));
    cJSON_Delete(v);
    v = cJSON_Parse("null");
    TEST_ASSERT_FALSE(floatFromJson(v, f));
    cJSON_Delete(v);
    v = cJSON_CreateNumber(std::nan(""));
    TEST_ASSERT_FALSE(floatFromJson(v, f));
    cJSON_Delete(v);
    v = cJSON_CreateNumber(INFINITY);
    TEST_ASSERT_FALSE(floatFromJson(v, f));
    cJSON_Delete(v);
    TEST_ASSERT_EQUAL_FLOAT(7.f, f);  // untouched on every failure

    v = cJSON_Parse("-40.5");
    TEST_ASSERT_TRUE(floatFromJson(v, f));
    TEST_ASSERT_EQUAL_FLOAT(-40.5f, f);
    cJSON_Delete(v);
    v = cJSON_CreateNumber(static_cast<double>(std::numeric_limits<float>::max()));
    TEST_ASSERT_TRUE(floatFromJson(v, f));
    TEST_ASSERT_EQUAL_FLOAT(std::numeric_limits<float>::max(), f);
    cJSON_Delete(v);
}

extern "C" void run_pid_json_numbers_tests(void)
{
    RUN_TEST(test_integer_rejects_wrapping_and_hostile_values);
    RUN_TEST(test_integer_accepts_the_real_boundaries);
    RUN_TEST(test_float_rejects_what_a_float_cannot_hold);
}
