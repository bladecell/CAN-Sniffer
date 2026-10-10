// Formula evaluation through PIDDefinition (TinyExpr + getBit/bitMask/getPID/getPIDRaw), OBD-4.
#include <cmath>
#include <cstring>

#include "pid_def.hpp"
#include "unity.h"

// Link-time stand-ins for the data-model lookups that live in obd2.cpp.
double get_pid_value(double id)
{
    return id == 12 ? 1500.0 : 0.0;
}

double get_pid_raw(double id, double byteIdx)
{
    return (id == 12 && byteIdx == 1) ? 7.0 : 0.0;
}

static PIDDefinition makePid(const char* formula)
{
    return PIDDefinition(1, 0x01, 0x0C, 2, "t", "u", "d", formula, 0.f, 100.f, 1, 100, 0, "i");
}

static float evalOk(const char* formula, const uint8_t* data, uint8_t len)
{
    PIDDefinition pid = makePid(formula);
    TEST_ASSERT_TRUE(pid.formulaValid());
    float out = -1.f;
    TEST_ASSERT_EQUAL_INT(ESP_OK, pid.evaluate(data, len, out));
    return out;
}

static void test_formula_uses_frame_bytes(void)
{
    const uint8_t d[] = {0x1A, 0xF8};
    TEST_ASSERT_EQUAL_FLOAT(1726.f, evalOk("(A*256+B)/4", d, 2));
}

static void test_missing_bytes_read_as_zero(void)
{
    const uint8_t d[] = {5};
    TEST_ASSERT_EQUAL_FLOAT(5.f, evalOk("A+B+C+D", d, 1));
}

static void test_getbit_and_bitmask(void)
{
    const uint8_t d[] = {0b00001000, 0b10110000};
    TEST_ASSERT_EQUAL_FLOAT(1.f, evalOk("getBit(A,3)", d, 2));
    TEST_ASSERT_EQUAL_FLOAT(0.f, evalOk("getBit(A,2)", d, 2));
    TEST_ASSERT_EQUAL_FLOAT(0.f, evalOk("getBit(A,40)", d, 2));
    TEST_ASSERT_EQUAL_FLOAT(11.f, evalOk("bitMask(B,4,4)", d, 2));
    TEST_ASSERT_EQUAL_FLOAT(0.f, evalOk("bitMask(B,40,4)", d, 2));
}

static void test_getpid_and_getpidraw(void)
{
    TEST_ASSERT_EQUAL_FLOAT(1501.f, evalOk("getPID(12)+1", nullptr, 0));
    TEST_ASSERT_EQUAL_FLOAT(7.f, evalOk("getPIDRaw(12,1)", nullptr, 0));
}

static void test_bit_helpers_survive_nan_negative_and_huge_inputs(void)
{
    const uint8_t d[] = {0xFF};
    evalOk("getBit(0/0,1)", d, 1);       // NaN input must not crash; result is not asserted
    evalOk("getBit(0-5,1)", d, 1);       // negative
    evalOk("getBit(1e300,1)", d, 1);     // out of uint32 range
    evalOk("bitMask(A,0,32)", d, 1);     // shift by the full register width
    evalOk("bitMask(A,0,1e300)", d, 1);
}

static void test_invalid_formula_is_rejected_not_fatal(void)
{
    PIDDefinition pid = makePid("2+*");
    TEST_ASSERT_FALSE(pid.formulaValid());
    float out = 123.f;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, pid.evaluate(nullptr, 0, out));
    TEST_ASSERT_EQUAL_FLOAT(123.f, out);
}

static void test_unknown_identifier_and_empty_formula_are_rejected(void)
{
    TEST_ASSERT_FALSE(makePid("Z+1").formulaValid());
    TEST_ASSERT_FALSE(makePid("").formulaValid());
    TEST_ASSERT_FALSE(makePid("A+(").formulaValid());
}

static void expectInvalid(const char* formula)
{
    PIDDefinition pid = makePid(formula);
    float         out = 123.f;
    const uint8_t d[] = {1, 2};
    TEST_ASSERT_NOT_EQUAL_MESSAGE(ESP_OK, pid.evaluate(d, 2, out), formula);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(123.f, out, formula);  // a rejected value must not be written
}

static void test_division_by_zero_marks_invalid(void)
{
    expectInvalid("1/0");
    expectInvalid("A/(B-2)");
}

static void test_nan_marks_invalid(void)
{
    expectInvalid("0/0");
    expectInvalid("sqrt(0-1)");
}

static void test_float_overflow_marks_invalid(void)
{
    expectInvalid("1e300");  // finite as double, inf as float
}

extern "C" void run_pid_def_tests(void)
{
    RUN_TEST(test_formula_uses_frame_bytes);
    RUN_TEST(test_missing_bytes_read_as_zero);
    RUN_TEST(test_getbit_and_bitmask);
    RUN_TEST(test_getpid_and_getpidraw);
    RUN_TEST(test_bit_helpers_survive_nan_negative_and_huge_inputs);
    RUN_TEST(test_invalid_formula_is_rejected_not_fatal);
    RUN_TEST(test_unknown_identifier_and_empty_formula_are_rejected);
    RUN_TEST(test_division_by_zero_marks_invalid);
    RUN_TEST(test_nan_marks_invalid);
    RUN_TEST(test_float_overflow_marks_invalid);
}
