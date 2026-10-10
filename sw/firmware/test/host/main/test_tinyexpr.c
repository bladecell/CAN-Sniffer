#include <math.h>
#include "tinyexpr.h"
#include "unity.h"

static double evalOk(const char *expr)
{
    int err = -1;
    double v = te_interp(expr, &err);
    TEST_ASSERT_EQUAL_INT(0, err);
    return v;
}

static void test_arithmetic(void)
{
    TEST_ASSERT_EQUAL_DOUBLE(14.0, evalOk("2+3*4"));
    TEST_ASSERT_EQUAL_DOUBLE(20.0, evalOk("(2+3)*4"));
}

static void test_syntax_error_reports_position(void)
{
    int err = 0;
    double v = te_interp("2+*3", &err);
    TEST_ASSERT_NOT_EQUAL(0, err);
    TEST_ASSERT_TRUE(isnan(v));
}

static void test_high_bit_chars_do_not_crash(void)
{
    int err = 0;
    double v = te_interp("\xff\xfe+1", &err);
    TEST_ASSERT_NOT_EQUAL(0, err);
    TEST_ASSERT_TRUE(isnan(v));
}

void run_tinyexpr_tests(void)
{
    RUN_TEST(test_arithmetic);
    RUN_TEST(test_syntax_error_reports_position);
    RUN_TEST(test_high_bit_chars_do_not_crash);
}
