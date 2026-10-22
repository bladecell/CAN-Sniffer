#include <cstring>

#include "../../../main/include/fixed_text.hpp"
#include "unity.h"

namespace
{
void test_starts_empty()
{
  FixedText<8> t;
  TEST_ASSERT_TRUE(t.empty());
  TEST_ASSERT_EQUAL_STRING("", t.c_str());
}

void test_set_and_append_build_text()
{
  FixedText<32> t;
  t.set("a");
  t.append("-b").append("-c");
  TEST_ASSERT_FALSE(t.empty());
  TEST_ASSERT_EQUAL_STRING("a-b-c", t.c_str());
}

void test_overflow_truncates_and_stays_terminated()
{
  FixedText<8> t;
  t.set("0123456789");
  TEST_ASSERT_EQUAL_STRING("0123456", t.c_str());
  t.append("xyz");
  TEST_ASSERT_EQUAL_STRING("0123456", t.c_str());
  t.set("ab");
  t.append("cdefghijk");
  TEST_ASSERT_EQUAL_STRING("abcdefg", t.c_str());
}

void test_null_input_is_ignored()
{
  FixedText<8> t;
  t.set("ab");
  t.append(nullptr);
  TEST_ASSERT_EQUAL_STRING("ab", t.c_str());
  t.set(nullptr);
  TEST_ASSERT_TRUE(t.empty());
}
}  // namespace

extern "C" void run_fixed_text_tests(void)
{
  RUN_TEST(test_starts_empty);
  RUN_TEST(test_set_and_append_build_text);
  RUN_TEST(test_overflow_truncates_and_stays_terminated);
  RUN_TEST(test_null_input_is_ignored);
}
