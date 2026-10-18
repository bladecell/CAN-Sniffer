// JSON pre-scan (B56 / HEAP-1): rejects inputs whose cJSON tree would cost far more heap than their byte size suggests.
#include <string>

#include "json_budget.hpp"
#include "unity.h"

using json_budget::Verdict;

static constexpr size_t kPlentyOfHeap = 8u * 1024 * 1024;

static Verdict check(const std::string& s, size_t freeHeap = kPlentyOfHeap)
{
    return json_budget::check(s.data(), s.size(), freeHeap);
}

static void test_a_normal_pid_array_passes_minified_and_pretty(void)
{
    const std::string minified =
        R"([{"id":2015,"mode":1,"pid":12,"length":2,"name":"RPM","unit":"rpm","formula":"(A*256+B)/4","interval":100},)"
        R"({"id":2015,"mode":1,"pid":13,"length":1,"name":"Speed","unit":"km/h","formula":"A","interval":100}])";
    TEST_ASSERT_EQUAL(Verdict::Ok, check(minified));
    const std::string pretty = "[\n  {\n    \"id\": 2015,\n    \"mode\": 1,\n    \"pid\": 4,\n    \"name\": \"Load\"\n  }\n]\n";
    TEST_ASSERT_EQUAL(Verdict::Ok, check(pretty));
}

static void test_64_kib_of_empty_arrays_is_rejected_before_parsing(void)
{
    std::string s = "[";
    while (s.size() < 64 * 1024) { s += "[],"; }
    s += "[]]";
    TEST_ASSERT_EQUAL(Verdict::TooManyNodes, check(s));
}

static void test_many_scalars_and_empty_objects_are_rejected(void)
{
    std::string a = "[";
    while (a.size() < 64 * 1024) { a += "0,"; }
    a += "0]";
    TEST_ASSERT_EQUAL(Verdict::TooManyNodes, check(a));
    std::string o = "[";
    while (o.size() < 64 * 1024) { o += "{},"; }
    o += "{}]";
    TEST_ASSERT_EQUAL(Verdict::TooManyNodes, check(o));
}

static void test_absolute_node_cap_holds_even_at_low_density(void)
{
    std::string s = "[";
    for (size_t i = 0; i < json_budget::MAX_NODES + 10; ++i) { s += "\"a long enough string to keep density low, 0123456789\","; }
    s += "0]";
    TEST_ASSERT_EQUAL(Verdict::TooManyNodes, check(s));
}

static void test_deep_nesting_is_rejected_brackets_inside_strings_do_not_coun(void)
{
    TEST_ASSERT_EQUAL(Verdict::TooDeep, check(std::string(100, '[')));
    TEST_ASSERT_EQUAL(Verdict::TooDeep, check(std::string(json_budget::MAX_DEPTH + 1, '[') + std::string(json_budget::MAX_DEPTH + 1, ']')));
    TEST_ASSERT_EQUAL(Verdict::Ok, check(std::string(json_budget::MAX_DEPTH, '[') + std::string(json_budget::MAX_DEPTH, ']')));
    TEST_ASSERT_EQUAL(Verdict::Ok, check(R"(["[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[["])"));
    TEST_ASSERT_EQUAL(Verdict::Ok, check(R"(["escaped \" quote then [[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[["])"));
}

static void test_object_keys_are_not_nodes(void)
{
    TEST_ASSERT_EQUAL(Verdict::Ok, check(R"({"a":1,"b":2,"c":3})"));
}

static void test_truncated_and_garbage_input_does_not_crash_the_scan(void)
{
    TEST_ASSERT_EQUAL(Verdict::Ok, check(""));
    check("[\"unterminated");
    check("[\"trailing backslash\\");
    check("]]]]]]]]");
    check(std::string("\0\0\xff[", 4));
    TEST_ASSERT_EQUAL(Verdict::Ok, json_budget::check(nullptr, 0, kPlentyOfHeap));
}

static void test_heap_check_refuses_a_parse_that_would_not_fit(void)
{
    const std::string s = R"([{"id":1,"pid":2,"name":"x"}])";
    const size_t      need = json_budget::estimateBytes(5, s.size());
    TEST_ASSERT_EQUAL(Verdict::NoMemory, check(s, 1024));
    TEST_ASSERT_EQUAL(Verdict::NoMemory, check(s, need + json_budget::HEAP_RESERVE - 1));
    TEST_ASSERT_EQUAL(Verdict::Ok, check(s, need + json_budget::HEAP_RESERVE + 1024));
}

static void test_estimate_saturates_instead_of_wrapping(void)
{
    TEST_ASSERT_TRUE(json_budget::estimateBytes(static_cast<size_t>(-1), static_cast<size_t>(-1)) > 1u << 30);
}

extern "C" void run_json_budget_tests(void)
{
  RUN_TEST(test_a_normal_pid_array_passes_minified_and_pretty);
  RUN_TEST(test_64_kib_of_empty_arrays_is_rejected_before_parsing);
  RUN_TEST(test_many_scalars_and_empty_objects_are_rejected);
  RUN_TEST(test_absolute_node_cap_holds_even_at_low_density);
  RUN_TEST(test_deep_nesting_is_rejected_brackets_inside_strings_do_not_coun);
  RUN_TEST(test_object_keys_are_not_nodes);
  RUN_TEST(test_truncated_and_garbage_input_does_not_crash_the_scan);
  RUN_TEST(test_heap_check_refuses_a_parse_that_would_not_fit);
  RUN_TEST(test_estimate_saturates_instead_of_wrapping);
}
