// P2.11 NB7 (M9): GET /sd_card/tree is bounded in depth and entry count, and the query that sets the bounds is strict.
#include <cstring>
#include <initializer_list>

#include "../../../components/sd_card/include/tree_limits.hpp"
#include "../../../main/include/http_parse.hpp"
#include "unity.h"

namespace
{
using sd_tree::Budget;
using sd_tree::Limits;
using V = Budget::Verdict;

void test_defaults_are_valid_and_match_the_old_depth(void)
{
  Limits l;
  TEST_ASSERT_TRUE(l.valid());
  TEST_ASSERT_EQUAL_INT(5, l.depth);
  TEST_ASSERT_EQUAL_UINT32(256, l.limit);
  TEST_ASSERT_EQUAL_UINT32(0, l.offset);
}

void test_limits_validity(void)
{
  Limits l;
  l.depth = -1;
  TEST_ASSERT_FALSE(l.valid());
  l.depth = 6;
  TEST_ASSERT_FALSE(l.valid());
  l.depth = 0;
  TEST_ASSERT_TRUE(l.valid());
  l.limit = 0;
  TEST_ASSERT_FALSE(l.valid());
  l.limit = sd_tree::kMaxLimit + 1;
  TEST_ASSERT_FALSE(l.valid());
  l.limit = sd_tree::kMaxLimit;
  TEST_ASSERT_TRUE(l.valid());
}

void test_budget_takes_up_to_the_limit_then_stops(void)
{
  Limits l;
  l.limit = 3;
  Budget b(l);
  for (int i = 0; i < 3; i++)
  {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Take), static_cast<int>(b.next(0)));
  }
  TEST_ASSERT_FALSE(b.truncated());  // exactly full is not truncated until one more entry exists
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Stop), static_cast<int>(b.next(0)));
  TEST_ASSERT_TRUE(b.truncated());
  TEST_ASSERT_EQUAL_UINT32(3, b.taken());
}

void test_budget_stop_is_sticky_for_every_level(void)
{
  Limits l;
  l.limit = 1;
  Budget b(l);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Take), static_cast<int>(b.next(2)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Stop), static_cast<int>(b.next(2)));
  // parent directories keep asking while they unwind: they all get Stop and nothing is added
  for (int level : {0, 1, 2, 3})
  {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Stop), static_cast<int>(b.next(level)));
  }
  TEST_ASSERT_EQUAL_UINT32(1, b.taken());
}

void test_offset_skips_only_top_level_entries(void)
{
  Limits l;
  l.offset = 2;
  l.limit  = 10;
  Budget b(l);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Skip), static_cast<int>(b.next(0)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Skip), static_cast<int>(b.next(0)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Take), static_cast<int>(b.next(0)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Take), static_cast<int>(b.next(1)));  // nested entries are never skipped
  TEST_ASSERT_EQUAL_UINT32(2, b.taken());
  TEST_ASSERT_FALSE(b.truncated());
}

void test_offset_does_not_count_toward_the_limit(void)
{
  Limits l;
  l.offset = 5;
  l.limit  = 2;
  Budget b(l);
  for (int i = 0; i < 5; i++)
  {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Skip), static_cast<int>(b.next(0)));
  }
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Take), static_cast<int>(b.next(0)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Take), static_cast<int>(b.next(0)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Stop), static_cast<int>(b.next(0)));
}

void test_offset_past_the_end_is_an_empty_untruncated_listing(void)
{
  Limits l;
  l.offset = 100;
  Budget b(l);
  for (int i = 0; i < 3; i++)  // a directory with 3 entries
  {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(V::Skip), static_cast<int>(b.next(0)));
  }
  TEST_ASSERT_EQUAL_UINT32(0, b.taken());
  TEST_ASSERT_FALSE(b.truncated());
}

void test_expansion_follows_depth(void)
{
  Limits l;
  l.depth = 0;
  TEST_ASSERT_FALSE(Budget(l).mayExpand(0));  // depth 0: the requested directory only
  l.depth = 1;
  TEST_ASSERT_TRUE(Budget(l).mayExpand(0));
  TEST_ASSERT_FALSE(Budget(l).mayExpand(1));
  l.depth = 5;
  TEST_ASSERT_TRUE(Budget(l).mayExpand(4));
  TEST_ASSERT_FALSE(Budget(l).mayExpand(5));
}

void test_unlisted_directory_marks_truncated(void)
{
  Budget b{Limits{}};
  TEST_ASSERT_FALSE(b.truncated());
  b.markTruncated();
  TEST_ASSERT_TRUE(b.truncated());
}

// ---- query parsing ----
using R = http_parse::Result;

int parse(const char* uri, Limits* l)
{
  return static_cast<int>(http_parse::treeLimits(uri, l));
}

void test_tree_query_defaults_when_absent(void)
{
  Limits l;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(R::Ok), parse("/api/v1/sd_card/tree", &l));
  TEST_ASSERT_EQUAL_INT(5, l.depth);
  TEST_ASSERT_EQUAL_UINT32(256, l.limit);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(R::Ok), parse("/api/v1/sd_card/tree/trips?", &l));
}

void test_tree_query_parses_each_key(void)
{
  Limits l;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(R::Ok), parse("/t?depth=1&limit=50&offset=100", &l));
  TEST_ASSERT_EQUAL_INT(1, l.depth);
  TEST_ASSERT_EQUAL_UINT32(50, l.limit);
  TEST_ASSERT_EQUAL_UINT32(100, l.offset);
  Limits m;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(R::Ok), parse("/t?limit=512", &m));
  TEST_ASSERT_EQUAL_INT(5, m.depth);  // others keep the defaults
  TEST_ASSERT_EQUAL_UINT32(512, m.limit);
  Limits z;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(R::Ok), parse("/t?depth=0", &z));
  TEST_ASSERT_EQUAL_INT(0, z.depth);
  Limits y;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(R::Ok), parse("/t?limit=007", &y));  // leading zeros are still decimal digits
  TEST_ASSERT_EQUAL_UINT32(7, y.limit);
}

void test_tree_query_rejects_hostile_values(void)
{
  const char* const bad[] = {"/t?depth=6",       "/t?depth=-1",      "/t?limit=0",         "/t?limit=513",
                             "/t?limit=99999999999", "/t?offset=100001", "/t?depth=",        "/t?depth",
                             "/t?depth=x",       "/t?depth=1x",      "/t?depth=+1",        "/t?depth=%31",
                             "/t?depth= 1",      "/t?limit=1.5",     "/t?depth=1&depth=2", "/t?foo=1",
                             "/t?depth=1&foo=1", "/t?limit=0x10"};
  for (const char* u : bad)
  {
    Limits l;
    l.depth = 3;
    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(R::InvalidArg), parse(u, &l), u);
    TEST_ASSERT_EQUAL_INT_MESSAGE(3, l.depth, u);  // untouched on error
  }
  TEST_ASSERT_EQUAL_INT(static_cast<int>(R::InvalidArg), static_cast<int>(http_parse::treeLimits("/t", nullptr)));
}
}  // namespace

extern "C" void run_tree_limits_tests(void)
{
  RUN_TEST(test_defaults_are_valid_and_match_the_old_depth);
  RUN_TEST(test_limits_validity);
  RUN_TEST(test_budget_takes_up_to_the_limit_then_stops);
  RUN_TEST(test_budget_stop_is_sticky_for_every_level);
  RUN_TEST(test_offset_skips_only_top_level_entries);
  RUN_TEST(test_offset_does_not_count_toward_the_limit);
  RUN_TEST(test_offset_past_the_end_is_an_empty_untruncated_listing);
  RUN_TEST(test_expansion_follows_depth);
  RUN_TEST(test_unlisted_directory_marks_truncated);
  RUN_TEST(test_tree_query_defaults_when_absent);
  RUN_TEST(test_tree_query_parses_each_key);
  RUN_TEST(test_tree_query_rejects_hostile_values);
}
