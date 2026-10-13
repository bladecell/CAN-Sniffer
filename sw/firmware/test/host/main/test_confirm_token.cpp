// One-shot confirm token for destructive requests (F1): none, wrong, expired, reused, rotated.
#include <cstring>

#include "confirm_token.hpp"
#include "unity.h"

namespace
{
uint64_t gNow  = 0;
uint32_t gRand = 0;
uint64_t fakeNow() { return gNow; }
uint32_t fakeRand() { return ++gRand; }

struct Fixture
{
  ConfirmToken t{fakeNow, fakeRand, 10000};
  Fixture() { gNow = 1000; gRand = 0; }
};

void test_no_token_never_confirms(void)
{
  Fixture f;
  TEST_ASSERT_FALSE(f.t.consume(nullptr));
  TEST_ASSERT_FALSE(f.t.consume(""));
  char tok[ConfirmToken::kTokenSize];
  f.t.issue(tok);
  TEST_ASSERT_FALSE(f.t.consume(nullptr));
  TEST_ASSERT_FALSE(f.t.consume(""));
}

void test_valid_token_confirms_once(void)
{
  Fixture f;
  char tok[ConfirmToken::kTokenSize];
  f.t.issue(tok);
  TEST_ASSERT_EQUAL_size_t(16, strlen(tok));
  TEST_ASSERT_TRUE(f.t.consume(tok));
  TEST_ASSERT_FALSE(f.t.consume(tok));
}

void test_wrong_token_rejected_and_does_not_burn_the_real_one(void)
{
  Fixture f;
  char tok[ConfirmToken::kTokenSize];
  f.t.issue(tok);
  TEST_ASSERT_FALSE(f.t.consume("0000000000000000"));
  TEST_ASSERT_FALSE(f.t.consume("zz"));
  TEST_ASSERT_FALSE(f.t.consume("this-token-is-far-longer-than-any-issued-one-0123456789"));
  TEST_ASSERT_TRUE(f.t.consume(tok));
}

void test_prefix_of_token_rejected(void)
{
  Fixture f;
  char tok[ConfirmToken::kTokenSize];
  f.t.issue(tok);
  tok[8] = '\0';
  TEST_ASSERT_FALSE(f.t.consume(tok));
}

void test_expiry_boundary(void)
{
  Fixture f;
  char tok[ConfirmToken::kTokenSize];
  f.t.issue(tok);
  gNow += 9999;
  TEST_ASSERT_TRUE(f.t.consume(tok));
  f.t.issue(tok);
  gNow += 10000;
  TEST_ASSERT_FALSE(f.t.consume(tok));
}

void test_issue_rotates_and_invalidates_previous(void)
{
  Fixture f;
  char a[ConfirmToken::kTokenSize];
  char b[ConfirmToken::kTokenSize];
  f.t.issue(a);
  f.t.issue(b);
  TEST_ASSERT_NOT_EQUAL(0, strcmp(a, b));
  TEST_ASSERT_FALSE(f.t.consume(a));
  TEST_ASSERT_TRUE(f.t.consume(b));
}

void test_no_token_call_reuses_pending_token_and_expiry(void)
{
  Fixture f;
  char a[ConfirmToken::kTokenSize];
  char b[ConfirmToken::kTokenSize];
  TEST_ASSERT_EQUAL_UINT32(10000, f.t.challenge(nullptr, a));
  gNow += 4000;
  TEST_ASSERT_EQUAL_UINT32(6000, f.t.challenge(nullptr, b));
  TEST_ASSERT_EQUAL_STRING(a, b);
  TEST_ASSERT_EQUAL_UINT32(6000, f.t.challenge("", b));
  TEST_ASSERT_EQUAL_STRING(a, b);
  TEST_ASSERT_TRUE(f.t.consume(a));
}

void test_wrong_token_challenge_rotates(void)
{
  Fixture f;
  char a[ConfirmToken::kTokenSize];
  char b[ConfirmToken::kTokenSize];
  f.t.challenge(nullptr, a);
  TEST_ASSERT_EQUAL_UINT32(10000, f.t.challenge("0000000000000000", b));
  TEST_ASSERT_NOT_EQUAL(0, strcmp(a, b));
  TEST_ASSERT_FALSE(f.t.consume(a));
  TEST_ASSERT_TRUE(f.t.consume(b));
}

void test_expired_or_used_token_challenge_rotates(void)
{
  Fixture f;
  char a[ConfirmToken::kTokenSize];
  char b[ConfirmToken::kTokenSize];
  f.t.challenge(nullptr, a);
  gNow += 10000;
  TEST_ASSERT_EQUAL_UINT32(10000, f.t.challenge(nullptr, b));
  TEST_ASSERT_NOT_EQUAL(0, strcmp(a, b));
  TEST_ASSERT_TRUE(f.t.consume(b));
  char c[ConfirmToken::kTokenSize];
  TEST_ASSERT_EQUAL_UINT32(10000, f.t.challenge(nullptr, c));
  TEST_ASSERT_NOT_EQUAL(0, strcmp(b, c));
}

void test_nothing_issued_nothing_confirms(void)
{
  Fixture f;
  TEST_ASSERT_FALSE(f.t.consume("0000000000000000"));
}
} // namespace

extern "C" void run_confirm_token_tests(void)
{
  RUN_TEST(test_no_token_never_confirms);
  RUN_TEST(test_valid_token_confirms_once);
  RUN_TEST(test_wrong_token_rejected_and_does_not_burn_the_real_one);
  RUN_TEST(test_prefix_of_token_rejected);
  RUN_TEST(test_expiry_boundary);
  RUN_TEST(test_issue_rotates_and_invalidates_previous);
  RUN_TEST(test_no_token_call_reuses_pending_token_and_expiry);
  RUN_TEST(test_wrong_token_challenge_rotates);
  RUN_TEST(test_expired_or_used_token_challenge_rotates);
  RUN_TEST(test_nothing_issued_nothing_confirms);
}
