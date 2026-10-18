// Response decisions behind send_json_response, the PUT created/overwritten status and the PATCH /settings reason.
#include "../../../main/include/response_rules.hpp"
#include "../../../main/include/settings_patch.hpp"
#include "unity.h"

using response_rules::Kind;

static void test_empty_204_is_sent_without_a_body(void)
{
  TEST_ASSERT_EQUAL(Kind::NoContent, response_rules::kindFor(false, 204));
}

static void test_a_null_body_with_another_status_is_an_error_response(void)
{
  TEST_ASSERT_EQUAL(Kind::Error, response_rules::kindFor(false, 200));
  TEST_ASSERT_EQUAL(Kind::Error, response_rules::kindFor(false, 500));
  TEST_ASSERT_EQUAL(Kind::Error, response_rules::kindFor(false, 404));
}

static void test_a_body_is_always_json(void)
{
  TEST_ASSERT_EQUAL(Kind::Json, response_rules::kindFor(true, 200));
  TEST_ASSERT_EQUAL(Kind::Json, response_rules::kindFor(true, 201));
  TEST_ASSERT_EQUAL(Kind::Json, response_rules::kindFor(true, 204));
  TEST_ASSERT_EQUAL(Kind::Json, response_rules::kindFor(true, 503));
}

static void test_status_line_is_only_set_for_non_200(void)
{
  TEST_ASSERT_FALSE(response_rules::needsStatusLine(200));
  TEST_ASSERT_TRUE(response_rules::needsStatusLine(201));
  TEST_ASSERT_TRUE(response_rules::needsStatusLine(202));
  TEST_ASSERT_TRUE(response_rules::needsStatusLine(400));
}

static void test_retry_after_only_on_429_and_503(void)
{
  TEST_ASSERT_EQUAL_UINT(0, response_rules::retryAfter(200, 0));
  TEST_ASSERT_EQUAL_UINT(0, response_rules::retryAfter(404, 9));
  TEST_ASSERT_EQUAL_UINT(1, response_rules::retryAfter(503, 0));
  TEST_ASSERT_EQUAL_UINT(5, response_rules::retryAfter(503, 5));
  TEST_ASSERT_EQUAL_UINT(30, response_rules::retryAfter(429, 30));
  TEST_ASSERT_EQUAL_UINT(1, response_rules::retryAfter(429, 0));
}

static void test_put_status_is_201_for_created_and_200_for_overwritten(void)
{
  TEST_ASSERT_EQUAL(201, response_rules::putStatus(true));
  TEST_ASSERT_EQUAL(200, response_rules::putStatus(false));
}

static void test_settings_reason_repeats_the_first_error(void)
{
  TEST_ASSERT_EQUAL_STRING("bad bitrate", settings_patch::topReason("bad bitrate"));
  TEST_ASSERT_EQUAL_STRING("settings not saved", settings_patch::topReason(nullptr));
  TEST_ASSERT_EQUAL_STRING("settings not saved", settings_patch::topReason(""));
}

extern "C" void run_response_rules_tests(void)
{
  RUN_TEST(test_empty_204_is_sent_without_a_body);
  RUN_TEST(test_a_null_body_with_another_status_is_an_error_response);
  RUN_TEST(test_a_body_is_always_json);
  RUN_TEST(test_status_line_is_only_set_for_non_200);
  RUN_TEST(test_retry_after_only_on_429_and_503);
  RUN_TEST(test_put_status_is_201_for_created_and_200_for_overwritten);
  RUN_TEST(test_settings_reason_repeats_the_first_error);
}
