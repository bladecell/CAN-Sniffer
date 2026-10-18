#include "../../../main/include/settings_patch.hpp"
#include "unity.h"

using namespace settings_patch;

static void test_section_names(void)
{
  TEST_ASSERT_TRUE(sectionOf("wifi") == Section::Wifi);
  TEST_ASSERT_TRUE(sectionOf("can") == Section::Can);
  TEST_ASSERT_TRUE(sectionOf("system") == Section::System);
  TEST_ASSERT_TRUE(sectionOf("WIFI") == Section::Unknown);
  TEST_ASSERT_TRUE(sectionOf("") == Section::Unknown);
  TEST_ASSERT_TRUE(sectionOf("name") == Section::Unknown);
}

static void test_status_all_saved_needs_restart(void)
{
  const Outcome o[] = {Outcome::Saved, Outcome::Saved};
  TEST_ASSERT_EQUAL_INT(200, httpStatus(o, 2));
  TEST_ASSERT_TRUE(restartRequired(o, 2));
}

static void test_status_error_precedence(void)
{
  const Outcome a[] = {Outcome::Saved, Outcome::Invalid};
  TEST_ASSERT_EQUAL_INT(400, httpStatus(a, 2));
  TEST_ASSERT_TRUE(restartRequired(a, 2));  // the valid section was still saved
  const Outcome b[] = {Outcome::Unknown, Outcome::Invalid};
  TEST_ASSERT_EQUAL_INT(400, httpStatus(b, 2));
  const Outcome c[] = {Outcome::Unknown};
  TEST_ASSERT_EQUAL_INT(422, httpStatus(c, 1));
  TEST_ASSERT_FALSE(restartRequired(c, 1));
  const Outcome d[] = {Outcome::Invalid, Outcome::Failed, Outcome::Unknown};
  TEST_ASSERT_EQUAL_INT(500, httpStatus(d, 3));
}

static void test_status_empty_is_bad_request(void)
{
  TEST_ASSERT_EQUAL_INT(400, httpStatus(nullptr, 0));
  TEST_ASSERT_FALSE(restartRequired(nullptr, 0));
}

static void test_section_count_is_capped(void)
{
  TEST_ASSERT_TRUE(sectionCountOk(1));
  TEST_ASSERT_TRUE(sectionCountOk(kMaxSections));
  TEST_ASSERT_FALSE(sectionCountOk(0));
  TEST_ASSERT_FALSE(sectionCountOk(kMaxSections + 1));
}

extern "C" void run_settings_patch_tests(void)
{
  RUN_TEST(test_section_names);
  RUN_TEST(test_status_all_saved_needs_restart);
  RUN_TEST(test_status_error_precedence);
  RUN_TEST(test_status_empty_is_bad_request);
  RUN_TEST(test_section_count_is_capped);
}
