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

static void test_power_only_system_patch_needs_no_restart(void)
{
  // system.sleep_enabled and the two voltage thresholds are applied live (Core::setPolicy); nothing else in the system
  // section is, and wifi and can only after a reboot.
  TEST_ASSERT_FALSE(systemKeyNeedsRestart("sleep_enabled"));
  TEST_ASSERT_FALSE(systemKeyNeedsRestart("stay_awake_voltage_mv"));
  TEST_ASSERT_FALSE(systemKeyNeedsRestart("low_battery_voltage_mv"));
  TEST_ASSERT_TRUE(systemKeyNeedsRestart("pid_def_path"));
  TEST_ASSERT_TRUE(systemKeyNeedsRestart("dtc_desc_path"));
  TEST_ASSERT_FALSE(systemKeyNeedsRestart("something_unknown"));  // ignored by the parser: nothing changed
  TEST_ASSERT_FALSE(systemKeyNeedsRestart(""));

  const Outcome saved[]         = {Outcome::Saved};
  const bool    powerOnly[]     = {false};
  const bool    needsRestart[]  = {true};
  TEST_ASSERT_FALSE(restartRequired(saved, powerOnly, 1));
  TEST_ASSERT_TRUE(restartRequired(saved, needsRestart, 1));
  const Outcome two[]    = {Outcome::Saved, Outcome::Saved};
  const bool    mixed[]  = {true, false};  // wifi + power
  TEST_ASSERT_TRUE(restartRequired(two, mixed, 2));
  const bool    both[]   = {false, false};
  TEST_ASSERT_FALSE(restartRequired(two, both, 2));
  const Outcome failed[] = {Outcome::Saved, Outcome::Invalid};  // the unsaved section needs nothing
  const bool    f[]      = {false, true};
  TEST_ASSERT_FALSE(restartRequired(failed, f, 2));
  const Outcome none[] = {Outcome::Invalid};
  const bool    n[]    = {true};
  TEST_ASSERT_FALSE(restartRequired(none, n, 1));
  TEST_ASSERT_FALSE(restartRequired(nullptr, nullptr, 0));
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

static void test_duplicate_section_names_are_found(void)
{
  using namespace settings_patch;
  const std::string_view ok[]  = {"wifi", "can", "system"};
  TEST_ASSERT_EQUAL(3, firstDuplicate(ok, 3));
  const std::string_view dup[] = {"wifi", "can", "wifi"};
  TEST_ASSERT_EQUAL(2, firstDuplicate(dup, 3));
  const std::string_view unk[] = {"x", "x"};
  TEST_ASSERT_EQUAL(1, firstDuplicate(unk, 2));
  const std::string_view one[] = {"can"};
  TEST_ASSERT_EQUAL(1, firstDuplicate(one, 1));
  TEST_ASSERT_EQUAL(0, firstDuplicate(one, 0));
  const std::string_view cased[] = {"wifi", "WIFI"};
  TEST_ASSERT_EQUAL(2, firstDuplicate(cased, 2));  // names are case sensitive, so these are different (and the second is unknown)
}

extern "C" void run_settings_patch_tests(void)
{
  RUN_TEST(test_duplicate_section_names_are_found);
  RUN_TEST(test_section_names);
  RUN_TEST(test_status_all_saved_needs_restart);
  RUN_TEST(test_status_error_precedence);
  RUN_TEST(test_power_only_system_patch_needs_no_restart);
  RUN_TEST(test_status_empty_is_bad_request);
  RUN_TEST(test_section_count_is_capped);
}
