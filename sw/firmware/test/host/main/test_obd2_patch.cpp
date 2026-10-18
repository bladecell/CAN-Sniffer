#include "../../../main/include/obd2_patch.hpp"
#include "cJSON.h"
#include "unity.h"

static bool parse(const char* text, bool& on)
{
  cJSON*     j  = cJSON_Parse(text);
  const bool ok = obd2_patch::parseAutoPoll(j, on);
  cJSON_Delete(j);
  return ok;
}

static void test_auto_poll_accepts_only_one_bool(void)
{
  bool on = false;
  TEST_ASSERT_TRUE(parse("{\"auto_poll\":true}", on));
  TEST_ASSERT_TRUE(on);
  TEST_ASSERT_TRUE(parse("{\"auto_poll\":false}", on));
  TEST_ASSERT_FALSE(on);
}

static void test_auto_poll_rejects_everything_else(void)
{
  bool on = true;
  TEST_ASSERT_FALSE(parse("{}", on));
  TEST_ASSERT_FALSE(parse("[]", on));
  TEST_ASSERT_FALSE(parse("true", on));
  TEST_ASSERT_FALSE(parse("{\"auto_poll\":1}", on));
  TEST_ASSERT_FALSE(parse("{\"auto_poll\":\"true\"}", on));
  TEST_ASSERT_FALSE(parse("{\"auto_poll\":null}", on));
  TEST_ASSERT_FALSE(parse("{\"auto_poll\":true,\"x\":1}", on));
  TEST_ASSERT_FALSE(parse("{\"x\":true}", on));
  TEST_ASSERT_FALSE(parse("{\"auto_poll\":true,\"auto_poll\":false}", on));
  TEST_ASSERT_FALSE(obd2_patch::parseAutoPoll(nullptr, on));
}

extern "C" void run_obd2_patch_tests(void)
{
  RUN_TEST(test_auto_poll_accepts_only_one_bool);
  RUN_TEST(test_auto_poll_rejects_everything_else);
}
