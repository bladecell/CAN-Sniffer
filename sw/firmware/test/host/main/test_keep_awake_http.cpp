// POST /system/keep_awake body: {"enabled": bool} and nothing else.
#include "../../../main/include/keep_awake_http.hpp"
#include "cJSON.h"
#include "unity.h"

static bool parse(const char* text, bool& on)
{
  cJSON*     j  = cJSON_Parse(text);
  const bool ok = keep_awake_http::parseBody(j, on);
  cJSON_Delete(j);
  return ok;
}

static void test_keep_awake_body_accepts_one_bool(void)
{
  bool on = false;
  TEST_ASSERT_TRUE(parse("{\"enabled\":true}", on));
  TEST_ASSERT_TRUE(on);
  TEST_ASSERT_TRUE(parse("{\"enabled\":false}", on));
  TEST_ASSERT_FALSE(on);
}

static void test_keep_awake_body_rejects_everything_else(void)
{
  bool on = true;
  const char* bad[] = {"{}", "[]", "null", "true", "{\"enabled\":1}", "{\"enabled\":\"true\"}", "{\"enabled\":null}",
                       "{\"enabled\":true,\"x\":1}", "{\"x\":true}", "{\"enabled\":true,\"enabled\":false}", "{\"keep_awake\":true}"};
  for (const char* b : bad) { TEST_ASSERT_FALSE_MESSAGE(parse(b, on), b); }
  TEST_ASSERT_FALSE(keep_awake_http::parseBody(nullptr, on));
}

extern "C" void run_keep_awake_http_tests(void)
{
  RUN_TEST(test_keep_awake_body_accepts_one_bool);
  RUN_TEST(test_keep_awake_body_rejects_everything_else);
}
