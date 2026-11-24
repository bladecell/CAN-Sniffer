// Recorder HTTP/WS wire rules: start body parsing and the 0x04 state frame.
#include <cstring>
#include <string>

#include "recorder_http.hpp"
#include "unity.h"

using namespace recorder_http;
using recorder_core::PidSel;

struct Parsed
{
  bool   ok;
  size_t n;
  PidSel sel[recorder_core::kMaxPids];
};

static Parsed parse(const char* json)
{
  Parsed p{};
  p.n          = 77;
  cJSON* root  = cJSON_Parse(json);
  p.ok         = parseStart(root, p.sel, p.n);
  cJSON_Delete(root);
  return p;
}

static void test_start_body_empty_object_means_polled_set(void)
{
  Parsed p = parse("{}");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL(0, p.n);
  p = parse("{\"pids\":[]}");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL(0, p.n);
}

static void test_start_body_pid_list(void)
{
  Parsed p = parse("{\"pids\":[{\"pid\":12},{\"pid\":13,\"min_interval_ms\":500}]}");
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL(2, p.n);
  TEST_ASSERT_EQUAL_UINT16(12, p.sel[0].pid);
  TEST_ASSERT_EQUAL_UINT16(0, p.sel[0].minSpacingMs);
  TEST_ASSERT_EQUAL_UINT16(13, p.sel[1].pid);
  TEST_ASSERT_EQUAL_UINT16(500, p.sel[1].minSpacingMs);
  p = parse("{\"pids\":[{\"pid\":65535,\"min_interval_ms\":60000}]}");
  TEST_ASSERT_TRUE(p.ok);
}

static void test_start_body_garbage_is_rejected(void)
{
  const char* bad[] = {"[]", "5", "\"x\"", "null", "{\"pids\":5}", "{\"pids\":{}}", "{\"pids\":[5]}",
                       "{\"pids\":[{}]}", "{\"pids\":[{\"pid\":\"12\"}]}", "{\"pids\":[{\"pid\":1.5}]}",
                       "{\"pids\":[{\"pid\":-1}]}", "{\"pids\":[{\"pid\":65536}]}",
                       "{\"pids\":[{\"pid\":1,\"min_interval_ms\":-1}]}",
                       "{\"pids\":[{\"pid\":1,\"min_interval_ms\":60001}]}",
                       "{\"pids\":[{\"pid\":1,\"min_interval_ms\":\"5\"}]}",
                       "{\"pids\":[{\"pid\":1,\"extra\":1}]}", "{\"other\":1}", "{\"pids\":[],\"x\":1}",
                       "{\"pids\":[{\"pid\":1e400}]}"};
  for (const char* b : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(parse(b).ok, b);
  }
  size_t unused = 0;
  TEST_ASSERT_FALSE(parseStart(nullptr, nullptr, unused));
}

static void test_start_body_more_than_32_pids_is_rejected_without_overflow(void)
{
  std::string s = "{\"pids\":[";
  for (int i = 0; i < 33; ++i) s += std::string(i ? "," : "") + "{\"pid\":" + std::to_string(i) + "}";
  s += "]}";
  TEST_ASSERT_FALSE(parse(s.c_str()).ok);
  s = "{\"pids\":[";
  for (int i = 0; i < 32; ++i) s += std::string(i ? "," : "") + "{\"pid\":" + std::to_string(i) + "}";
  s += "]}";
  const Parsed p = parse(s.c_str());
  TEST_ASSERT_TRUE(p.ok);
  TEST_ASSERT_EQUAL(32, p.n);
}

static void test_state_frame_layout_is_fixed_16_bytes_little_endian(void)
{
  uint8_t f[kStateFrameBytes];
  memset(f, 0xEE, sizeof(f));
  encodeStateFrame(f, {recorder_core::State::Recording, 0x01020304u, 0x0A0B0C0Du, 0x11223344u, true});
  const uint8_t want[kStateFrameBytes] = {0x04, 14, 1, 0x04, 0x03, 0x02, 0x01, 0x0D, 0x0C, 0x0B, 0x0A, 0x44, 0x33, 0x22, 0x11, 1};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(want, f, kStateFrameBytes);
  encodeStateFrame(f, {recorder_core::State::Error, 0, 0, 0, false});
  TEST_ASSERT_EQUAL_UINT8(3, f[2]);
  TEST_ASSERT_EQUAL_UINT8(0, f[15]);
  encodeStateFrame(f, {recorder_core::State::Idle, 0, 0, 0, false});
  TEST_ASSERT_EQUAL_UINT8(0, f[2]);
  encodeStateFrame(f, {recorder_core::State::Stopping, 0, 0, 0, false});
  TEST_ASSERT_EQUAL_UINT8(2, f[2]);
}

static void test_start_error_mapping(void)
{
  TEST_ASSERT_EQUAL(200, startHttpStatus(ESP_OK));
  TEST_ASSERT_EQUAL(409, startHttpStatus(ESP_ERR_NOT_FINISHED));  // live session, including one paused in Error
  TEST_ASSERT_EQUAL(422, startHttpStatus(ESP_ERR_INVALID_ARG));
  TEST_ASSERT_EQUAL(429, startHttpStatus(ESP_ERR_NOT_ALLOWED));  // start cooldown
  TEST_ASSERT_EQUAL(503, startHttpStatus(ESP_ERR_INVALID_STATE));  // not initialized
  TEST_ASSERT_EQUAL(503, startHttpStatus(ESP_ERR_NOT_FOUND));      // no card
  TEST_ASSERT_EQUAL(503, startHttpStatus(ESP_ERR_TIMEOUT));
  TEST_ASSERT_EQUAL(500, startHttpStatus(ESP_ERR_NO_MEM));
  TEST_ASSERT_EQUAL(503, startHttpStatus(ESP_FAIL));  // the first file could not be opened: card error
}


static bool patchOf(const char* json, TriggerPatch& p)
{
  p            = {};
  cJSON* root  = cJSON_Parse(json);
  const bool ok = parseTriggerPatch(root, p);
  cJSON_Delete(root);
  return ok;
}

static void test_trigger_patch_accepts_each_key_alone_or_together(void)
{
  TriggerPatch p;
  TEST_ASSERT_TRUE(patchOf("{\"trigger\":\"auto\"}", p));
  TEST_ASSERT_TRUE(p.hasMode);
  TEST_ASSERT_EQUAL(trip_trigger::Mode::Auto, p.mode);
  TEST_ASSERT_FALSE(p.hasContinuous);
  TEST_ASSERT_TRUE(patchOf("{\"continuous_polling\":true}", p));
  TEST_ASSERT_FALSE(p.hasMode);
  TEST_ASSERT_TRUE(p.hasContinuous);
  TEST_ASSERT_TRUE(p.continuous);
  TEST_ASSERT_TRUE(patchOf("{\"trigger\":\"manual\",\"continuous_polling\":false}", p));
  TEST_ASSERT_EQUAL(trip_trigger::Mode::Manual, p.mode);
  TEST_ASSERT_FALSE(p.continuous);
}

static void test_trigger_patch_rejects_hostile_bodies(void)
{
  TriggerPatch p;
  const char* bad[] = {"{}", "[]", "null", "\"auto\"", "{\"trigger\":\"Auto\"}", "{\"trigger\":\"\"}", "{\"trigger\":1}",
                       "{\"trigger\":null}", "{\"continuous_polling\":1}", "{\"continuous_polling\":\"true\"}",
                       "{\"trigger\":\"auto\",\"extra\":1}", "{\"trigger\":\"auto\",\"trigger\":\"manual\"}",
                       "{\"continuous_polling\":true,\"continuous_polling\":false}", "{\"pids\":[]}"};
  for (const char* b : bad) { TEST_ASSERT_FALSE_MESSAGE(patchOf(b, p), b); }
  TEST_ASSERT_FALSE(parseTriggerPatch(nullptr, p));
}

extern "C" void run_recorder_http_tests(void)
{
  RUN_TEST(test_start_error_mapping);
  RUN_TEST(test_start_body_empty_object_means_polled_set);
  RUN_TEST(test_start_body_pid_list);
  RUN_TEST(test_start_body_garbage_is_rejected);
  RUN_TEST(test_start_body_more_than_32_pids_is_rejected_without_overflow);
  RUN_TEST(test_state_frame_layout_is_fixed_16_bytes_little_endian);
  RUN_TEST(test_trigger_patch_accepts_each_key_alone_or_together);
  RUN_TEST(test_trigger_patch_rejects_hostile_bodies);
}
