// POST /can/send body rules: id and data parsing, single-frame PCI check, SID allow-list.
#include <cstring>
#include <string>

#include "../../../../main/include/raw_tx_policy.hpp"
#include "cJSON.h"
#include "unity.h"

using namespace raw_tx;

namespace
{
struct Parsed
{
  Error   err;
  Request req;
};

Parsed parseJson(const char* json)
{
  Parsed p{};
  p.req.len  = 99;
  cJSON* root = cJSON_Parse(json);
  p.err       = parse(root, p.req);
  cJSON_Delete(root);
  return p;
}
int e(Error x) { return static_cast<int>(x); }
}  // namespace

static void test_valid_requests_parse(void)
{
  Parsed p = parseJson("{\"id\":2015,\"data\":\"02 01 00\"}");
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  TEST_ASSERT_EQUAL_UINT32(0x7DF, p.req.id);
  TEST_ASSERT_EQUAL(3, p.req.len);
  TEST_ASSERT_EQUAL_UINT8(0x01, p.req.data[1]);
  p = parseJson("{\"id\":2016,\"data\":\"0209 02\"}");  // separators are optional
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  TEST_ASSERT_EQUAL(3, p.req.len);
  p = parseJson("{\"data\":\"03 22 F1 90 00 00 00 00\",\"id\":2023}");  // padded to 8, key order free
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  TEST_ASSERT_EQUAL(8, p.req.len);
  TEST_ASSERT_EQUAL_UINT32(0x7E7, p.req.id);
  p = parseJson("{\"id\":2015,\"data\":\"02 0a 0a\"}");  // lower case hex
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
}

static void test_any_standard_id_passes(void)
{
  for (uint32_t id = 0; id < 0x800; id++)
  {
    TEST_ASSERT_TRUE(idAllowed(id));
  }
  TEST_ASSERT_FALSE(idAllowed(0x800));
  TEST_ASSERT_FALSE(idAllowed(0x18DB33F1));
  TEST_ASSERT_FALSE(idAllowed(0xFFFFFFFFu));
  Parsed p = parseJson("{\"id\":0,\"data\":\"FF\"}");
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  p = parseJson("{\"id\":2047,\"data\":\"10 08 22 F1 90 00 00 00\"}");  // first frame, service 22: no PCI or service rules
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  p = parseJson("{\"id\":512,\"data\":\"01 04\"}");  // clear DTC is no longer filtered here (D1)
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  const char* bad[] = {"{\"id\":2048,\"data\":\"02 01 00\"}", "{\"id\":-1,\"data\":\"02 01 00\"}",
                       "{\"id\":2015.5,\"data\":\"02 01 00\"}", "{\"id\":\"2015\",\"data\":\"02 01 00\"}",
                       "{\"id\":1e300,\"data\":\"02 01 00\"}", "{\"id\":null,\"data\":\"02 01 00\"}",
                       "{\"id\":4294969311,\"data\":\"02 01 00\"}", "{\"id\":true,\"data\":\"02 01 00\"}"};
  for (const char* b : bad)
  {
    TEST_ASSERT_EQUAL_MESSAGE(e(Error::badId), e(parseJson(b).err), b);
  }
}

static void test_bad_bodies(void)
{
  const char* bad[] = {"[]", "42", "\"x\"", "null", "{}", "{\"id\":2015}", "{\"data\":\"02 01 00\"}",
                       "{\"id\":2015,\"data\":\"02 01 00\",\"extra\":1}", "{\"id\":2015,\"data\":5}", "{\"id\":2015,\"data\":[2,1,0]}"};
  for (const char* b : bad)
  {
    TEST_ASSERT_EQUAL_MESSAGE(e(Error::badBody), e(parseJson(b).err), b);
  }
  Request r{};
  TEST_ASSERT_EQUAL(e(Error::badBody), e(parse(nullptr, r)));
}

static void test_hex_parser_bounds(void)
{
  uint8_t out[8];
  size_t  n = 77;
  TEST_ASSERT_TRUE(parseHex("02 01 00", out, sizeof(out), n));
  TEST_ASSERT_EQUAL(3, n);
  TEST_ASSERT_TRUE(parseHex("0201", out, sizeof(out), n));
  TEST_ASSERT_EQUAL(2, n);
  const char* bad[] = {"", " ", "0", "020", "02 0", "02  01", " 02 01", "02 01 ", "0x02", "zz", "02-01", "02\t01", "02,01",
                       "0\xC3\xA9", "02 01 00 00 00 00 00 00 00", "0201000000000000 00",
                       "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"};
  for (const char* b : bad)
  {
    n = 55;
    TEST_ASSERT_FALSE_MESSAGE(parseHex(b, out, sizeof(out), n), b);
  }
  TEST_ASSERT_FALSE(parseHex(nullptr, out, sizeof(out), n));
  TEST_ASSERT_FALSE(parseHex("01", nullptr, 8, n));
  TEST_ASSERT_FALSE(parseHex("01 02", out, 1, n));  // capacity smaller than the data
  TEST_ASSERT_TRUE(parseHex("01 02 03 04 05 06 07 08", out, sizeof(out), n));
  TEST_ASSERT_EQUAL(8, n);
}

static void test_dlc_bounds(void)
{
  TEST_ASSERT_EQUAL(e(Error::none), e(parseJson("{\"id\":1,\"data\":\"01 02 03 04 05 06 07 08\"}").err));
  TEST_ASSERT_EQUAL(e(Error::badData), e(parseJson("{\"id\":1,\"data\":\"01 02 03 04 05 06 07 08 09\"}").err));  // DLC 9
  TEST_ASSERT_EQUAL(e(Error::badData), e(parseJson("{\"id\":1,\"data\":\"\"}").err));
  TEST_ASSERT_EQUAL(e(Error::badData), e(parseJson("{\"id\":1,\"data\":\"0\"}").err));
  TEST_ASSERT_EQUAL(e(Error::badData), e(parseJson("{\"id\":1,\"data\":\"zz\"}").err));
}

namespace
{
struct PlanParsed
{
  Error err;
  Plan  plan;
};
PlanParsed planJson(const char* json)
{
  PlanParsed p{};
  cJSON*     root = cJSON_Parse(json);
  p.err           = parsePlan(root, p.plan);
  cJSON_Delete(root);
  return p;
}
std::string frames(int n)
{
  std::string s = "[";
  for (int i = 0; i < n; i++) { s += std::string(i ? "," : "") + "{\"id\":" + std::to_string(i) + ",\"data\":\"AA\"}"; }
  return s + "]";
}
}  // namespace

static void test_plan_valid(void)
{
  PlanParsed p = planJson("{\"frames\":[{\"id\":256,\"data\":\"01 02\"},{\"id\":257,\"data\":\"03\"}],\"period_ms\":10,\"repeat\":3}");
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  TEST_ASSERT_EQUAL(2, p.plan.count);
  TEST_ASSERT_EQUAL_UINT32(10, p.plan.periodMs);
  TEST_ASSERT_EQUAL_UINT32(3, p.plan.repeat);
  TEST_ASSERT_EQUAL_UINT32(257, p.plan.frames[1].id);
  TEST_ASSERT_EQUAL(1, p.plan.frames[1].len);
  p = planJson(("{\"frames\":" + frames(32) + ",\"period_ms\":10}").c_str());  // 32 frames, repeat defaults to 1
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  TEST_ASSERT_EQUAL(32, p.plan.count);
  TEST_ASSERT_EQUAL_UINT32(1, p.plan.repeat);
  p = planJson("{\"frames\":[{\"id\":1,\"data\":\"AA\"}],\"period_ms\":3600000,\"repeat\":0}");  // 0 = until stopped
  TEST_ASSERT_EQUAL(e(Error::none), e(p.err));
  TEST_ASSERT_EQUAL_UINT32(0, p.plan.repeat);
}

static void test_plan_rejects_bad_input(void)
{
  TEST_ASSERT_EQUAL(e(Error::badFrames), e(planJson(("{\"frames\":" + frames(33) + ",\"period_ms\":10}").c_str()).err));  // 33 frames
  TEST_ASSERT_EQUAL(e(Error::badFrames), e(planJson("{\"frames\":[],\"period_ms\":10}").err));
  TEST_ASSERT_EQUAL(e(Error::badFrames), e(planJson("{\"frames\":5,\"period_ms\":10}").err));
  TEST_ASSERT_EQUAL(e(Error::badFrames), e(planJson("{\"frames\":[5],\"period_ms\":10}").err));
  TEST_ASSERT_EQUAL(e(Error::badFrames), e(planJson("{\"frames\":[{\"id\":1,\"data\":\"AA\",\"x\":1}],\"period_ms\":10}").err));
  TEST_ASSERT_EQUAL(e(Error::badId), e(planJson("{\"frames\":[{\"id\":2048,\"data\":\"AA\"}],\"period_ms\":10}").err));
  TEST_ASSERT_EQUAL(e(Error::badData), e(planJson("{\"frames\":[{\"id\":1,\"data\":\"01 02 03 04 05 06 07 08 09\"}],\"period_ms\":10}").err));
  const char* periods[] = {"5", "9", "0", "-10", "3600001", "10.5", "\"10\"", "null", "1e300"};
  for (const char* per : periods)
  {
    const std::string j = std::string("{\"frames\":[{\"id\":1,\"data\":\"AA\"}],\"period_ms\":") + per + "}";
    TEST_ASSERT_EQUAL_MESSAGE(e(Error::badPeriod), e(planJson(j.c_str()).err), per);
  }
  TEST_ASSERT_EQUAL(e(Error::badPeriod), e(planJson("{\"frames\":[{\"id\":1,\"data\":\"AA\"}]}").err));  // period is required
  const char* repeats[] = {"-1", "65536", "1.5", "\"2\"", "null"};
  for (const char* rep : repeats)
  {
    const std::string j = std::string("{\"frames\":[{\"id\":1,\"data\":\"AA\"}],\"period_ms\":10,\"repeat\":") + rep + "}";
    TEST_ASSERT_EQUAL_MESSAGE(e(Error::badRepeat), e(planJson(j.c_str()).err), rep);
  }
  TEST_ASSERT_EQUAL(e(Error::badBody), e(planJson("{\"frames\":[{\"id\":1,\"data\":\"AA\"}],\"period_ms\":10,\"id\":1}").err));
  TEST_ASSERT_EQUAL(e(Error::badBody), e(planJson("[]").err));
  Plan pl{};
  TEST_ASSERT_EQUAL(e(Error::badBody), e(parsePlan(nullptr, pl)));
}

static void test_cursor_walks_the_sequence(void)
{
  Cursor c(3, 2);
  const size_t want[] = {0, 1, 2, 0, 1, 2};
  for (size_t w : want)
  {
    TEST_ASSERT_FALSE(c.done());
    TEST_ASSERT_EQUAL(w, c.next());
  }
  TEST_ASSERT_TRUE(c.done());
  TEST_ASSERT_EQUAL_UINT32(6, c.sent());
  Cursor inf(2, 0);  // repeat 0: until stopped
  for (int i = 0; i < 1000; i++) { TEST_ASSERT_FALSE(inf.done()); TEST_ASSERT_EQUAL(static_cast<size_t>(i % 2), inf.next()); }
  Cursor empty(0, 1);
  TEST_ASSERT_TRUE(empty.done());
}

static void test_http_mapping(void)
{
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badBody));
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badId));
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badData));
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badFrames));
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badPeriod));
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badRepeat));
  TEST_ASSERT_EQUAL_STRING("BAD_FRAMES", code(Error::badFrames));
  TEST_ASSERT_EQUAL_STRING("BAD_PERIOD", code(Error::badPeriod));
  TEST_ASSERT_EQUAL_STRING("BAD_REPEAT", code(Error::badRepeat));
  TEST_ASSERT_EQUAL_STRING("BAD_ID", code(Error::badId));
  TEST_ASSERT_EQUAL_STRING("BAD_DATA", code(Error::badData));
  TEST_ASSERT_EQUAL_STRING("BAD_JSON", code(Error::badBody));
  TEST_ASSERT_EQUAL(250, kMinIntervalMs);
}

extern "C" void run_raw_tx_policy_tests(void)
{
  RUN_TEST(test_valid_requests_parse);
  RUN_TEST(test_any_standard_id_passes);
  RUN_TEST(test_bad_bodies);
  RUN_TEST(test_hex_parser_bounds);
  RUN_TEST(test_dlc_bounds);
  RUN_TEST(test_plan_valid);
  RUN_TEST(test_plan_rejects_bad_input);
  RUN_TEST(test_cursor_walks_the_sequence);
  RUN_TEST(test_http_mapping);
}
