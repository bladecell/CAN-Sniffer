// POST /can/send body rules: id and data parsing, single-frame PCI check, SID allow-list.
#include <cstring>

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

static void test_only_the_obd_request_ids_pass(void)
{
  for (uint32_t id = 0; id < 0x800; id++)
  {
    const bool ok = id == 0x7DF || (id >= 0x7E0 && id <= 0x7E7);
    TEST_ASSERT_EQUAL(ok, idAllowed(id));
  }
  TEST_ASSERT_FALSE(idAllowed(0x7E8));
  TEST_ASSERT_FALSE(idAllowed(0x18DB33F1));
  TEST_ASSERT_FALSE(idAllowed(0xFFFFFFFFu));
  const char* bad[] = {"{\"id\":2014,\"data\":\"02 01 00\"}", "{\"id\":2024,\"data\":\"02 01 00\"}", "{\"id\":-1,\"data\":\"02 01 00\"}",
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

static void test_single_frame_pci_must_be_consistent(void)
{
  const char* bad[] = {"{\"id\":2015,\"data\":\"00 01\"}",       // length 0
                       "{\"id\":2015,\"data\":\"08 01 00 00 00 00 00 00\"}",  // 8 does not fit a single frame
                       "{\"id\":2015,\"data\":\"02 01\"}",       // says 2 payload bytes, has 1
                       "{\"id\":2015,\"data\":\"07 01 00\"}",
                       "{\"id\":2015,\"data\":\"10 08 22 F1 90 00 00 00\"}",  // first frame, not ours to send
                       "{\"id\":2015,\"data\":\"30 00 00\"}",    // flow control
                       "{\"id\":2015,\"data\":\"21 01 02\"}",    // consecutive frame
                       "{\"id\":2015,\"data\":\"02\"}",
                       "{\"id\":2015,\"data\":\"FF 01 00\"}"};
  for (const char* b : bad)
  {
    TEST_ASSERT_EQUAL_MESSAGE(e(Error::badData), e(parseJson(b).err), b);
  }
  TEST_ASSERT_EQUAL(e(Error::badData), e(parseJson("{\"id\":2015,\"data\":\"\"}").err));
}

static void test_only_allow_listed_services_pass(void)
{
  const uint8_t allowed[] = {0x01, 0x02, 0x03, 0x07, 0x09, 0x0A, 0x19, 0x22};
  for (int sid = 0; sid < 256; sid++)
  {
    bool want = false;
    for (uint8_t a : allowed) { want = want || a == sid; }
    TEST_ASSERT_EQUAL_MESSAGE(want, sidAllowed(static_cast<uint8_t>(sid)), "sid");
  }
  const uint8_t dangerous[] = {0x04, 0x10, 0x11, 0x14, 0x27, 0x28, 0x2E, 0x2F, 0x31, 0x34, 0x35, 0x36, 0x37, 0x3E, 0x85, 0x00, 0xFF};
  for (uint8_t sid : dangerous)
  {
    uint8_t d[3] = {0x02, sid, 0x00};
    TEST_ASSERT_EQUAL(e(Error::blocked), e(validate(0x7E0, d, 3)));
  }
  Parsed p = parseJson("{\"id\":2015,\"data\":\"01 04\"}");  // clear DTC stays on its own route
  TEST_ASSERT_EQUAL(e(Error::blocked), e(p.err));
}

static void test_check_order_id_then_data_then_service(void)
{
  TEST_ASSERT_EQUAL(e(Error::badId), e(parseJson("{\"id\":1,\"data\":\"zz\"}").err));
  TEST_ASSERT_EQUAL(e(Error::badData), e(parseJson("{\"id\":2015,\"data\":\"05 04\"}").err));  // PCI wrong beats blocked
  TEST_ASSERT_EQUAL(e(Error::blocked), e(parseJson("{\"id\":2015,\"data\":\"02 04 00\"}").err));
}

static void test_http_mapping(void)
{
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badBody));
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badId));
  TEST_ASSERT_EQUAL(422, httpStatus(Error::badData));
  TEST_ASSERT_EQUAL(403, httpStatus(Error::blocked));
  TEST_ASSERT_EQUAL_STRING("BAD_ID", code(Error::badId));
  TEST_ASSERT_EQUAL_STRING("BAD_DATA", code(Error::badData));
  TEST_ASSERT_EQUAL_STRING("SERVICE_BLOCKED", code(Error::blocked));
  TEST_ASSERT_EQUAL_STRING("BAD_JSON", code(Error::badBody));
  TEST_ASSERT_EQUAL(250, kMinIntervalMs);
}

extern "C" void run_raw_tx_policy_tests(void)
{
  RUN_TEST(test_valid_requests_parse);
  RUN_TEST(test_only_the_obd_request_ids_pass);
  RUN_TEST(test_bad_bodies);
  RUN_TEST(test_hex_parser_bounds);
  RUN_TEST(test_single_frame_pci_must_be_consistent);
  RUN_TEST(test_only_allow_listed_services_pass);
  RUN_TEST(test_check_order_id_then_data_then_service);
  RUN_TEST(test_http_mapping);
}
