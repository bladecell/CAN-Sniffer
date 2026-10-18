// P2.13a: TEST-1. Hostile-input parsers behind the SD REST routes and the DTC query string.
#include <cstring>
#include <string>

#include "../../../components/sd_card/include/sd_path.hpp"
#include "../../../main/include/http_parse.hpp"
#include "unity.h"

namespace
{
using http_parse::Result;

void test_valid_relative_accepts_normal_paths(void)
{
  TEST_ASSERT_TRUE(sd_path::validRelative("/a/b.csv", false));
  TEST_ASSERT_TRUE(sd_path::validRelative("a/b", false));
  TEST_ASSERT_TRUE(sd_path::validRelative("/logs/", false));
  TEST_ASSERT_TRUE(sd_path::validRelative("/..a/.b/a..", false));
  TEST_ASSERT_TRUE(sd_path::validRelative("/my file (1).csv", false));
}

void test_valid_relative_root_only_when_allowed(void)
{
  TEST_ASSERT_TRUE(sd_path::validRelative("", true));
  TEST_ASSERT_TRUE(sd_path::validRelative("/", true));
  TEST_ASSERT_FALSE(sd_path::validRelative("", false));
  TEST_ASSERT_FALSE(sd_path::validRelative("/", false));
  TEST_ASSERT_FALSE(sd_path::validRelative(nullptr, true));
}

void test_valid_relative_rejects_hostile(void)
{
  const char* bad[] = {"/..",      "/../x",   "/a/../b", "/a/..",  "/.",     "/a/./b", "//a",     "/a//b",
                       "/a//",     "/a\\b",   "/a?b",    "/a#b",   "/a\"b",  "/a\tb",  "/a\nb",   "/a\x7f",
                       "/a/\x01",  "../a",    "./a",     "/a/../../etc/passwd"};
  for (const char* p : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(sd_path::validRelative(p, true), p);
  }
}

void test_is_under_uses_component_boundary(void)
{
  TEST_ASSERT_TRUE(sd_path::isUnder("/sdcard", "/sdcard"));
  TEST_ASSERT_TRUE(sd_path::isUnder("/sdcard/a/b", "/sdcard"));
  TEST_ASSERT_TRUE(sd_path::isUnder("/sdcard/", "/sdcard"));
  TEST_ASSERT_FALSE(sd_path::isUnder("/sdcard2/a", "/sdcard"));
  TEST_ASSERT_FALSE(sd_path::isUnder("/sdcardx", "/sdcard"));
  TEST_ASSERT_FALSE(sd_path::isUnder("/sdcard/../etc", "/sdcard"));
  TEST_ASSERT_FALSE(sd_path::isUnder("/sdcard//a", "/sdcard"));
  TEST_ASSERT_FALSE(sd_path::isUnder("/sd", "/sdcard"));
  TEST_ASSERT_FALSE(sd_path::isUnder(nullptr, "/sdcard"));
  TEST_ASSERT_FALSE(sd_path::isUnder("/sdcard", nullptr));
}

Result rest(const char* uri, char* out, size_t n, bool root = false)
{
  return http_parse::sdRestPath(uri, "/api/v1/sd_card/file", out, n, root);
}

void test_rest_path_decodes(void)
{
  char out[64];
  TEST_ASSERT_EQUAL(Result::Ok, rest("/api/v1/sd_card/file/logs/a%20b.csv", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("/logs/a b.csv", out);
  TEST_ASSERT_EQUAL(Result::Ok, rest("/api/v1/sd_card/file/x?download=1", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("/x", out);
  TEST_ASSERT_EQUAL(Result::Ok, rest("/api/v1/sd_card/file/%41%62", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("/Ab", out);
}

void test_rest_path_root_rules(void)
{
  char out[16];
  TEST_ASSERT_EQUAL(Result::Ok, rest("/api/v1/sd_card/file", out, sizeof(out), true));
  TEST_ASSERT_EQUAL(Result::Ok, rest("/api/v1/sd_card/file/", out, sizeof(out), true));
  TEST_ASSERT_EQUAL(Result::Ok, rest("/api/v1/sd_card/file?x=1", out, sizeof(out), true));
  TEST_ASSERT_EQUAL(Result::InvalidArg, rest("/api/v1/sd_card/file", out, sizeof(out), false));
  TEST_ASSERT_EQUAL(Result::InvalidArg, rest("/api/v1/sd_card/file/", out, sizeof(out), false));
}

void test_rest_path_rejects_hostile(void)
{
  char out[64];
  const char* bad[] = {
      "/api/v1/sd_card/filex/a",             // route boundary
      "/api/v1/other/a",                     // wrong route
      "/api/v1/sd_card/file/%2e%2e/etc",     // decodes to a ".." component
      "/api/v1/sd_card/file/%2E%2E",
      "/api/v1/sd_card/file/a/../b",
      "/api/v1/sd_card/file/a%2Fb",          // encoded separator
      "/api/v1/sd_card/file/a%2fb",
      "/api/v1/sd_card/file/a%5Cb",          // encoded backslash
      "/api/v1/sd_card/file/a\\b",
      "/api/v1/sd_card/file/a%3Fb",          // encoded '?'
      "/api/v1/sd_card/file/a%23b",          // encoded '#'
      "/api/v1/sd_card/file/a#b",
      "/api/v1/sd_card/file/a?x=1#frag",     // fragment after query
      "/api/v1/sd_card/file/a%00b",          // NUL
      "/api/v1/sd_card/file/a%0Ab",          // control byte
      "/api/v1/sd_card/file/a%7Fb",
      "/api/v1/sd_card/file/a%",             // truncated escapes
      "/api/v1/sd_card/file/a%2",
      "/api/v1/sd_card/file/a%zz",
      "/api/v1/sd_card/file/a%2g",
      "/api/v1/sd_card/file//a",
      "/api/v1/sd_card/file/a//b",
      "/api/v1/sd_card/file/%22",            // '"'
  };
  for (const char* uri : bad)
  {
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(Result::InvalidArg), static_cast<int>(rest(uri, out, sizeof(out), true)), uri);
  }
}

void test_rest_path_buffer_bounds(void)
{
  char out[8];
  // "/abcdefg" needs 9 bytes with NUL: one too many for out[8].
  TEST_ASSERT_EQUAL(Result::NoMem, rest("/api/v1/sd_card/file/abcdefg", out, sizeof(out)));
  TEST_ASSERT_EQUAL(Result::Ok, rest("/api/v1/sd_card/file/abcdef", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("/abcdef", out);
  char one[1];
  TEST_ASSERT_EQUAL(Result::NoMem, rest("/api/v1/sd_card/file/a", one, 1));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::sdRestPath(nullptr, "/r", out, sizeof(out), true));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::sdRestPath("/r", "/r", out, 0, true));
  // Long input must not overrun a canary.
  char buf[16];
  std::memset(buf, 0x5a, sizeof(buf));
  const std::string longUri = "/api/v1/sd_card/file/" + std::string(500, 'a');
  TEST_ASSERT_EQUAL(Result::NoMem, rest(longUri.c_str(), buf, 8));
  TEST_ASSERT_EQUAL_HEX8(0x5a, buf[8]);
}

void test_strict_int(void)
{
  int v = 0;
  TEST_ASSERT_EQUAL(Result::Ok, http_parse::strictInt("-12", &v));
  TEST_ASSERT_EQUAL_INT(-12, v);
  const char* bad[] = {"", "1x", " 1x", "1 ", "99999999999999999999", "2147483648", "-2147483649", "abc", "0x10 "};
  for (const char* b : bad)
  {
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(Result::InvalidArg), static_cast<int>(http_parse::strictInt(b, &v)), b);
  }
  TEST_ASSERT_EQUAL(Result::Ok, http_parse::strictInt("2147483647", &v));
  TEST_ASSERT_EQUAL_INT(2147483647, v);
}

void test_path_int(void)
{
  int v = 0;
  TEST_ASSERT_EQUAL(Result::Ok, http_parse::pathInt("/api/v1/pid_def/12", "/api/v1/pid_def/", &v));
  TEST_ASSERT_EQUAL_INT(12, v);
  TEST_ASSERT_EQUAL(Result::Ok, http_parse::pathInt("/api/v1/pid_def/12?x=1", "/api/v1/pid_def/", &v));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::pathInt("/api/v1/pid_def/", "/api/v1/pid_def/", &v));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::pathInt("/api/v1/pid_def/?a", "/api/v1/pid_def/", &v));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::pathInt("/api/v1/pid_def/1/2", "/api/v1/pid_def/", &v));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::pathInt("/api/v1/pid_def/x", "/api/v1/pid_def/", &v));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::pathInt("/api/v1/pid_def/99999999999", "/api/v1/pid_def/", &v));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::pathInt("/api/v2/pid_def/1", "/api/v1/pid_def/", &v));
}

void test_query_bool(void)
{
  TEST_ASSERT_TRUE(http_parse::queryBool("true"));
  TEST_ASSERT_TRUE(http_parse::queryBool("TRUE"));
  TEST_ASSERT_TRUE(http_parse::queryBool("1"));
  TEST_ASSERT_FALSE(http_parse::queryBool("0"));
  TEST_ASSERT_FALSE(http_parse::queryBool("yes"));
  TEST_ASSERT_FALSE(http_parse::queryBool(""));
  TEST_ASSERT_FALSE(http_parse::queryBool(nullptr));
}

void test_dtc_percent_decode(void)
{
  char out[16];
  TEST_ASSERT_TRUE(http_parse::percentDecodeDtcCodes("P0100%2CP0200", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("P0100,P0200", out);
  TEST_ASSERT_TRUE(http_parse::percentDecodeDtcCodes("%2c%2C", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING(",,", out);
  TEST_ASSERT_FALSE(http_parse::percentDecodeDtcCodes("P01%", out, sizeof(out)));
  TEST_ASSERT_FALSE(http_parse::percentDecodeDtcCodes("P01%2", out, sizeof(out)));
  TEST_ASSERT_FALSE(http_parse::percentDecodeDtcCodes("P01%zz", out, sizeof(out)));
  TEST_ASSERT_FALSE(http_parse::percentDecodeDtcCodes("P01%00", out, sizeof(out)));
  TEST_ASSERT_FALSE(http_parse::percentDecodeDtcCodes(nullptr, out, sizeof(out)));
  TEST_ASSERT_FALSE(http_parse::percentDecodeDtcCodes("P", out, 0));
  char tiny[4];
  TEST_ASSERT_FALSE(http_parse::percentDecodeDtcCodes("P010", tiny, sizeof(tiny)));
  TEST_ASSERT_TRUE(http_parse::percentDecodeDtcCodes("P01", tiny, sizeof(tiny)));
}

void test_dtc_code_list(void)
{
  char        scratch[64];
  const char* ptrs[3];
  size_t      n = 99;
  TEST_ASSERT_EQUAL(Result::Ok, http_parse::dtcCodeList("P0100,U0200%2CB0300", scratch, sizeof(scratch), ptrs, 3, &n));
  TEST_ASSERT_EQUAL_size_t(3, n);
  TEST_ASSERT_EQUAL_STRING("P0100", ptrs[0]);
  TEST_ASSERT_EQUAL_STRING("U0200", ptrs[1]);
  TEST_ASSERT_EQUAL_STRING("B0300", ptrs[2]);

  TEST_ASSERT_EQUAL(Result::InvalidSize, http_parse::dtcCodeList("A,B,C,D", scratch, sizeof(scratch), ptrs, 3, &n));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::dtcCodeList("", scratch, sizeof(scratch), ptrs, 3, &n));
  // Empty leading/middle tokens are split as-is; the caller's isValidDtcCode (length 5) rejects them.
  TEST_ASSERT_EQUAL(Result::Ok, http_parse::dtcCodeList(",P0100", scratch, sizeof(scratch), ptrs, 3, &n));
  TEST_ASSERT_FALSE(http_parse::isValidDtcCode(ptrs[0]));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::dtcCodeList("P0100,", scratch, sizeof(scratch), ptrs, 3, &n));
  TEST_ASSERT_EQUAL(Result::Ok, http_parse::dtcCodeList("P0100,,P0200", scratch, sizeof(scratch), ptrs, 3, &n));
  TEST_ASSERT_FALSE(http_parse::isValidDtcCode(ptrs[1]));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::dtcCodeList("P01%", scratch, sizeof(scratch), ptrs, 3, &n));
  TEST_ASSERT_EQUAL(Result::InvalidArg, http_parse::dtcCodeList("P0100,P0200", scratch, 5, ptrs, 3, &n));
  TEST_ASSERT_EQUAL(Result::InvalidSize, http_parse::dtcCodeList("A", scratch, sizeof(scratch), ptrs, 0, &n));
  TEST_ASSERT_EQUAL_size_t(0, n);
}

void test_dtc_code_validity(void)
{
  TEST_ASSERT_TRUE(http_parse::isValidDtcCode("P0100"));
  TEST_ASSERT_TRUE(http_parse::isValidDtcCode("U3FFF"));
  TEST_ASSERT_TRUE(http_parse::isValidDtcCode("B1A2C"));
  TEST_ASSERT_TRUE(http_parse::isValidDtcCode("C0000"));
  const char* bad[] = {"",      "P010",  "P01000", "X0100", "p0100", "P4100", "P0g00", "P01a0", "P 100", "P0100\n", "P01\0" "0", "%50100"};
  for (const char* b : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(http_parse::isValidDtcCode(b), b);
  }
  TEST_ASSERT_FALSE(http_parse::isValidDtcCode(nullptr));
}
}  // namespace

extern "C" void run_http_parse_tests(void)
{
  RUN_TEST(test_valid_relative_accepts_normal_paths);
  RUN_TEST(test_valid_relative_root_only_when_allowed);
  RUN_TEST(test_valid_relative_rejects_hostile);
  RUN_TEST(test_is_under_uses_component_boundary);
  RUN_TEST(test_rest_path_decodes);
  RUN_TEST(test_rest_path_root_rules);
  RUN_TEST(test_rest_path_rejects_hostile);
  RUN_TEST(test_rest_path_buffer_bounds);
  RUN_TEST(test_strict_int);
  RUN_TEST(test_path_int);
  RUN_TEST(test_query_bool);
  RUN_TEST(test_dtc_percent_decode);
  RUN_TEST(test_dtc_code_list);
  RUN_TEST(test_dtc_code_validity);
}
