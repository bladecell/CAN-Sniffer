// P2.11 NB2/NB3 (B22): the one HTTP error contract, hardware-free.
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "../../../components/async_web_server/include/http_error.hpp"
#include "unity.h"

namespace
{
namespace he = http_error;

// {"status":"error","reason":"X","code":"Y"} has exactly 12 unescaped quotes, no raw control bytes and no byte >= 0x80.
bool wellFormed(const char* b)
{
  const char* head = "{\"status\":\"error\",\"reason\":\"";
  if (strncmp(b, head, strlen(head)) != 0)
  {
    return false;
  }
  const size_t len = strlen(b);
  if (len < 2 || strcmp(b + len - 2, "\"}") != 0)
  {
    return false;
  }
  int  quotes  = 0;
  bool escaped = false;
  for (size_t i = 0; i < len; i++)
  {
    const unsigned char c = static_cast<unsigned char>(b[i]);
    if (c < 0x20 || c >= 0x80)
    {
      return false;
    }
    if (escaped)
    {
      escaped = false;
    }
    else if (c == '\\')
    {
      escaped = true;
    }
    else if (c == '"')
    {
      quotes++;
    }
  }
  return !escaped && quotes == 12 && strstr(b, "\",\"code\":\"") != nullptr;
}

void test_status_lines_are_valid_reason_phrases(void)
{
  TEST_ASSERT_EQUAL_STRING("503 Service Unavailable", he::statusLine(503));
  TEST_ASSERT_EQUAL_STRING("429 Too Many Requests", he::statusLine(429));
  TEST_ASSERT_EQUAL_STRING("502 Bad Gateway", he::statusLine(502));
  TEST_ASSERT_NULL(he::statusLine(418));
  for (int s = 100; s < 600; s++)
  {
    const char* line = he::statusLine(s);
    if (line != nullptr)
    {
      TEST_ASSERT_EQUAL_INT(s, atoi(line));  // the number in front matches the status
      TEST_ASSERT_NOT_NULL(strchr(line, ' '));
    }
  }
}

void test_unknown_or_non_error_status_is_sent_as_500(void)
{
  TEST_ASSERT_EQUAL_INT(500, he::errorStatus(200));
  TEST_ASSERT_EQUAL_INT(500, he::errorStatus(0));
  TEST_ASSERT_EQUAL_INT(500, he::errorStatus(-1));
  TEST_ASSERT_EQUAL_INT(500, he::errorStatus(418));
  TEST_ASSERT_EQUAL_INT(500, he::errorStatus(999));
  TEST_ASSERT_EQUAL_INT(404, he::errorStatus(404));
  TEST_ASSERT_EQUAL_INT(401, he::errorStatus(401));  // auth (P2.10) reuses this
  TEST_ASSERT_EQUAL_INT(403, he::errorStatus(403));
}

void test_every_error_status_has_a_reason_and_code(void)
{
  for (int s = 400; s < 600; s++)
  {
    if (he::statusLine(s) != nullptr)
    {
      TEST_ASSERT_TRUE(strlen(he::defaultReason(s)) > 0);
      TEST_ASSERT_TRUE(strlen(he::defaultCode(s)) > 0);
    }
  }
  TEST_ASSERT_EQUAL_STRING("RATE_LIMITED", he::defaultCode(429));
  TEST_ASSERT_EQUAL_STRING("BUSY", he::defaultCode(503));
  TEST_ASSERT_EQUAL_STRING("NOT_FOUND", he::defaultCode(404));
}

void test_retry_after_only_for_429_and_503(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, he::retryAfterS(429));
  TEST_ASSERT_EQUAL_UINT32(1, he::retryAfterS(503));
  TEST_ASSERT_EQUAL_UINT32(0, he::retryAfterS(404));
  TEST_ASSERT_EQUAL_UINT32(0, he::retryAfterS(401));
}

void test_body_shape(void)
{
  char b[he::kBodyMax];
  const size_t n = he::buildBody(b, sizeof(b), 404, "File not found", nullptr);
  TEST_ASSERT_EQUAL_STRING("{\"status\":\"error\",\"reason\":\"File not found\",\"code\":\"NOT_FOUND\"}", b);
  TEST_ASSERT_EQUAL_UINT32(strlen(b), n);
}

void test_body_defaults_for_null_and_empty(void)
{
  char b[he::kBodyMax];
  he::buildBody(b, sizeof(b), 503, nullptr, "");
  TEST_ASSERT_EQUAL_STRING("{\"status\":\"error\",\"reason\":\"Service unavailable\",\"code\":\"BUSY\"}", b);
}

void test_body_explicit_code(void)
{
  char b[he::kBodyMax];
  he::buildBody(b, sizeof(b), 503, "Server is shutting down", he::kShuttingDown);
  TEST_ASSERT_NOT_NULL(strstr(b, "\"code\":\"SHUTTING_DOWN\""));
  he::buildBody(b, sizeof(b), 504, "x", "ESP_ERR_TIMEOUT");
  TEST_ASSERT_NOT_NULL(strstr(b, "\"code\":\"ESP_ERR_TIMEOUT\""));
}

void test_body_escapes_hostile_reason(void)
{
  char b[he::kBodyMax];
  he::buildBody(b, sizeof(b), 400, "a\"b\\c\n\x01\xc3\xa9\xff", nullptr);
  TEST_ASSERT_TRUE(wellFormed(b));
  TEST_ASSERT_NOT_NULL(strstr(b, "a\\\"b\\\\c\\u000a\\u0001??"));
}

void test_long_reason_is_cut_and_code_survives(void)
{
  char longReason[1024];
  memset(longReason, 'x', sizeof(longReason) - 1);
  longReason[sizeof(longReason) - 1] = '\0';
  char b[he::kBodyMax];
  const size_t n = he::buildBody(b, sizeof(b), 400, longReason, he::kBadPath);
  TEST_ASSERT_TRUE(n < sizeof(b));
  TEST_ASSERT_EQUAL_UINT32(strlen(b), n);
  TEST_ASSERT_TRUE(wellFormed(b));
  TEST_ASSERT_NOT_NULL(strstr(b, "\"code\":\"BAD_PATH\"}"));
}

void test_long_reason_of_escapes_never_overflows(void)
{
  // Worst case: every byte needs a 6 byte escape, at every buffer size from the minimum up. Guard bytes catch a write
  // past cap.
  char reason[400];
  memset(reason, 0x01, sizeof(reason) - 1);
  reason[sizeof(reason) - 1] = '\0';
  for (size_t cap = 96; cap <= he::kBodyMax; cap++)
  {
    char buf[he::kBodyMax + 8];
    memset(buf, 0x7e, sizeof(buf));
    const size_t n = he::buildBody(buf, cap, 500, reason, nullptr);
    TEST_ASSERT_TRUE(n < cap);
    TEST_ASSERT_EQUAL_UINT8(0x7e, static_cast<uint8_t>(buf[cap]));  // nothing written past cap
    TEST_ASSERT_TRUE(wellFormed(buf));
  }
}

void test_cut_never_splits_an_escape(void)
{
  // A quote lands exactly at the cut for some size; the result must still balance.
  char reason[300];
  memset(reason, '"', sizeof(reason) - 1);
  reason[sizeof(reason) - 1] = '\0';
  for (size_t cap = 96; cap <= he::kBodyMax; cap++)
  {
    char buf[he::kBodyMax];
    he::buildBody(buf, cap, 400, reason, nullptr);
    TEST_ASSERT_TRUE(wellFormed(buf));
  }
}

void test_tiny_or_null_buffer_is_refused(void)
{
  char b[8] = {'z'};
  TEST_ASSERT_EQUAL_UINT32(0, he::buildBody(b, sizeof(b), 400, "x", nullptr));
  TEST_ASSERT_EQUAL_UINT32(0, he::buildBody(nullptr, 200, 400, "x", nullptr));
  char c[95];
  TEST_ASSERT_EQUAL_UINT32(0, he::buildBody(c, sizeof(c), 400, "x", nullptr));
}

// P2.11 NB3 (H4): the status table.
void test_timeout_is_504_only_for_bus_waits(void)
{
  TEST_ASSERT_EQUAL_INT(504, he::statusForError(ESP_ERR_TIMEOUT));
  TEST_ASSERT_EQUAL_INT(504, he::statusForError(ESP_ERR_TIMEOUT, 400, he::Fault::Bus));
  TEST_ASSERT_EQUAL_INT(503, he::statusForError(ESP_ERR_TIMEOUT, 400, he::Fault::Local));  // PID map lock, SD lock
}

void test_not_running_and_capacity_are_503_on_every_route(void)
{
  // PUT /pid_def used to answer 500 for "OBD2 not running" while DELETE and auto_poll answered 503.
  for (int arg : {400, 404, 422})
  {
    TEST_ASSERT_EQUAL_INT(503, he::statusForError(ESP_ERR_INVALID_STATE, arg));
    TEST_ASSERT_EQUAL_INT(503, he::statusForError(ESP_ERR_INVALID_STATE, arg, he::Fault::Local));
    TEST_ASSERT_EQUAL_INT(503, he::statusForError(ESP_ERR_INVALID_SIZE, arg));
  }
}

void test_invalid_arg_uses_the_route_status(void)
{
  TEST_ASSERT_EQUAL_INT(400, he::statusForError(ESP_ERR_INVALID_ARG));
  TEST_ASSERT_EQUAL_INT(422, he::statusForError(ESP_ERR_INVALID_ARG, 422));
  TEST_ASSERT_EQUAL_INT(404, he::statusForError(ESP_ERR_INVALID_ARG, 404));
}

void test_unusable_ecu_answer_is_502(void)
{
  TEST_ASSERT_EQUAL_INT(502, he::statusForError(ESP_ERR_INVALID_RESPONSE));
  TEST_ASSERT_EQUAL_INT(502, he::statusForError(ESP_ERR_INVALID_RESPONSE, 503, he::Fault::Local));
}

void test_not_found_oom_and_unknown(void)
{
  TEST_ASSERT_EQUAL_INT(404, he::statusForError(ESP_ERR_NOT_FOUND, 422));
  TEST_ASSERT_EQUAL_INT(500, he::statusForError(ESP_ERR_NO_MEM));
  TEST_ASSERT_EQUAL_INT(500, he::statusForError(ESP_FAIL));
  TEST_ASSERT_EQUAL_INT(500, he::statusForError(static_cast<esp_err_t>(0x7fff)));
  TEST_ASSERT_EQUAL_INT(200, he::statusForError(ESP_OK));
}

void test_every_mapped_status_has_a_status_line(void)
{
  const esp_err_t errs[] = {ESP_OK, ESP_FAIL, ESP_ERR_NO_MEM, ESP_ERR_INVALID_ARG, ESP_ERR_NOT_FOUND, ESP_ERR_TIMEOUT,
                            ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_SIZE, ESP_ERR_INVALID_RESPONSE};
  for (esp_err_t e : errs)
  {
    for (he::Fault f : {he::Fault::Bus, he::Fault::Local})
    {
      TEST_ASSERT_NOT_NULL(he::statusLine(he::statusForError(e, 400, f)));
      TEST_ASSERT_NOT_NULL(he::statusLine(he::statusForError(e, 422, f)));
    }
  }
}
}  // namespace

extern "C" void run_http_error_tests(void)
{
  RUN_TEST(test_status_lines_are_valid_reason_phrases);
  RUN_TEST(test_unknown_or_non_error_status_is_sent_as_500);
  RUN_TEST(test_every_error_status_has_a_reason_and_code);
  RUN_TEST(test_retry_after_only_for_429_and_503);
  RUN_TEST(test_body_shape);
  RUN_TEST(test_body_defaults_for_null_and_empty);
  RUN_TEST(test_body_explicit_code);
  RUN_TEST(test_body_escapes_hostile_reason);
  RUN_TEST(test_long_reason_is_cut_and_code_survives);
  RUN_TEST(test_long_reason_of_escapes_never_overflows);
  RUN_TEST(test_cut_never_splits_an_escape);
  RUN_TEST(test_tiny_or_null_buffer_is_refused);
  RUN_TEST(test_timeout_is_504_only_for_bus_waits);
  RUN_TEST(test_not_running_and_capacity_are_503_on_every_route);
  RUN_TEST(test_invalid_arg_uses_the_route_status);
  RUN_TEST(test_unusable_ecu_answer_is_502);
  RUN_TEST(test_not_found_oom_and_unknown);
  RUN_TEST(test_every_mapped_status_has_a_status_line);
}
