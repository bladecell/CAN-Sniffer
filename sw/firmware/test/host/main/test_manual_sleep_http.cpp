#include "../../../main/include/manual_sleep_http.hpp"
#include "unity.h"
#include <cstdint>
#include <initializer_list>

namespace
{
struct Fake
{
  int events[4]{};
  unsigned count = 0;
  uint32_t token = 7;
  esp_err_t sendError = ESP_OK;
  uint32_t finishedToken = 0;
  bool sent = false;

  esp_err_t dispatch(const char* uri = "/api/v1/system/sleep", size_t bytes = 0, bool transfer = false)
  {
    return manual_sleep_http::dispatch(uri, bytes, transfer,
      [&]() { events[count++] = 1; return token; },
      [&](int status) { events[count++] = status; return sendError; },
      [&](uint32_t id, bool success) { events[count++] = 2; finishedToken = id; sent = success; });
  }
};

void test_sleep_dispatch_reserves_then_sends_then_publishes()
{
  Fake f;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.dispatch());
  TEST_ASSERT_EQUAL_UINT32(3, f.count);
  TEST_ASSERT_EQUAL_INT(1, f.events[0]);
  TEST_ASSERT_EQUAL_INT(202, f.events[1]);
  TEST_ASSERT_EQUAL_INT(2, f.events[2]);
  TEST_ASSERT_EQUAL_UINT32(7, f.finishedToken);
  TEST_ASSERT_TRUE(f.sent);
}

void test_sleep_send_failure_finishes_as_cancellation_and_preserves_transport_error()
{
  Fake f;
  f.sendError = ESP_FAIL;
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, f.dispatch());
  TEST_ASSERT_EQUAL_UINT32(3, f.count);
  TEST_ASSERT_EQUAL_INT(202, f.events[1]);
  TEST_ASSERT_EQUAL_UINT32(7, f.finishedToken);
  TEST_ASSERT_FALSE(f.sent);
}

void test_sleep_busy_admission_sends_503_without_publication()
{
  Fake f;
  f.token = 0;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.dispatch());
  TEST_ASSERT_EQUAL_UINT32(2, f.count);
  TEST_ASSERT_EQUAL_INT(1, f.events[0]);
  TEST_ASSERT_EQUAL_INT(503, f.events[1]);
  TEST_ASSERT_EQUAL_UINT32(0, f.finishedToken);
}

void test_sleep_rejects_nonempty_body_query_and_transfer_encoding_before_admission()
{
  const char* const uris[] = {nullptr, "/api/v1/system/sleep?", "/api/v1/system/sleep?force=true",
                             "/api/v1/system/sleep/", "/api/v1/system/sleeping", "/api/v1/system/sleep%3F"};
  for (const char* uri : uris)
  {
    Fake f;
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.dispatch(uri));
    TEST_ASSERT_EQUAL_UINT32(1, f.count);
    TEST_ASSERT_EQUAL_INT(400, f.events[0]);
  }
  for (size_t bytes : {size_t{1}, SIZE_MAX})
  {
    Fake f;
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.dispatch("/api/v1/system/sleep", bytes));
    TEST_ASSERT_EQUAL_UINT32(1, f.count);
    TEST_ASSERT_EQUAL_INT(400, f.events[0]);
  }
  Fake transfer;
  TEST_ASSERT_EQUAL_INT(ESP_OK, transfer.dispatch("/api/v1/system/sleep", 0, true));
  TEST_ASSERT_EQUAL_UINT32(1, transfer.count);
  TEST_ASSERT_EQUAL_INT(400, transfer.events[0]);
}
}

extern "C" void run_manual_sleep_http_tests()
{
  RUN_TEST(test_sleep_dispatch_reserves_then_sends_then_publishes);
  RUN_TEST(test_sleep_send_failure_finishes_as_cancellation_and_preserves_transport_error);
  RUN_TEST(test_sleep_busy_admission_sends_503_without_publication);
  RUN_TEST(test_sleep_rejects_nonempty_body_query_and_transfer_encoding_before_admission);
}
