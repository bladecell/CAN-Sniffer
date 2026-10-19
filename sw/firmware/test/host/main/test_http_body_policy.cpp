#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>

#include "../../../main/include/http_body_policy.hpp"
#include "unity.h"

namespace
{
namespace policy = http_body_policy;
constexpr size_t kMaxLength = 4194304;

void test_upload_length_distinguishes_missing_from_explicit_zero(void)
{
  size_t length = 123;
  TEST_ASSERT_FALSE(policy::uploadContentLength(nullptr, 0, 4194304, &length));
  TEST_ASSERT_FALSE(policy::uploadContentLength("", 0, 4194304, &length));
  TEST_ASSERT_TRUE(policy::uploadContentLength("0", 0, 4194304, &length));
  TEST_ASSERT_EQUAL_UINT(0, length);
}

void test_upload_length_syntax_and_framing_agreement(void)
{
  const char* invalid[] = {" ", "\t", "-1", "+1", "1.0", "1x", "1 2", "1\t2", "1,1", "1\r\n"};
  for (const char* header : invalid)
  {
    size_t length = 123;
    TEST_ASSERT_FALSE_MESSAGE(policy::uploadContentLength(header, 1, kMaxLength, &length), header);
    TEST_ASSERT_EQUAL_UINT(123, length);
  }
  size_t length = 0;
  TEST_ASSERT_TRUE(policy::uploadContentLength(" \t0012\t ", 12, kMaxLength, &length));
  TEST_ASSERT_EQUAL_UINT(12, length);
  TEST_ASSERT_FALSE(policy::uploadContentLength("12", 11, kMaxLength, &length));
  TEST_ASSERT_FALSE(policy::uploadContentLength("0", 1, kMaxLength, &length));
  TEST_ASSERT_FALSE(policy::uploadContentLength("1", 1, kMaxLength, nullptr));
}

void test_upload_length_cap_and_overflow_preserve_error_precedence(void)
{
  size_t length = 0;
  TEST_ASSERT_TRUE(policy::uploadContentLength("4194304", kMaxLength, kMaxLength, &length));
  TEST_ASSERT_EQUAL_UINT(kMaxLength, length);
  // Oversize remains 413 even if the server's framing value differs.
  TEST_ASSERT_TRUE(policy::uploadContentLength("4194305", 0, kMaxLength, &length));
  TEST_ASSERT_EQUAL_UINT(kMaxLength + 1, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("999999999999999999999999999999999999", 0, kMaxLength, &length));
  TEST_ASSERT_EQUAL_UINT(kMaxLength + 1, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("999999999999999999999999999999999999",
                                              std::numeric_limits<size_t>::max(), kMaxLength, &length));
  TEST_ASSERT_EQUAL_UINT(kMaxLength + 1, length);
  TEST_ASSERT_FALSE(policy::uploadContentLength("999999999999999999999x", 0, kMaxLength, &length));
  TEST_ASSERT_FALSE(policy::uploadContentLength("999999999999999999999 1", 0, kMaxLength, &length));
}

void test_upload_length_supports_caps_smaller_than_a_digit(void)
{
  size_t length = 123;
  TEST_ASSERT_TRUE(policy::uploadContentLength("9", 9, 8, &length));
  TEST_ASSERT_EQUAL_UINT(9, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("19", 19, 8, &length));
  TEST_ASSERT_EQUAL_UINT(9, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("9", 0, 8, &length));
  TEST_ASSERT_EQUAL_UINT(9, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("1", 1, 0, &length));
  TEST_ASSERT_EQUAL_UINT(1, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("2", 2, 0, &length));
  TEST_ASSERT_EQUAL_UINT(1, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("8", 8, 8, &length));
  TEST_ASSERT_EQUAL_UINT(8, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("0", 0, 0, &length));
  TEST_ASSERT_EQUAL_UINT(0, length);
  TEST_ASSERT_TRUE(policy::uploadContentLength("80", 80, 8, &length));
  TEST_ASSERT_EQUAL_UINT(9, length);
}

void test_stalled_upload_stops_at_idle_deadline(void)
{
  char buffer[4];
  int calls = 0;
  int writes = 0;
  int64_t now = 0;
  const auto result = policy::receiveUpload(
    [&](char*, size_t) { ++calls; now += 1000000; return -3; },
    [&](const char*, size_t) { ++writes; return true; }, [&] { return now; },
    buffer, sizeof(buffer), 8, -3, 5000000, 60000000);
  TEST_ASSERT_EQUAL(policy::UploadResult::Timeout, result);
  TEST_ASSERT_EQUAL_INT(5, calls);
  TEST_ASSERT_EQUAL_INT(0, writes);
}

void test_trickling_upload_cannot_extend_absolute_deadline(void)
{
  char buffer[4];
  int writes = 0;
  int64_t now = 0;
  const auto result = policy::receiveUpload(
    [&](char* dst, size_t) { dst[0] = 'x'; now += 4000000; return 1; },
    [&](const char*, size_t) { ++writes; return true; }, [&] { return now; },
    buffer, sizeof(buffer), 20, -3, 5000000, 60000000);
  TEST_ASSERT_EQUAL(policy::UploadResult::Timeout, result);
  TEST_ASSERT_EQUAL_INT(15, writes);
  TEST_ASSERT_EQUAL_INT64(60000000, now);
}

void test_upload_assembles_bounded_chunks_and_resets_idle_on_progress(void)
{
  char buffer[4];
  const char source[] = "abcdef";
  char written[6] = {};
  size_t received = 0;
  size_t stored = 0;
  int calls = 0;
  int64_t now = 0;
  const auto result = policy::receiveUpload(
    [&](char* dst, size_t size) {
      ++calls;
      now += 2000000;
      if (calls % 2 == 1)
      {
        return -3;
      }
      TEST_ASSERT_EQUAL_UINT(received == 0 ? 4 : 2, size);
      memcpy(dst, source + received, size);
      received += size;
      return static_cast<int>(size);
    },
    [&](const char* data, size_t size) { memcpy(written + stored, data, size); stored += size; return true; },
    [&] { return now; }, buffer, sizeof(buffer), 6, -3, 5000000, 60000000);
  TEST_ASSERT_EQUAL(policy::UploadResult::Ok, result);
  TEST_ASSERT_EQUAL_UINT(6, stored);
  TEST_ASSERT_EQUAL_MEMORY(source, written, 6);
  TEST_ASSERT_EQUAL_INT(4, calls);
}

void test_upload_zero_length_and_expired_deadline_do_not_receive_or_write(void)
{
  char buffer[4];
  int calls = 0;
  int64_t now = 0;
  auto recv = [&](char*, size_t) { ++calls; return 0; };
  auto write = [&](const char*, size_t) { ++calls; return true; };
  TEST_ASSERT_EQUAL(policy::UploadResult::Ok,
    policy::receiveUpload(recv, write, [&] { return now; }, buffer, sizeof(buffer), 0, -3, 5000000, 60000000));
  now = 60000000;
  TEST_ASSERT_EQUAL(policy::UploadResult::Timeout,
    policy::receiveUpload(recv, write, [&] { return now; }, buffer, sizeof(buffer), 0, -3, 5000000, 60000000));
  TEST_ASSERT_EQUAL(policy::UploadResult::Timeout,
    policy::receiveUpload(recv, write, [&] { return now; }, buffer, sizeof(buffer), 4, -3, 5000000, 60000000));
  TEST_ASSERT_EQUAL_INT(0, calls);
}

void test_upload_finishing_at_absolute_deadline_is_still_timed_out(void)
{
  char buffer[4];
  int writes = 0;
  int64_t now = 59000000;
  const auto result = policy::receiveUpload(
    [&](char* dst, size_t) { dst[0] = 'x'; now = 60000000; return 1; },
    [&](const char*, size_t) { ++writes; return true; }, [&] { return now; },
    buffer, sizeof(buffer), 1, -3, 5000000, 60000000);
  TEST_ASSERT_EQUAL(policy::UploadResult::Timeout, result);
  TEST_ASSERT_EQUAL_INT(1, writes);
}

void test_upload_disconnect_and_write_failure_stop_immediately(void)
{
  char buffer[4];
  for (int received : {0, -1, 5})
  {
    int writes = 0;
    TEST_ASSERT_EQUAL(policy::UploadResult::Disconnected,
      policy::receiveUpload([&](char*, size_t) { return received; },
        [&](const char*, size_t) { ++writes; return true; }, [] { return int64_t{0}; },
        buffer, sizeof(buffer), 4, -3, 5000000, 60000000));
    TEST_ASSERT_EQUAL_INT(0, writes);
  }
  int calls = 0;
  TEST_ASSERT_EQUAL(policy::UploadResult::WriteError,
    policy::receiveUpload([&](char*, size_t) { ++calls; return 1; },
      [](const char*, size_t) { return false; }, [] { return int64_t{0}; },
      buffer, sizeof(buffer), 4, -3, 5000000, 60000000));
  TEST_ASSERT_EQUAL_INT(1, calls);
}

void test_json_length_rejects_absent_empty_and_oversized_bodies(void)
{
  TEST_ASSERT_EQUAL(policy::JsonLengthResult::Required, policy::jsonContentLength(0, 2048));
  TEST_ASSERT_EQUAL(policy::JsonLengthResult::Ok, policy::jsonContentLength(1, 2048));
  TEST_ASSERT_EQUAL(policy::JsonLengthResult::Ok, policy::jsonContentLength(2048, 2048));
  TEST_ASSERT_EQUAL(policy::JsonLengthResult::TooLarge, policy::jsonContentLength(2049, 2048));
  TEST_ASSERT_EQUAL(policy::JsonLengthResult::Ok, policy::jsonContentLength(65536, 65536));
  TEST_ASSERT_EQUAL(policy::JsonLengthResult::TooLarge, policy::jsonContentLength(65537, 65536));
}
}  // namespace

extern "C" void run_http_body_policy_tests(void)
{
  RUN_TEST(test_upload_length_distinguishes_missing_from_explicit_zero);
  RUN_TEST(test_upload_length_syntax_and_framing_agreement);
  RUN_TEST(test_upload_length_cap_and_overflow_preserve_error_precedence);
  RUN_TEST(test_upload_length_supports_caps_smaller_than_a_digit);
  RUN_TEST(test_stalled_upload_stops_at_idle_deadline);
  RUN_TEST(test_trickling_upload_cannot_extend_absolute_deadline);
  RUN_TEST(test_upload_assembles_bounded_chunks_and_resets_idle_on_progress);
  RUN_TEST(test_upload_zero_length_and_expired_deadline_do_not_receive_or_write);
  RUN_TEST(test_upload_finishing_at_absolute_deadline_is_still_timed_out);
  RUN_TEST(test_upload_disconnect_and_write_failure_stop_immediately);
  RUN_TEST(test_json_length_rejects_absent_empty_and_oversized_bodies);
}
