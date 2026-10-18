// P2.9c: a stalled client must not hold the receive loop forever (PUT /pid_def used to `continue` on every timeout).
#include <cstdint>
#include <cstring>

#include "../../../main/include/body_reader.hpp"
#include "unity.h"

namespace
{
constexpr int kTimeout = -3;
namespace br = body_reader;

void test_reads_whole_body_in_pieces(void)
{
  const char src[] = "0123456789";
  size_t     pos   = 0;
  char       buf[10];
  auto       recv = [&](char* b, size_t n) {
    const size_t k = n < 3 ? n : 3;
    memcpy(b, src + pos, k);
    pos += k;
    return static_cast<int>(k);
  };
  int64_t now = 0;
  TEST_ASSERT_EQUAL(br::Result::Ok, br::readBody(recv, [&] { return now++; }, buf, 10, kTimeout, 1000));
  TEST_ASSERT_EQUAL_MEMORY(src, buf, 10);
}

void test_endless_socket_timeouts_hit_the_deadline(void)
{
  int     calls = 0;
  int64_t now   = 0;
  char    buf[8];
  auto    recv = [&](char*, size_t) {
    calls++;
    now += 1000;  // each socket timeout burns 1 ms
    return kTimeout;
  };
  TEST_ASSERT_EQUAL(br::Result::Timeout, br::readBody(recv, [&] { return now; }, buf, 8, kTimeout, 5000));
  TEST_ASSERT_EQUAL_INT(5, calls);
}

void test_trickle_cannot_extend_the_deadline(void)
{
  int64_t now = 0;
  char    buf[100];
  auto    recv = [&](char* b, size_t) {
    b[0] = 'x';
    now += 400;
    return 1;
  };
  TEST_ASSERT_EQUAL(br::Result::Timeout, br::readBody(recv, [&] { return now; }, buf, 100, kTimeout, 1000));
}

void test_closed_socket_and_overrun_are_errors(void)
{
  char    buf[4];
  int64_t now = 0;
  TEST_ASSERT_EQUAL(br::Result::Error, br::readBody([](char*, size_t) { return 0; }, [&] { return now; }, buf, 4, kTimeout, 1000));
  TEST_ASSERT_EQUAL(br::Result::Error, br::readBody([](char*, size_t) { return -1; }, [&] { return now; }, buf, 4, kTimeout, 1000));
  TEST_ASSERT_EQUAL(br::Result::Error, br::readBody([](char*, size_t n) { return static_cast<int>(n + 1); }, [&] { return now; }, buf, 4, kTimeout, 1000));
}
}  // namespace

extern "C" void run_body_reader_tests(void)
{
  RUN_TEST(test_reads_whole_body_in_pieces);
  RUN_TEST(test_endless_socket_timeouts_hit_the_deadline);
  RUN_TEST(test_trickle_cannot_extend_the_deadline);
  RUN_TEST(test_closed_socket_and_overrun_are_errors);
}
