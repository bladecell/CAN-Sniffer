// B51: CanDriver admission gate. Reject after close, drain waits for calls in flight, drain times out.
#include "../../../components/can_driver/include/op_gate.hpp"
#include "unity.h"

namespace
{
void test_closed_until_opened(void)
{
  OpGate g;
  TEST_ASSERT_FALSE(g.enter());
  g.open();
  TEST_ASSERT_TRUE(g.enter());
  TEST_ASSERT_EQUAL_UINT32(1, g.inFlight());
  g.leave();
  TEST_ASSERT_EQUAL_UINT32(0, g.inFlight());
}

void test_rejects_after_close_but_keeps_in_flight(void)
{
  OpGate g;
  g.open();
  TEST_ASSERT_TRUE(g.enter());
  g.close();
  TEST_ASSERT_FALSE(g.enter());
  TEST_ASSERT_EQUAL_UINT32(1, g.inFlight());
  g.leave();
  TEST_ASSERT_EQUAL_UINT32(0, g.inFlight());
  TEST_ASSERT_FALSE(g.enter());
}

void test_drain_waits_for_in_flight(void)
{
  OpGate g;
  g.open();
  TEST_ASSERT_TRUE(g.enter());
  TEST_ASSERT_TRUE(g.enter());
  g.close();
  int sleeps = 0;
  const bool ok = g.drain(100, 10, [&](uint32_t) {
    if (++sleeps == 2) g.leave();
    if (sleeps == 4) g.leave();
  });
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT(4, sleeps);
  TEST_ASSERT_EQUAL_UINT32(0, g.inFlight());
}

void test_drain_times_out_and_leaves_gate_closed(void)
{
  OpGate g;
  g.open();
  TEST_ASSERT_TRUE(g.enter());
  g.close();
  int sleeps = 0;
  TEST_ASSERT_FALSE(g.drain(50, 10, [&](uint32_t) { ++sleeps; }));
  TEST_ASSERT_EQUAL_INT(5, sleeps);
  TEST_ASSERT_EQUAL_UINT32(1, g.inFlight());
  TEST_ASSERT_TRUE(g.closed());
}

void test_drain_idle_returns_at_once(void)
{
  OpGate g;
  g.open();
  g.close();
  int sleeps = 0;
  TEST_ASSERT_TRUE(g.drain(50, 10, [&](uint32_t) { ++sleeps; }));
  TEST_ASSERT_EQUAL_INT(0, sleeps);
}

void test_scope_releases_and_reports_rejection(void)
{
  OpGate g;
  g.open();
  {
    OpGate::Scope a(g);
    TEST_ASSERT_TRUE(a.admitted());
    TEST_ASSERT_EQUAL_UINT32(1, g.inFlight());
  }
  TEST_ASSERT_EQUAL_UINT32(0, g.inFlight());
  g.close();
  {
    OpGate::Scope b(g);
    TEST_ASSERT_FALSE(b.admitted());
  }
  TEST_ASSERT_EQUAL_UINT32(0, g.inFlight());
}

void test_reopen_after_close(void)
{
  OpGate g;
  g.open();
  g.close();
  g.open();
  TEST_ASSERT_TRUE(g.enter());
  g.leave();
}
}  // namespace

extern "C" void run_op_gate_tests(void)
{
  RUN_TEST(test_closed_until_opened);
  RUN_TEST(test_rejects_after_close_but_keeps_in_flight);
  RUN_TEST(test_drain_waits_for_in_flight);
  RUN_TEST(test_drain_times_out_and_leaves_gate_closed);
  RUN_TEST(test_drain_idle_returns_at_once);
  RUN_TEST(test_scope_releases_and_reports_rejection);
  RUN_TEST(test_reopen_after_close);
}
