// B52: TX-done hook cell. One consistent {fn,arg} pair, clear, replace, set() waits for the running callback.
#include <atomic>
#include <memory>
#include <thread>

#include "../../../components/can_driver/include/hook_cell.hpp"
#include "unity.h"

namespace
{
typedef void (*Fn)(void*);
void fnA(void*) {}
void fnB(void*) {}
int  argA, argB;
auto noSleep = [](uint32_t) {};

typedef HookCell<Fn> Cell;

void test_starts_empty(void)
{
  Cell         c;
  Cell::Guard g(c);
  TEST_ASSERT_NULL(g.fn);
  TEST_ASSERT_NULL(g.arg);
}

void test_set_replace_clear(void)
{
  Cell c;
  TEST_ASSERT_TRUE(c.set(fnA, &argA, 100, 10, noSleep));
  {
    Cell::Guard g(c);
    TEST_ASSERT_TRUE(g.fn == fnA);
    TEST_ASSERT_TRUE(g.arg == &argA);
  }
  TEST_ASSERT_TRUE(c.set(fnB, &argB, 100, 10, noSleep));
  {
    Cell::Guard g(c);
    TEST_ASSERT_TRUE(g.fn == fnB);
    TEST_ASSERT_TRUE(g.arg == &argB);
  }
  TEST_ASSERT_TRUE(c.set(nullptr, nullptr, 100, 10, noSleep));
  {
    Cell::Guard g(c);
    TEST_ASSERT_NULL(g.fn);
    TEST_ASSERT_NULL(g.arg);
  }
}

void test_set_waits_for_running_callback(void)
{
  Cell c;
  TEST_ASSERT_TRUE(c.set(fnA, &argA, 100, 10, noSleep));
  auto reader = std::make_unique<Cell::Guard>(c);  // a callback "in flight" on the old pair
  int  sleeps = 0;
  const bool ok = c.set(fnB, &argB, 1000, 10, [&](uint32_t) {
    if (++sleeps == 3) reader.reset();
  });
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_GREATER_OR_EQUAL_INT(3, sleeps);
  Cell::Guard g(c);
  TEST_ASSERT_TRUE(g.fn == fnB);
}

void test_set_times_out_while_callback_runs(void)
{
  Cell c;
  TEST_ASSERT_TRUE(c.set(fnA, &argA, 100, 10, noSleep));
  Cell::Guard reader(c);
  int         sleeps = 0;
  TEST_ASSERT_FALSE(c.set(fnB, &argB, 50, 10, [&](uint32_t) { ++sleeps; }));
  TEST_ASSERT_EQUAL_INT(5, sleeps);
  // the new pair is live, a later set can still finish once the reader is gone
}

// Writer alternates two pairs, two readers check fn and arg always belong together.
void test_pair_is_consistent_under_concurrent_set(void)
{
  Cell              c;
  std::atomic<bool> stop{false};
  std::atomic<int>  torn{0};
  std::atomic<long> seen{0};
  TEST_ASSERT_TRUE(c.set(fnA, &argA, 100, 10, noSleep));
  auto reader = [&]() {
    while (!stop.load())
    {
      Cell::Guard g(c);
      if (g.fn == nullptr) continue;  // lost race with a set(): allowed
      seen++;
      if ((g.fn == fnA) != (g.arg == &argA)) torn++;
    }
  };
  std::thread t1(reader), t2(reader);
  for (int i = 0; i < 20000; i++)
  {
    const bool a = (i & 1) == 0;
    c.set(a ? fnB : fnA, a ? &argB : &argA, 5000, 1, [](uint32_t) { std::this_thread::yield(); });
  }
  stop.store(true);
  t1.join();
  t2.join();
  TEST_ASSERT_EQUAL_INT(0, torn.load());
  TEST_ASSERT_GREATER_THAN_INT(0, (int)seen.load());
}
}  // namespace

extern "C" void run_hook_cell_tests(void)
{
  RUN_TEST(test_starts_empty);
  RUN_TEST(test_set_replace_clear);
  RUN_TEST(test_set_waits_for_running_callback);
  RUN_TEST(test_set_times_out_while_callback_runs);
  RUN_TEST(test_pair_is_consistent_under_concurrent_set);
}
