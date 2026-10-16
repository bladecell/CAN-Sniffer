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

// Each pair's arg is a counter of callbacks currently running on it.
struct Pair { std::atomic<int> inFlight{0}; };
Pair pairA, pairB;
void yieldProbe() { std::this_thread::yield(); }

// Writer alternates two pairs; readers check fn and arg belong together (and the yield between load and count widens the
// window of the recheck), and that once set() returned true no callback is still running on the replaced pair.
void test_pair_is_consistent_under_concurrent_set(void)
{
  Cell              c;
  std::atomic<bool> stop{false};
  std::atomic<int>  torn{0};
  std::atomic<int>  stale{0};
  std::atomic<long> seen{0};
  TEST_ASSERT_TRUE(c.set(fnA, &pairA, 100, 10, noSleep));
  auto reader = [&]() {
    while (!stop.load())
    {
      Cell::Guard g(c, yieldProbe);
      if (g.fn == nullptr) continue;  // lost the race with a set(): allowed
      Pair* p = static_cast<Pair*>(g.arg);
      p->inFlight++;
      seen++;
      if ((g.fn == fnA) != (p == &pairA)) torn++;
      std::this_thread::yield();
      p->inFlight--;
    }
  };
  std::thread t1(reader), t2(reader);
  for (int i = 0; i < 20000; i++)
  {
    const bool toB = (i & 1) == 0;
    Pair&      old = toB ? pairA : pairB;
    const bool ok  = c.set(toB ? fnB : fnA, toB ? &pairB : &pairA, 5000, 1, [](uint32_t) { std::this_thread::yield(); });
    if (ok && old.inFlight.load() != 0) stale++;
  }
  stop.store(true);
  t1.join();
  t2.join();
  TEST_ASSERT_EQUAL_INT(0, torn.load());
  TEST_ASSERT_EQUAL_INT(0, stale.load());
  TEST_ASSERT_GREATER_THAN_INT(0, (int)seen.load());
}

// Deterministic window: a set() lands between the reader's count and its recheck, every try, so the reader gives up
// after kRetries and sees no hook (it never reads a half-written slot).
Cell* gCell;
int   gProbeCalls;
void  racingSetProbe()
{
  gProbeCalls++;
  (void)gCell->set(gProbeCalls & 1 ? fnB : fnA, gProbeCalls & 1 ? &argB : &argA, 0, 1, noSleep);
}

void test_reader_bails_out_after_retries(void)
{
  Cell c;
  TEST_ASSERT_TRUE(c.set(fnA, &argA, 100, 10, noSleep));
  gCell       = &c;
  gProbeCalls = 0;
  {
    Cell::Guard g(c, racingSetProbe);
    TEST_ASSERT_EQUAL_INT(Cell::kRetries, gProbeCalls);
    TEST_ASSERT_NULL(g.fn);
    TEST_ASSERT_NULL(g.arg);
  }
  // the bailed-out reader released its counts: a plain set() goes through
  TEST_ASSERT_TRUE(c.set(fnB, &argB, 100, 10, noSleep));
  Cell::Guard g(c);
  TEST_ASSERT_TRUE(g.fn == fnB);
}

void test_concurrent_set_is_refused(void)
{
  Cell c;
  TEST_ASSERT_TRUE(c.set(fnA, &argA, 100, 10, noSleep));
  auto reader = std::make_unique<Cell::Guard>(c);
  bool inner  = true;
  int  sleeps = 0;
  const bool outer = c.set(fnB, &argB, 1000, 10, [&](uint32_t) {  // outer set() is waiting for the reader
    if (++sleeps == 1)
    {
      inner = c.set(fnA, &argA, 10, 10, noSleep);
      reader.reset();
    }
  });
  TEST_ASSERT_FALSE(inner);
  TEST_ASSERT_TRUE(outer);
  Cell::Guard g(c);
  TEST_ASSERT_TRUE(g.fn == fnB);
}

void test_zero_step_still_times_out(void)
{
  Cell c;
  TEST_ASSERT_TRUE(c.set(fnA, &argA, 100, 10, noSleep));
  Cell::Guard reader(c);
  TEST_ASSERT_FALSE(c.set(fnB, &argB, 3, 0, noSleep));
}
}  // namespace

extern "C" void run_hook_cell_tests(void)
{
  RUN_TEST(test_starts_empty);
  RUN_TEST(test_set_replace_clear);
  RUN_TEST(test_set_waits_for_running_callback);
  RUN_TEST(test_set_times_out_while_callback_runs);
  RUN_TEST(test_pair_is_consistent_under_concurrent_set);
  RUN_TEST(test_reader_bails_out_after_retries);
  RUN_TEST(test_concurrent_set_is_refused);
  RUN_TEST(test_zero_step_still_times_out);
}
