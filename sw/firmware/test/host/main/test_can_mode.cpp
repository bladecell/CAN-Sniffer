// can_mode: the mode table and its controller (no ESP-IDF, fake environment).
#include <cstring>
#include <string>
#include <vector>

#include "../../../../main/include/can_mode.hpp"
#include "../../../../main/include/can_mode_http.hpp"
#include "cJSON.h"
#include "unity.h"

using namespace can_mode;

namespace
{
struct FakeEnv
{
  bool running  = true;
  bool recorder = false;
  bool poll     = false;
  bool failSet  = false;
  std::vector<bool> sets;
  int  fcDisarms = 0;
  bool inflight  = false;                 // a REST OBD request is on the bus
  bool parsing   = true;                  // what the rx task was last told
  bool healthPing = true;                 // whether OBD2 may send its health-check ping
  void setHealthPing(bool on) { healthPing = on; }
  std::vector<bool> parsingLog;
  bool obdRequestInFlight() { return inflight; }
  void setObdParsing(bool on)
  {
    parsing = on;
    parsingLog.push_back(on);
  }
  bool recorderStartsWhenPollingStops = false;  // models Recorder::start finishing between the check and the switch


  bool supervisorRunning() { return running; }
  bool recorderBusy() { return recorder; }
  bool autoPoll() { return poll; }
  bool setAutoPoll(bool on)
  {
    sets.push_back(on);
    if (failSet) { return false; }
    poll = on;
    if (!on && recorderStartsWhenPollingStops) { recorder = true; }
    return true;
  }
  void fcDisarm() { fcDisarms++; }
};
using Ctl = Controller<FakeEnv>;

int st(Status s) { return static_cast<int>(s); }
}  // namespace

static void test_starts_in_monitor_and_allows_no_raw_send(void)
{
  FakeEnv env;
  Ctl     c(env);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_FALSE(c.allows(kOpRawSend));
  TEST_ASSERT_TRUE(c.allows(kOpPollOn));
  TEST_ASSERT_FALSE(c.fc());
  TEST_ASSERT_EQUAL(2, Ctl::count());
  TEST_ASSERT_EQUAL_STRING("monitor", Ctl::nameAt(0));
  TEST_ASSERT_EQUAL_STRING("raw", Ctl::nameAt(1));
  TEST_ASSERT_NULL(Ctl::nameAt(2));
}

static void test_raw_turns_polling_off_and_leaving_restores_it(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  TEST_ASSERT_EQUAL(st(Status::ok), st(c.set("raw", true, 1000)));
  TEST_ASSERT_EQUAL_STRING("raw", c.name());
  TEST_ASSERT_TRUE(c.allows(kOpRawSend));
  TEST_ASSERT_TRUE(c.fc());
  TEST_ASSERT_FALSE(env.poll);
  TEST_ASSERT_EQUAL(st(Status::ok), st(c.set("monitor", false, 2000)));
  TEST_ASSERT_TRUE(env.poll);
  TEST_ASSERT_FALSE(c.allows(kOpRawSend));
  TEST_ASSERT_FALSE(c.fc());
  TEST_ASSERT_EQUAL(1, env.fcDisarms);
}

static void test_raw_mode_does_not_allow_turning_polling_on(void)
{
  FakeEnv env;
  Ctl     c(env);
  c.set("raw", false, 0);
  TEST_ASSERT_FALSE(c.allows(kOpPollOn));
  TEST_ASSERT_TRUE(c.allows(kOpRawSend));
  c.set("monitor", false, 1);
  TEST_ASSERT_TRUE(c.allows(kOpPollOn));
  TEST_ASSERT_FALSE(c.allows(kOpRawSend));
}

// ---- OBD parsing switch (published to the rx task through Env::setObdParsing) ----

static void test_raw_does_not_parse_unless_asked(void)
{
  FakeEnv env;
  Ctl     c(env);
  TEST_ASSERT_TRUE(env.parsing);
  TEST_ASSERT_EQUAL(st(Status::ok), st(c.set("raw", false, false, 0)));
  TEST_ASSERT_FALSE(env.parsing);
  TEST_ASSERT_FALSE(c.parsing());
  TEST_ASSERT_EQUAL(st(Status::ok), st(c.set("raw", false, true, 1)));  // changed while staying in Raw: immediate
  TEST_ASSERT_TRUE(env.parsing);
  TEST_ASSERT_TRUE(c.parsing());
  c.set("raw", false, false, 2);
  TEST_ASSERT_FALSE(env.parsing);
}

static void test_the_three_argument_set_means_no_parsing(void)
{
  FakeEnv env;
  Ctl     c(env);
  c.set("raw", true, 0);
  TEST_ASSERT_FALSE(env.parsing);
  TEST_ASSERT_TRUE(c.fc());  // flow control is its own switch
}

static void test_leaving_raw_turns_parsing_back_on_and_forgets_the_option(void)
{
  FakeEnv env;
  Ctl     c(env);
  c.set("raw", true, true, 0);
  c.set("monitor", false, false, 1);
  TEST_ASSERT_TRUE(env.parsing);
  TEST_ASSERT_FALSE(c.parsing());
  c.set("raw", false, 2);
  TEST_ASSERT_FALSE(c.parsing());  // not remembered from the earlier session
  c.set("monitor", true, true, 3);  // parsing is meaningless in Monitor: stored false, rx parses anyway
  TEST_ASSERT_TRUE(env.parsing);
  TEST_ASSERT_FALSE(c.parsing());
}

static void test_every_revert_path_publishes_parsing_on(void)
{
  FakeEnv env;
  Ctl     c(env);
  c.set("raw", false, 0);
  c.reset(true);
  TEST_ASSERT_TRUE(env.parsing);
  c.set("raw", false, 0);
  c.reset(false);
  TEST_ASSERT_TRUE(env.parsing);
  c.set("raw", false, 0);
  c.deferReset(true);
  c.tick(1);
  TEST_ASSERT_TRUE(env.parsing);
  c.set("raw", false, 0);
  env.running = false;
  c.tick(2);
  TEST_ASSERT_TRUE(env.parsing);
  env.running = true;
  c.set("raw", false, 0);
  c.tick(kIdleTimeoutMs);
  TEST_ASSERT_TRUE(env.parsing);
}

static void test_raw_suspends_the_health_ping_and_every_revert_resumes_it(void)
{
  FakeEnv env;
  Ctl     c(env);
  TEST_ASSERT_TRUE(env.healthPing);
  c.set("raw", false, 0);
  TEST_ASSERT_FALSE(env.healthPing);
  c.set("raw", true, true, 1);  // options change inside Raw: still no ping
  TEST_ASSERT_FALSE(env.healthPing);
  c.set("monitor", false, 2);
  TEST_ASSERT_TRUE(env.healthPing);
  const char* paths[] = {"reset", "resetNoRestore", "deferred", "supervisor", "idle"};
  for (const char* path : paths)
  {
    env.running = true;
    c.set("raw", false, 10);
    TEST_ASSERT_FALSE_MESSAGE(env.healthPing, path);
    const std::string p(path);
    if (p == "reset") { c.reset(true); }
    else if (p == "resetNoRestore") { c.reset(false); }
    else if (p == "deferred") { c.deferReset(true); c.tick(11); }
    else if (p == "supervisor") { env.running = false; c.tick(11); }
    else { c.tick(10 + kIdleTimeoutMs); }
    TEST_ASSERT_TRUE_MESSAGE(env.healthPing, path);
  }
}

static void test_a_refused_raw_entry_keeps_the_health_ping(void)
{
  FakeEnv env;
  env.recorder = true;
  Ctl c(env);
  c.set("raw", false, 0);
  TEST_ASSERT_TRUE(env.healthPing);
  env.recorder = false;
  env.inflight = true;
  c.set("raw", false, 0);
  TEST_ASSERT_TRUE(env.healthPing);
}

static void test_a_refused_raw_entry_never_stops_the_parser(void)
{
  FakeEnv env;
  env.running = false;
  Ctl c(env);
  c.set("raw", false, 0);
  env.running  = true;
  env.recorder = true;
  c.set("raw", false, 0);
  env.recorder = false;
  env.poll     = true;
  env.failSet  = true;
  c.set("raw", false, 0);
  env.failSet = false;
  env.recorderStartsWhenPollingStops = true;
  c.set("raw", false, 0);
  for (const bool on : env.parsingLog) { TEST_ASSERT_TRUE(on); }
  TEST_ASSERT_TRUE(env.parsing);
}

static void test_raw_is_refused_while_a_rest_obd_request_is_on_the_bus(void)
{
  FakeEnv env;
  env.poll     = true;
  env.inflight = true;
  Ctl c(env);
  TEST_ASSERT_EQUAL(st(Status::busy), st(c.set("raw", false, 0)));
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.sets.empty());
  TEST_ASSERT_TRUE(env.parsing);
  env.inflight = false;
  TEST_ASSERT_EQUAL(st(Status::ok), st(c.set("raw", false, 1)));
}

// ---- lock-free view (what GET /can/mode reports) must follow every transition ----

static void test_view_follows_every_transition_and_revert_path(void)
{
  // Regression of e0095c2: the view was refreshed only by some paths, so after the last WebSocket closed GET still said "raw".
  FakeEnv env;
  Ctl     c(env);
  TEST_ASSERT_EQUAL(0, c.view().mode);
  TEST_ASSERT_FALSE(c.view().fc);
  c.set("raw", true, true, 0);
  TEST_ASSERT_EQUAL_STRING("raw", Ctl::nameAt(c.view().mode));
  TEST_ASSERT_TRUE(c.view().fc);
  TEST_ASSERT_TRUE(c.view().parsing);
  c.set("raw", false, false, 1);  // changed while staying in Raw
  TEST_ASSERT_FALSE(c.view().fc);
  TEST_ASSERT_FALSE(c.view().parsing);
  c.reset(true);  // last WebSocket client closed
  TEST_ASSERT_EQUAL(0, c.view().mode);
  c.set("raw", true, true, 2);
  c.reset(false);
  TEST_ASSERT_EQUAL(0, c.view().mode);
  c.set("raw", true, true, 3);
  c.deferReset(true);  // close hook could not take the lock
  c.tick(4);
  TEST_ASSERT_EQUAL(0, c.view().mode);
  TEST_ASSERT_FALSE(c.view().fc);
  c.set("raw", false, 5);
  c.tick(5 + kIdleTimeoutMs);  // idle timeout
  TEST_ASSERT_EQUAL(0, c.view().mode);
  c.set("raw", false, 6);
  env.running = false;  // supervisor left RUNNING
  c.tick(7);
  TEST_ASSERT_EQUAL(0, c.view().mode);
}

static void test_a_refused_raw_entry_leaves_the_view_in_monitor(void)
{
  FakeEnv env;
  env.recorder = true;
  Ctl c(env);
  c.set("raw", false, 0);
  TEST_ASSERT_EQUAL(0, c.view().mode);
}

static void test_a_leaked_request_counter_cannot_block_the_revert(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", true, 0);
  env.inflight = true;  // a counter that was never decremented
  c.reset(true);
  TEST_ASSERT_EQUAL(0, c.view().mode);
  TEST_ASSERT_TRUE(env.poll);
  c.set("raw", false, 1);  // (entry is refused while it is stuck, but the revert paths do not look at it)
  TEST_ASSERT_EQUAL(0, c.view().mode);
  env.inflight = false;
  c.set("raw", false, 2);
  env.inflight = true;
  c.deferReset(true);
  c.tick(3);
  TEST_ASSERT_EQUAL(0, c.view().mode);
  c.set("raw", false, 4);
  env.inflight = true;
  c.tick(4 + kIdleTimeoutMs);
  TEST_ASSERT_EQUAL(0, c.view().mode);
}

static void test_polling_that_was_off_stays_off_and_is_never_touched(void)
{
  FakeEnv env;
  Ctl     c(env);
  c.set("raw", false, 0);
  c.set("monitor", false, 10);
  TEST_ASSERT_TRUE(env.sets.empty());
  TEST_ASSERT_FALSE(env.poll);
}

static void test_raw_is_refused_unless_the_supervisor_runs(void)
{
  FakeEnv env;
  env.running = false;
  env.poll    = true;
  Ctl c(env);
  TEST_ASSERT_EQUAL(st(Status::notRunning), st(c.set("raw", false, 0)));
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.sets.empty());
  TEST_ASSERT_TRUE(env.poll);
}

static void test_raw_is_refused_while_recording(void)
{
  FakeEnv env;
  env.recorder = true;
  env.poll     = true;
  Ctl c(env);
  TEST_ASSERT_EQUAL(st(Status::recording), st(c.set("raw", false, 0)));
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.sets.empty());
}

static void test_polling_that_cannot_be_stopped_keeps_monitor(void)
{
  FakeEnv env;
  env.poll    = true;
  env.failSet = true;
  Ctl c(env);
  TEST_ASSERT_EQUAL(st(Status::busy), st(c.set("raw", false, 0)));
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.poll);
}

static void test_hostile_mode_names_are_refused(void)
{
  FakeEnv env;
  Ctl     c(env);
  const char* bad[] = {nullptr, "", "Raw", "RAW", "raw ", " raw", "monitor\n", "sniffer", "rawraw", "0"};
  for (const char* b : bad)
  {
    TEST_ASSERT_EQUAL(st(Status::unknownMode), st(c.set(b, false, 0)));
  }
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
}

static void test_raw_again_only_updates_fc_and_keeps_the_saved_polling(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  TEST_ASSERT_FALSE(c.fc());
  TEST_ASSERT_EQUAL(st(Status::ok), st(c.set("raw", true, 5)));
  TEST_ASSERT_TRUE(c.fc());
  TEST_ASSERT_EQUAL(1, env.sets.size());  // polling was switched off once
  c.set("monitor", false, 6);
  TEST_ASSERT_TRUE(env.poll);
}

static void test_turning_fc_off_while_staying_in_raw_disarms_the_responder(void)
{
  FakeEnv env;
  Ctl     c(env);
  c.set("raw", true, 0);
  TEST_ASSERT_EQUAL(0, env.fcDisarms);
  c.set("raw", true, 1);  // unchanged: stays armed
  TEST_ASSERT_EQUAL(0, env.fcDisarms);
  c.set("raw", false, 2);
  TEST_ASSERT_EQUAL(1, env.fcDisarms);
  TEST_ASSERT_FALSE(c.fc());
  c.set("raw", false, 3);  // already off: nothing more to disarm
  TEST_ASSERT_EQUAL(1, env.fcDisarms);
}

static void test_polling_turned_on_by_someone_else_in_raw_is_left_alone(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  env.poll = true;  // the page or another client turned it on again
  env.sets.clear();
  c.set("monitor", false, 1);
  TEST_ASSERT_TRUE(env.sets.empty());
}

static void test_idle_timeout_reverts_and_activity_extends_it(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 1000);
  c.tick(1000 + kIdleTimeoutMs - 1);
  TEST_ASSERT_EQUAL_STRING("raw", c.name());
  c.touch(1000 + kIdleTimeoutMs - 1);
  c.tick(1000 + kIdleTimeoutMs + 5000);
  TEST_ASSERT_EQUAL_STRING("raw", c.name());
  c.tick(1000 + kIdleTimeoutMs - 1 + kIdleTimeoutMs);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.poll);
}

static void test_idle_timeout_is_clock_wrap_safe(void)
{
  FakeEnv env;
  Ctl     c(env);
  c.set("raw", false, 0xFFFFFF00u);
  c.tick(0x100u);
  TEST_ASSERT_EQUAL_STRING("raw", c.name());
  c.tick(0xFFFFFF00u + kIdleTimeoutMs);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
}

static void test_supervisor_leaving_running_reverts_and_restores_polling_best_effort(void)
{
  // RUNNING -> NOT_CONNECTED (bus drop) keeps OBD2 alive: polling must come back.
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", true, 0);
  env.sets.clear();
  env.running = false;
  c.tick(10);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_EQUAL(1, env.sets.size());
  TEST_ASSERT_TRUE(env.sets[0]);
  TEST_ASSERT_TRUE(env.poll);
  TEST_ASSERT_EQUAL(1, env.fcDisarms);
  TEST_ASSERT_FALSE(c.fc());
}

static void test_supervisor_leaving_running_with_obd_down_leaves_no_pending_work(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  env.running = false;
  env.failSet = true;  // OBD2 stopped: the restore is refused
  c.tick(10);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_FALSE(c.needsTick());  // the refused restore is forgotten, not applied later behind OBD2's back
  env.running = true;
  env.failSet = false;
  env.sets.clear();
  c.tick(2000);
  TEST_ASSERT_TRUE(env.sets.empty());
}

static void test_recording_that_starts_during_the_switch_rolls_raw_back(void)
{
  FakeEnv env;
  env.poll                          = true;
  env.recorderStartsWhenPollingStops = true;
  Ctl c(env);
  TEST_ASSERT_EQUAL(st(Status::recording), st(c.set("raw", false, 0)));
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.poll);  // the recording keeps its polling
  TEST_ASSERT_FALSE(c.needsTick());
}

static void test_needs_tick_while_raw_or_a_restore_is_pending(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  TEST_ASSERT_FALSE(c.needsTick());
  c.set("raw", false, 0);
  TEST_ASSERT_TRUE(c.needsTick());
  env.failSet = true;
  c.reset(true);
  TEST_ASSERT_TRUE(c.needsTick());  // restore pending
  env.failSet = false;
  c.tick(5);
  TEST_ASSERT_FALSE(c.needsTick());
  TEST_ASSERT_TRUE(env.poll);
}

static void test_idle_timeout_needs_nothing_but_tick(void)
{
  // No WebSocket anywhere in this model: the periodic tick alone ends an idle Raw mode.
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", true, 0);
  for (uint32_t t = 1000; t < kIdleTimeoutMs; t += 1000) { c.tick(t); }
  TEST_ASSERT_EQUAL_STRING("raw", c.name());
  c.tick(kIdleTimeoutMs);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.poll);
  TEST_ASSERT_FALSE(c.needsTick());
}

static void test_deferred_reset_is_applied_by_the_next_tick(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  c.deferReset(true);  // the caller could not get the lock
  TEST_ASSERT_EQUAL_STRING("raw", c.name());
  c.tick(10);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.poll);
}

static void test_deferred_reset_runs_before_the_next_mode_change(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", true, 0);
  c.deferReset(true);
  TEST_ASSERT_EQUAL(st(Status::ok), st(c.set("raw", false, 5)));  // reverted first, then entered again
  TEST_ASSERT_EQUAL_STRING("raw", c.name());
  TEST_ASSERT_FALSE(c.fc());
  TEST_ASSERT_FALSE(env.poll);
  c.set("monitor", false, 6);
  TEST_ASSERT_TRUE(env.poll);
}

static void test_deferred_reset_without_restore_leaves_polling_alone(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  env.sets.clear();
  c.deferReset(false);
  c.tick(10);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_TRUE(env.sets.empty());
  c.deferReset(true);  // nothing to revert: harmless
  c.tick(20);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
}

static void test_reset_with_and_without_restore(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  c.reset(true);
  TEST_ASSERT_TRUE(env.poll);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  c.set("raw", false, 0);
  env.sets.clear();
  c.reset(false);
  TEST_ASSERT_TRUE(env.sets.empty());
  TEST_ASSERT_FALSE(env.poll);
  c.reset(true);  // already monitor: nothing happens
  c.reset(false);
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
}

static void test_failed_restore_is_retried_by_tick_and_survives_a_new_raw_entry(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  env.failSet = true;
  c.reset(true);  // restore fails
  TEST_ASSERT_EQUAL_STRING("monitor", c.name());
  TEST_ASSERT_FALSE(env.poll);
  c.tick(1000);  // still failing
  TEST_ASSERT_FALSE(env.poll);
  env.failSet = false;
  c.tick(2000);
  TEST_ASSERT_TRUE(env.poll);

  // pending restore, then raw again: the user's original "on" must not be forgotten
  FakeEnv env2;
  env2.poll = true;
  Ctl c2(env2);
  c2.set("raw", false, 0);
  env2.failSet = true;
  c2.reset(true);
  env2.failSet = false;
  TEST_ASSERT_EQUAL(st(Status::ok), st(c2.set("raw", false, 5)));
  c2.set("monitor", false, 6);
  TEST_ASSERT_TRUE(env2.poll);
}

static void test_pending_restore_is_dropped_when_the_supervisor_is_not_running(void)
{
  FakeEnv env;
  env.poll = true;
  Ctl c(env);
  c.set("raw", false, 0);
  env.failSet = true;
  c.reset(true);
  env.running = false;
  env.failSet = false;
  env.sets.clear();
  c.tick(10);
  TEST_ASSERT_TRUE(env.sets.empty());
  env.running = true;
  c.tick(20);
  TEST_ASSERT_TRUE(env.sets.empty());  // forgotten, not applied later behind OBD2's back
}

// ---- POST /can/mode body and status mapping ----

namespace
{
bool body(const char* json, const char*& mode, bool& fc, bool* parsingOut = nullptr)
{
  cJSON* root = cJSON_Parse(json);
  static char keep[16];
  const char* m = nullptr;
  bool        f = true;
  bool        p = true;
  const bool  ok = can_mode_http::parseBody(root, m, f, p);
  if (parsingOut != nullptr) { *parsingOut = p; }
  if (ok && m != nullptr) { std::strncpy(keep, m, sizeof(keep) - 1); keep[sizeof(keep) - 1] = 0; mode = keep; } else { mode = nullptr; }
  fc = f;
  cJSON_Delete(root);
  return ok;
}
}  // namespace

static void test_mode_body_accepts_mode_and_optional_fc(void)
{
  const char* m = nullptr;
  bool        fc = true;
  TEST_ASSERT_TRUE(body("{\"mode\":\"raw\"}", m, fc));
  TEST_ASSERT_EQUAL_STRING("raw", m);
  TEST_ASSERT_FALSE(fc);  // default
  TEST_ASSERT_TRUE(body("{\"mode\":\"raw\",\"isotp_flow_control\":true}", m, fc));
  TEST_ASSERT_TRUE(fc);
  TEST_ASSERT_TRUE(body("{\"isotp_flow_control\":false,\"mode\":\"monitor\"}", m, fc));
  TEST_ASSERT_EQUAL_STRING("monitor", m);
}

static void test_mode_body_rejects_everything_else(void)
{
  const char* m = nullptr;
  bool        fc = false;
  const char* bad[] = {"[]", "null", "\"raw\"", "{}", "{\"isotp_flow_control\":true}", "{\"mode\":1}", "{\"mode\":null}",
                       "{\"mode\":\"raw\",\"isotp_flow_control\":1}", "{\"mode\":\"raw\",\"fc\":true}", "{\"mode\":\"raw\",\"fc\":false}", "{\"mode\":\"raw\",\"isotp_flow_control\":\"true\"}",
                       "{\"mode\":\"raw\",\"extra\":1}", "{\"mode\":\"raw\",\"mode\":\"monitor\"}"};
  for (const char* b : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(body(b, m, fc), b);
  }
  bool p = false;
  TEST_ASSERT_FALSE(can_mode_http::parseBody(nullptr, m, fc, p));
}

static void test_mode_body_obd_parsing_option(void)
{
  const char* m  = nullptr;
  bool        fc = true, parsing = true;
  TEST_ASSERT_TRUE(body("{\"mode\":\"raw\"}", m, fc, &parsing));
  TEST_ASSERT_FALSE(parsing);  // default off
  TEST_ASSERT_TRUE(body("{\"mode\":\"raw\",\"obd_parsing\":true}", m, fc, &parsing));
  TEST_ASSERT_TRUE(parsing);
  TEST_ASSERT_FALSE(fc);  // the two switches are independent
  TEST_ASSERT_TRUE(body("{\"obd_parsing\":true,\"isotp_flow_control\":true,\"mode\":\"raw\"}", m, fc, &parsing));
  TEST_ASSERT_TRUE(parsing && fc);
  const char* bad[] = {"{\"mode\":\"raw\",\"obd_parsing\":1}", "{\"mode\":\"raw\",\"obd_parsing\":\"true\"}",
                       "{\"mode\":\"raw\",\"obd_parsing\":null}", "{\"mode\":\"raw\",\"obd_parsing\":true,\"obd_parsing\":false}",
                       "{\"mode\":\"raw\",\"parsing\":true}"};
  for (const char* b : bad)
  {
    TEST_ASSERT_FALSE_MESSAGE(body(b, m, fc, &parsing), b);
  }
}

static void test_status_mapping(void)
{
  TEST_ASSERT_EQUAL(200, can_mode_http::httpStatus(Status::ok));
  TEST_ASSERT_EQUAL(422, can_mode_http::httpStatus(Status::unknownMode));
  TEST_ASSERT_EQUAL(503, can_mode_http::httpStatus(Status::notRunning));
  TEST_ASSERT_EQUAL(409, can_mode_http::httpStatus(Status::recording));
  TEST_ASSERT_EQUAL(503, can_mode_http::httpStatus(Status::busy));
  TEST_ASSERT_EQUAL_STRING("BAD_MODE", can_mode_http::code(Status::unknownMode));
  TEST_ASSERT_EQUAL_STRING("NOT_RUNNING", can_mode_http::code(Status::notRunning));
  TEST_ASSERT_EQUAL_STRING("RECORDING", can_mode_http::code(Status::recording));
  TEST_ASSERT_EQUAL_STRING("BUSY", can_mode_http::code(Status::busy));
}

extern "C" void run_can_mode_tests(void)
{
  RUN_TEST(test_starts_in_monitor_and_allows_no_raw_send);
  RUN_TEST(test_raw_turns_polling_off_and_leaving_restores_it);
  RUN_TEST(test_raw_mode_does_not_allow_turning_polling_on);
  RUN_TEST(test_raw_does_not_parse_unless_asked);
  RUN_TEST(test_the_three_argument_set_means_no_parsing);
  RUN_TEST(test_leaving_raw_turns_parsing_back_on_and_forgets_the_option);
  RUN_TEST(test_every_revert_path_publishes_parsing_on);
  RUN_TEST(test_raw_suspends_the_health_ping_and_every_revert_resumes_it);
  RUN_TEST(test_a_refused_raw_entry_keeps_the_health_ping);
  RUN_TEST(test_a_refused_raw_entry_never_stops_the_parser);
  RUN_TEST(test_raw_is_refused_while_a_rest_obd_request_is_on_the_bus);
  RUN_TEST(test_view_follows_every_transition_and_revert_path);
  RUN_TEST(test_a_refused_raw_entry_leaves_the_view_in_monitor);
  RUN_TEST(test_a_leaked_request_counter_cannot_block_the_revert);
  RUN_TEST(test_polling_that_was_off_stays_off_and_is_never_touched);
  RUN_TEST(test_raw_is_refused_unless_the_supervisor_runs);
  RUN_TEST(test_raw_is_refused_while_recording);
  RUN_TEST(test_polling_that_cannot_be_stopped_keeps_monitor);
  RUN_TEST(test_hostile_mode_names_are_refused);
  RUN_TEST(test_raw_again_only_updates_fc_and_keeps_the_saved_polling);
  RUN_TEST(test_turning_fc_off_while_staying_in_raw_disarms_the_responder);
  RUN_TEST(test_polling_turned_on_by_someone_else_in_raw_is_left_alone);
  RUN_TEST(test_idle_timeout_reverts_and_activity_extends_it);
  RUN_TEST(test_idle_timeout_is_clock_wrap_safe);
  RUN_TEST(test_supervisor_leaving_running_reverts_and_restores_polling_best_effort);
  RUN_TEST(test_supervisor_leaving_running_with_obd_down_leaves_no_pending_work);
  RUN_TEST(test_recording_that_starts_during_the_switch_rolls_raw_back);
  RUN_TEST(test_needs_tick_while_raw_or_a_restore_is_pending);
  RUN_TEST(test_idle_timeout_needs_nothing_but_tick);
  RUN_TEST(test_deferred_reset_is_applied_by_the_next_tick);
  RUN_TEST(test_deferred_reset_runs_before_the_next_mode_change);
  RUN_TEST(test_deferred_reset_without_restore_leaves_polling_alone);
  RUN_TEST(test_reset_with_and_without_restore);
  RUN_TEST(test_failed_restore_is_retried_by_tick_and_survives_a_new_raw_entry);
  RUN_TEST(test_pending_restore_is_dropped_when_the_supervisor_is_not_running);
  RUN_TEST(test_mode_body_accepts_mode_and_optional_fc);
  RUN_TEST(test_mode_body_rejects_everything_else);
  RUN_TEST(test_mode_body_obd_parsing_option);
  RUN_TEST(test_status_mapping);
}
