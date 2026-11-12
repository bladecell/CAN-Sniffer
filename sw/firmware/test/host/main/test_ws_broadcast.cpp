#include <cstdlib>
#include <utility>
#include <vector>
#include "../../../components/async_web_server/include/ws_broadcast_dispatch.hpp"
#include "../../../components/async_web_server/include/ws_session.hpp"
#include "unity.h"

struct FakeHttpd
{
    struct Client { int fd; bool upgraded; };
    struct Job { void (*fn)(void*); void* arg; };
    std::vector<Client> clients{{4, true}};
    std::vector<Job> jobs;
    std::vector<int> sent;
    std::vector<uint8_t> bytes;
    unsigned allocations = 0;
    unsigned frees = 0;
    int active = 4;
    int locks = 0;
    bool writable = true;
    uint32_t now = 0;
    bool reject = false;
    bool failAllocation = false;
    bool failSend = false;
    std::vector<int> evicted;
    unsigned sendErrors = 0;
    void (*duringQueue)(void*) = nullptr;
    void* queueArg = nullptr;
    void (*onSend)(void*) = nullptr;
    void* onSendArg = nullptr;

    void run()
    {
        auto queued = jobs;
        jobs.clear();
        for (auto job : queued) { job.fn(job.arg); }
    }
};

struct Backend
{
    FakeHttpd* httpd = nullptr;
    void* allocate(size_t size)
    {
        if (httpd->failAllocation) { return nullptr; }
        void* p = std::malloc(size);
        if (p) { ++httpd->allocations; }
        return p;
    }
    void release(void* p) { ++httpd->frees; std::free(p); }
    bool queue(void (*fn)(void*), void* arg)
    {
        if (httpd->duringQueue) { httpd->duringQueue(httpd->queueArg); }
        if (httpd->reject) { return false; }
        httpd->jobs.push_back({fn, arg});
        return true;
    }
    int active() { return httpd->active; }
    bool writable(int) { return httpd->writable; }
    void lock() { ++httpd->locks; }
    void unlock() { --httpd->locks; }
    uint32_t nowMs() { return httpd->now; }
    bool ready(int fd)
    {
        for (auto client : httpd->clients)
        {
            if (client.fd == fd) { return client.upgraded; }
        }
        return false;
    }
    bool send(int fd, uint8_t* payload, size_t len)
    {
        if (httpd->failSend) { ++httpd->sendErrors; return false; }
        httpd->sent.push_back(fd);
        if (httpd->onSend) { httpd->onSend(httpd->onSendArg); }
        httpd->bytes.assign(payload, payload + len);
        return true;
    }
    void evict(int fd) { httpd->evicted.push_back(fd); }
};

using Dispatcher = ws_broadcast::Dispatcher<Backend>;
using Result = ws_broadcast::Result;
static const uint8_t packet[] = {3, 10, 3};

static void test_reused_http_fd_does_not_receive_frame()
{
    FakeHttpd h;
    Dispatcher d;
    TEST_ASSERT_TRUE(d.start());
    TEST_ASSERT_EQUAL(Result::accepted, d.submit({&h}, packet, sizeof(packet)));
    h.clients = {{4, false}};
    h.run();
    TEST_ASSERT_TRUE(h.sent.empty());
    TEST_ASSERT_EQUAL(1, h.frees);
    TEST_ASSERT_EQUAL(0, d.pending());
}

static void test_only_the_active_client_receives()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    h.clients = {{4, true}, {5, true}, {6, false}};
    h.active = 5;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
    TEST_ASSERT_EQUAL(5, h.sent[0]);
    h.active = -1;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
}

static void test_unchanged_client_receives_owned_copy()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    uint8_t payload[] = {3, 10, 3};
    d.submit({&h}, payload, sizeof(payload));
    payload[0] = 99;
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(packet, h.bytes.data(), sizeof(packet));
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
}

static void test_disconnected_client_is_not_a_recipient()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    d.submit({&h}, packet, sizeof(packet));
    h.clients = {{7, true}};
    h.active = 7;
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
    TEST_ASSERT_EQUAL(7, h.sent[0]);
}

static void stopDuringQueue(void* arg)
{
    auto* d = static_cast<Dispatcher*>(arg);
    d->stop();
    TEST_ASSERT_EQUAL(1, d->pending());
    TEST_ASSERT_FALSE(d->start());
}

static void test_shutdown_during_submission_keeps_admission_until_cleanup()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    h.duringQueue = stopDuringQueue;
    h.queueArg = &d;
    TEST_ASSERT_EQUAL(Result::accepted, d.submit({&h}, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL(1, d.pending());
    h.run();
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_TRUE(h.sent.empty());
    TEST_ASSERT_EQUAL(1, h.frees);
    TEST_ASSERT_TRUE(d.start());
}

static void test_rejection_and_allocation_failure_release_admission()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    h.reject = true;
    TEST_ASSERT_EQUAL(Result::queueFailed, d.submit({&h}, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL(1, h.frees);
    TEST_ASSERT_EQUAL(0, d.pending());
    h.failAllocation = true;
    TEST_ASSERT_EQUAL(Result::noMemory, d.submit({&h}, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_EQUAL(1, h.allocations);
}

static void test_pending_and_payload_are_bounded()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    TEST_ASSERT_EQUAL(Result::invalid, d.submit({&h}, packet, Dispatcher::kMaxPayload + 1));
    TEST_ASSERT_EQUAL(Result::invalid, d.submit({&h}, nullptr, 1));
    TEST_ASSERT_EQUAL(Result::invalid, d.submit({&h}, packet, 0));
    for (unsigned i = 0; i < Dispatcher::kMaxHigh; ++i)
    {
        TEST_ASSERT_EQUAL(Result::accepted, d.submit({&h}, packet, sizeof(packet)));
    }
    TEST_ASSERT_EQUAL(Result::full, d.submit({&h}, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL(Dispatcher::kMaxHigh, h.allocations);
    TEST_ASSERT_EQUAL(Dispatcher::kMaxHigh, d.pending());
    h.run();
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
}

static void test_shutdown_drains_ownership_without_sending_and_can_restart()
{
    FakeHttpd h;
    Dispatcher d;
    TEST_ASSERT_EQUAL(Result::stopped, d.submit({&h}, packet, sizeof(packet)));
    d.start();
    d.submit({&h}, packet, sizeof(packet));
    d.stop();
    TEST_ASSERT_FALSE(d.start());
    TEST_ASSERT_EQUAL(Result::stopped, d.submit({&h}, packet, sizeof(packet)));
    h.run();
    TEST_ASSERT_TRUE(h.sent.empty());
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
    TEST_ASSERT_TRUE(d.start());
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
}

static void test_enumeration_and_send_failures_release_ownership()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    h.active = -1;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_TRUE(h.sent.empty());
    h.active = 4;
    h.failSend = true;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(1, h.sendErrors);
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
}

// R1: a stalled client used to be sent to again by every queued work, each send blocking the HTTPD task.
static void test_failed_send_skips_client_in_queued_work()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    h.failSend = true;
    for (unsigned i = 0; i < Dispatcher::kMaxHigh; ++i) { d.submit({&h}, packet, sizeof(packet)); }
    for (unsigned i = 0; i < 5; ++i) { d.submitLow({&h}, packet, sizeof(packet)); }
    h.run();
    TEST_ASSERT_EQUAL(1, h.sendErrors);
    TEST_ASSERT_TRUE(h.evicted.empty());
    // After the skip window the client is tried again.
    h.now = ws_broadcast::SendHealth::kSkipMs;
    h.failSend = false;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
}

// R1: a socket with a full send buffer is never written to, so the send cannot block.
static void test_unwritable_socket_gets_no_send()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    h.writable = false;
    h.failSend = true;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(0, h.sendErrors);
    TEST_ASSERT_TRUE(h.sent.empty());
    TEST_ASSERT_TRUE(h.evicted.empty());
}

static void failAt(FakeHttpd& h, Dispatcher& d, uint32_t now)
{
    h.now = now;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
}

// R2 / RD1: failures and time together. Three failures in under a second do not close the client.
static void test_evicts_only_after_failures_and_time()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    h.writable = false;
    failAt(h, d, 1000);
    failAt(h, d, 1300);
    failAt(h, d, 1600);
    failAt(h, d, 5900);
    TEST_ASSERT_TRUE(h.evicted.empty());
    failAt(h, d, 6000);  // inside the skip window: not tried, not counted
    TEST_ASSERT_TRUE(h.evicted.empty());
    failAt(h, d, 6200);
    TEST_ASSERT_EQUAL(1, h.evicted.size());
    TEST_ASSERT_EQUAL(4, h.evicted[0]);
}

static void test_one_long_gap_with_two_failures_does_not_evict()
{
    ws_broadcast::SendHealth health;
    TEST_ASSERT_FALSE(health.skip(4, 0));
    TEST_ASSERT_FALSE(health.fail(0));
    TEST_ASSERT_FALSE(health.fail(9000));
    TEST_ASSERT_TRUE(health.fail(9300));
}

static void test_success_or_new_client_clears_failures()
{
    ws_broadcast::SendHealth health;
    health.skip(4, 0);
    health.fail(0);
    health.fail(300);
    health.ok();
    TEST_ASSERT_FALSE(health.skip(4, 301));
    TEST_ASSERT_FALSE(health.fail(6000));
    TEST_ASSERT_FALSE(health.fail(6300));
    TEST_ASSERT_TRUE(health.skip(4, 6400));
    // Another socket starts clean, also across a millisecond counter wrap.
    TEST_ASSERT_FALSE(health.skip(5, 6400));
    TEST_ASSERT_FALSE(health.fail(0xFFFFFFF0u));
    TEST_ASSERT_TRUE(health.skip(5, 0xFFFFFFFFu));
    TEST_ASSERT_TRUE(health.skip(5, 10));
    TEST_ASSERT_FALSE(health.skip(5, 0xFFFFFFF0u + ws_broadcast::SendHealth::kSkipMs));
}

// R4 / RD3: a burst of PID frames costs one queue slot, so status frames still get in and are delivered.
static void test_status_frames_are_delivered_while_the_pid_buffer_is_full()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    const uint8_t pid[] = {2, 18, 7};
    for (size_t i = 0; i < Dispatcher::kLowFrames; ++i)
    {
        TEST_ASSERT_EQUAL(Result::accepted, d.submitLow({&h}, pid, sizeof(pid)));
    }
    TEST_ASSERT_EQUAL(Result::full, d.submitLow({&h}, pid, sizeof(pid)));
    TEST_ASSERT_EQUAL(1, d.pending());
    TEST_ASSERT_EQUAL(1, h.jobs.size());
    for (unsigned i = 0; i < Dispatcher::kMaxHigh; ++i)
    {
        TEST_ASSERT_EQUAL(Result::accepted, d.submit({&h}, packet, sizeof(packet)));
    }
    TEST_ASSERT_EQUAL(Dispatcher::kMaxPending, d.pending());
    h.run();
    TEST_ASSERT_EQUAL(Dispatcher::kLowFrames + Dispatcher::kMaxHigh, h.sent.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(packet, h.bytes.data(), sizeof(packet));
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_EQUAL(0, h.locks);
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
}

static void test_pid_frames_keep_order_and_need_no_allocation()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    for (uint8_t i = 1; i <= 3; ++i)
    {
        const uint8_t pid[] = {2, 18, i};
        d.submitLow({&h}, pid, sizeof(pid));
    }
    TEST_ASSERT_EQUAL(0, h.allocations);
    h.run();
    TEST_ASSERT_EQUAL(3, h.sent.size());
    TEST_ASSERT_EQUAL(3, h.bytes[2]);
    // The buffer is free again and a later frame queues a new drain item.
    const uint8_t next[] = {2, 18, 9};
    TEST_ASSERT_EQUAL(Result::accepted, d.submitLow({&h}, next, sizeof(next)));
    TEST_ASSERT_EQUAL(1, h.jobs.size());
    h.run();
    TEST_ASSERT_EQUAL(9, h.bytes[2]);
    TEST_ASSERT_EQUAL(0, d.pending());
}

static void test_pid_path_bounds_and_failures_leave_nothing_pending()
{
    FakeHttpd h;
    Dispatcher d;
    TEST_ASSERT_EQUAL(Result::stopped, d.submitLow({&h}, packet, sizeof(packet)));
    d.start();
    uint8_t big[Dispatcher::kLowPayload + 1] = {};
    TEST_ASSERT_EQUAL(Result::invalid, d.submitLow({&h}, big, sizeof(big)));
    TEST_ASSERT_EQUAL(Result::invalid, d.submitLow({&h}, nullptr, 1));
    TEST_ASSERT_EQUAL(Result::invalid, d.submitLow({&h}, packet, 0));
    h.reject = true;
    TEST_ASSERT_EQUAL(Result::queueFailed, d.submitLow({&h}, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL(0, d.pending());
    h.reject = false;
    // The frame from the failed attempt is gone, and the path works again.
    TEST_ASSERT_EQUAL(Result::accepted, d.submitLow({&h}, packet, sizeof(packet)));
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
    // Stop with frames waiting: the drain item discards them and releases its count.
    d.submitLow({&h}, packet, sizeof(packet));
    d.stop();
    h.run();
    TEST_ASSERT_EQUAL(1, h.sent.size());
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_TRUE(d.start());
    d.submitLow({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(2, h.sent.size());
    TEST_ASSERT_EQUAL(0, h.locks);
}

static void refill(void* arg)
{
    auto* pair = static_cast<std::pair<Dispatcher*, FakeHttpd*>*>(arg);
    pair->first->submitLow({pair->second}, packet, sizeof(packet));
}

// A producer that keeps the buffer non-empty does not hold the HTTPD task: the drain item re-queues itself.
static void test_drain_yields_to_the_queue_under_a_fast_producer()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    std::pair<Dispatcher*, FakeHttpd*> pair{&d, &h};
    d.submitLow({&h}, packet, sizeof(packet));
    h.onSend = refill;
    h.onSendArg = &pair;
    h.run();
    TEST_ASSERT_EQUAL(1, h.jobs.size());
    TEST_ASSERT_EQUAL(1, d.pending());
    TEST_ASSERT_TRUE(h.sent.size() >= 3 && h.sent.size() <= 8);
    h.onSend = nullptr;
    h.run();
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_TRUE(h.jobs.empty());
}

// RD2 / RD4: one session, newest wins, a different live session is asked first.
static void test_session_admission()
{
    using ws_session::Admit;
    ws_session::Holder none;
    TEST_ASSERT_EQUAL(Admit::accept, ws_session::admit(none, 0, 0xAB, false));
    ws_session::Holder held{7, 0xAB, 1000};
    TEST_ASSERT_EQUAL(Admit::busy, ws_session::admit(held, 2000, 0xCD, false));
    TEST_ASSERT_EQUAL(Admit::replace, ws_session::admit(held, 2000, 0xCD, true));
    TEST_ASSERT_EQUAL(Admit::replace, ws_session::admit(held, 2000, 0xAB, false));  // the same page reconnecting
    TEST_ASSERT_EQUAL(Admit::replace, ws_session::admit(held, 2000, 0, false));     // no session id
    TEST_ASSERT_EQUAL(Admit::busy, ws_session::admit(held, 1000 + ws_session::kStaleMs - 1, 0xCD, false));
    TEST_ASSERT_EQUAL(Admit::replace, ws_session::admit(held, 1000 + ws_session::kStaleMs, 0xCD, false));
    ws_session::Holder wrapped{7, 0xAB, 0xFFFFFF00u};
    TEST_ASSERT_EQUAL(Admit::busy, ws_session::admit(wrapped, 0x100, 0xCD, false));
}

static void test_session_id_parse_rejects_bad_input()
{
    TEST_ASSERT_EQUAL_HEX32(0x1A2B3C4D, ws_session::parseSid("1a2B3c4D"));
    TEST_ASSERT_EQUAL_HEX32(0xF, ws_session::parseSid("f"));
    TEST_ASSERT_EQUAL(0, ws_session::parseSid(nullptr));
    TEST_ASSERT_EQUAL(0, ws_session::parseSid(""));
    TEST_ASSERT_EQUAL(0, ws_session::parseSid("123456789"));
    TEST_ASSERT_EQUAL(0, ws_session::parseSid("12g4"));
    TEST_ASSERT_EQUAL(0, ws_session::parseSid("-1"));
    TEST_ASSERT_EQUAL(0, ws_session::parseSid(" 12"));
    TEST_ASSERT_EQUAL(0, ws_session::parseSid("0x12"));
}

// R5: the close reply is empty or a status code, never a length without a payload.
static void test_close_payload_is_empty_or_a_status_code()
{
    uint8_t out[2] = {0xEE, 0xEE};
    TEST_ASSERT_EQUAL(0, ws_session::closePayload(0, out));
    TEST_ASSERT_EQUAL(2, ws_session::closePayload(ws_session::kCloseInUse, out));
    TEST_ASSERT_EQUAL_HEX8(0x0F, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA0, out[1]);
    TEST_ASSERT_EQUAL(2, ws_session::closePayload(ws_session::kCloseReplaced, out));
    TEST_ASSERT_EQUAL_HEX8(0xA1, out[1]);
}

static void test_successful_send_does_not_evict()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_TRUE(h.evicted.empty());
}

static void test_drop_log_is_rate_limited_and_counts_suppressed()
{
    ws_broadcast::DropLog log;
    unsigned suppressed = 99;
    TEST_ASSERT_TRUE(log.shouldLog(1000, suppressed));
    TEST_ASSERT_EQUAL(0, suppressed);
    TEST_ASSERT_FALSE(log.shouldLog(1500, suppressed));
    TEST_ASSERT_FALSE(log.shouldLog(5999, suppressed));
    TEST_ASSERT_TRUE(log.shouldLog(6000, suppressed));
    TEST_ASSERT_EQUAL(2, suppressed);
    TEST_ASSERT_FALSE(log.shouldLog(6001, suppressed));
    TEST_ASSERT_TRUE(log.shouldLog(11000, suppressed));
    TEST_ASSERT_EQUAL(1, suppressed);
}

extern "C" void run_ws_broadcast_tests(void)
{
    RUN_TEST(test_failed_send_skips_client_in_queued_work);
    RUN_TEST(test_unwritable_socket_gets_no_send);
    RUN_TEST(test_evicts_only_after_failures_and_time);
    RUN_TEST(test_one_long_gap_with_two_failures_does_not_evict);
    RUN_TEST(test_success_or_new_client_clears_failures);
    RUN_TEST(test_status_frames_are_delivered_while_the_pid_buffer_is_full);
    RUN_TEST(test_pid_frames_keep_order_and_need_no_allocation);
    RUN_TEST(test_pid_path_bounds_and_failures_leave_nothing_pending);
    RUN_TEST(test_drain_yields_to_the_queue_under_a_fast_producer);
    RUN_TEST(test_session_admission);
    RUN_TEST(test_session_id_parse_rejects_bad_input);
    RUN_TEST(test_close_payload_is_empty_or_a_status_code);
    RUN_TEST(test_successful_send_does_not_evict);
    RUN_TEST(test_drop_log_is_rate_limited_and_counts_suppressed);
    RUN_TEST(test_reused_http_fd_does_not_receive_frame);
    RUN_TEST(test_only_the_active_client_receives);
    RUN_TEST(test_unchanged_client_receives_owned_copy);
    RUN_TEST(test_disconnected_client_is_not_a_recipient);
    RUN_TEST(test_shutdown_during_submission_keeps_admission_until_cleanup);
    RUN_TEST(test_rejection_and_allocation_failure_release_admission);
    RUN_TEST(test_pending_and_payload_are_bounded);
    RUN_TEST(test_shutdown_drains_ownership_without_sending_and_can_restart);
    RUN_TEST(test_enumeration_and_send_failures_release_ownership);
}
