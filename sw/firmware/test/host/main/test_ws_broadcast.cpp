#include <cstdlib>
#include <vector>
#include "../../../components/async_web_server/include/ws_broadcast_dispatch.hpp"
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
    unsigned lists = 0;
    bool reject = false;
    bool failAllocation = false;
    bool failList = false;
    bool failSend = false;
    unsigned sendErrors = 0;
    void (*duringQueue)(void*) = nullptr;
    void* queueArg = nullptr;

    void run()
    {
        auto queued = jobs;
        jobs.clear();
        for (auto job : queued) { job.fn(job.arg); }
    }
};

struct Backend
{
    FakeHttpd* httpd;
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
    bool list(int* fds, size_t& count)
    {
        ++httpd->lists;
        if (httpd->failList) { return false; }
        count = count < httpd->clients.size() ? count : httpd->clients.size();
        for (size_t i = 0; i < count; ++i) { fds[i] = httpd->clients[i].fd; }
        return true;
    }
    bool ready(int fd)
    {
        for (auto client : httpd->clients)
        {
            if (client.fd == fd) { return client.upgraded; }
        }
        return false;
    }
    void send(int fd, uint8_t* payload, size_t len)
    {
        if (httpd->failSend) { ++httpd->sendErrors; return; }
        httpd->sent.push_back(fd);
        httpd->bytes.assign(payload, payload + len);
    }
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
    TEST_ASSERT_EQUAL(0, h.lists);
    h.clients = {{4, false}};
    h.run();
    TEST_ASSERT_TRUE(h.sent.empty());
    TEST_ASSERT_EQUAL(1, h.frees);
    TEST_ASSERT_EQUAL(0, d.pending());
}

static void test_execution_selects_current_upgraded_clients()
{
    FakeHttpd h;
    Dispatcher d;
    d.start();
    d.submit({&h}, packet, sizeof(packet));
    h.clients = {{4, true}, {5, true}, {6, false}};
    h.run();
    TEST_ASSERT_EQUAL(2, h.sent.size());
    TEST_ASSERT_EQUAL(4, h.sent[0]);
    TEST_ASSERT_EQUAL(5, h.sent[1]);
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
    for (unsigned i = 0; i < Dispatcher::kMaxPending; ++i)
    {
        TEST_ASSERT_EQUAL(Result::accepted, d.submit({&h}, packet, sizeof(packet)));
    }
    TEST_ASSERT_EQUAL(Result::full, d.submit({&h}, packet, sizeof(packet)));
    TEST_ASSERT_EQUAL(Dispatcher::kMaxPending, h.allocations);
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
    h.failList = true;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_TRUE(h.sent.empty());
    h.failList = false;
    h.failSend = true;
    d.submit({&h}, packet, sizeof(packet));
    h.run();
    TEST_ASSERT_EQUAL(1, h.sendErrors);
    TEST_ASSERT_EQUAL(0, d.pending());
    TEST_ASSERT_EQUAL(h.allocations, h.frees);
}

extern "C" void run_ws_broadcast_tests(void)
{
    RUN_TEST(test_reused_http_fd_does_not_receive_frame);
    RUN_TEST(test_execution_selects_current_upgraded_clients);
    RUN_TEST(test_unchanged_client_receives_owned_copy);
    RUN_TEST(test_disconnected_client_is_not_a_recipient);
    RUN_TEST(test_shutdown_during_submission_keeps_admission_until_cleanup);
    RUN_TEST(test_rejection_and_allocation_failure_release_admission);
    RUN_TEST(test_pending_and_payload_are_bounded);
    RUN_TEST(test_shutdown_drains_ownership_without_sending_and_can_restart);
    RUN_TEST(test_enumeration_and_send_failures_release_ownership);
}
