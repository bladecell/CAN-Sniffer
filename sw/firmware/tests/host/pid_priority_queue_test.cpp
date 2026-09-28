#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

#include "pid_priority_queue.hpp"

static TickType_t fakeNow = 0;
static int notificationCount = 0;
static TaskHandle_t lastNotified = nullptr;
TickType_t xTaskGetTickCount() { return fakeNow; }
void xTaskNotifyGive(TaskHandle_t task) { ++notificationCount; lastNotified = task; }
SemaphoreHandle_t xSemaphoreCreateMutex() { return new int(0); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }

static PollRequest request(uint16_t pid, TickType_t wake, bool recurring = true)
{
    PollRequest r{};
    r.nextWake = wake;
    r.payload.obd.pid = pid;
    r.payload.obd.mode = MODE_CURRENT_DATA;
    r.isRecurring = recurring;
    r.isRaw = false;
    return r;
}
static PollRequest rawRequest(uint8_t byte, TickType_t wake)
{
    PollRequest r{};
    r.nextWake = wake;
    r.isRaw = true;
    r.payload.raw.data[0] = byte;
    return r;
}

int main()
{
    PIDPriorityQueue q;
    PollRequest out{};
    fakeNow = 100;
    assert(q.push(request(1, 110)));
    assert(q.getWait() == 10);
    assert(!q.tryPopDue(out));
    fakeNow = 110;
    assert(q.tryPopDue(out) && out.payload.obd.pid == 1);

    int consumer;
    const int n0 = notificationCount;
    q.setConsumerTask(&consumer);
    assert(q.getConsumerTask() == &consumer);
    assert(notificationCount == n0 + 1 && lastNotified == &consumer);
    assert(q.push(request(2, 200)));
    assert(lastNotified == &consumer);
    const int beforeEarlierInsert = notificationCount;
    assert(q.push(request(8, 180)));
    assert(notificationCount == beforeEarlierInsert + 1);
    assert(q.tryPopDue(out) == false);
    // Remove the earlier head using compatibility pop; the next item remains future.
    assert(q.tryPop(out) && out.payload.obd.pid == 8);
    fakeNow = 120;
    assert(!q.tryPopDue(out));
    // Replacing a deadline after the consumer wakes must not permit early pop.
    PollRequest later = request(2, 220);
    assert(q.replaceRecurringPidRange(2, 2, MODE_CURRENT_DATA, &later, 1));
    assert(!q.tryPopDue(out));
    fakeNow = 220;
    assert(q.tryPopDue(out));

    // Modular ordering and wait/latency across tick rollover.
    fakeNow = UINT32_MAX - 4;
    assert(q.push(request(3, 3)));
    assert(q.getWait() == 8);
    assert(!q.tryPopDue(out));
    fakeNow = 3;
    assert(q.getTopLatency() == 0);
    assert(q.tryPopDue(out));
    fakeNow = 5;
    assert(q.push(request(4, UINT32_MAX - 2)));
    assert(q.tryPopDue(out));

    // removePID removes every definition-bound match but retains raw payload.
    assert(q.push(request(7, 10, false)));
    assert(q.push(request(7, 11, true)));
    assert(q.push(rawRequest(7, 12)));
    q.removePID(7);
    assert(q.getFillFactor() > 0);
    assert(q.tryPop(out) && out.isRaw && out.payload.raw.data[0] == 7);
    assert(q.isEmpty());

    // Failed whole-queue replacement is transactional.
    std::vector<PollRequest> full;
    for (int i = 0; i < NUMBER_OF_ITEMS; ++i) full.push_back(rawRequest((uint8_t)i, 100 + i));
    assert(q.replaceRecurring(full.data(), full.size()));
    PollRequest addition = request(99, 0);
    assert(!q.replaceRecurring(&addition, 1));
    assert(q.getFillFactor() == 1.0f);
    assert(q.tryPop(out) && out.isRaw && out.payload.raw.data[0] == 0);
    q.clear();

    assert(q.replaceRecurring(full.data(), full.size()));
    assert(!q.replaceRecurringPidRange(90, 99, MODE_CURRENT_DATA, &addition, 1));
    assert(q.getFillFactor() == 1.0f);
    assert(q.tryPop(out) && out.isRaw && out.payload.raw.data[0] == 0);
    q.clear();

    // Range reconciliation retains raw and unrelated work, refreshes target,
    // and removes target PIDs omitted by the replacement set.
    assert(q.push(rawRequest(0xA5, 40)));
    assert(q.push(request(10, 41)));
    assert(q.push(request(11, 42)));
    assert(q.push(request(20, 43)));
    PollRequest desired[] = {request(10, 50)};
    desired[0].priority = 2;
    assert(q.replaceRecurringPidRange(10, 11, MODE_CURRENT_DATA, desired, 1));
    assert(q.getFillFactor() == 3.0f / NUMBER_OF_ITEMS);
    assert(q.tryPop(out) && out.payload.raw.data[0] == 0xA5);
    assert(q.tryPop(out) && out.payload.obd.pid == 10 && out.priority == 2);
    assert(q.tryPop(out) && out.payload.obd.pid == 20);
    assert(q.isEmpty());

    q.push(request(30, 60));
    const int beforeDetach = notificationCount;
    q.setConsumerTask(nullptr);
    assert(q.getConsumerTask() == nullptr);
    const int afterDetach = notificationCount;
    assert(afterDetach == beforeDetach);
    assert(q.push(request(31, 61)));
    assert(notificationCount == afterDetach);
    std::cout << "PIDPriorityQueue host regressions passed\n";
}
