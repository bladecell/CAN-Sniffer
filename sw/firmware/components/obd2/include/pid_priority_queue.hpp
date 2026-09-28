#pragma once

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "obd2_common.hpp"

#define NUMBER_OF_ITEMS 256

class PIDPriorityQueue
{
private:
    PollRequest       heap[NUMBER_OF_ITEMS];  // Adjust size as needed
    int               size = 0;
    SemaphoreHandle_t lock;
    TaskHandle_t      attachedConsumerTask = nullptr;

    void notifyConsumerLocked()
    {
        if (attachedConsumerTask != nullptr)
            xTaskNotifyGive(attachedConsumerTask);
    }

    void swap(int i, int j)
    {
        PollRequest t = heap[i];
        heap[i]       = heap[j];
        heap[j]       = t;
    }

public:
    // Deprecated compatibility mirror only. Callers must migrate to
    // setConsumerTask()/getConsumerTask(); direct writes are not synchronized.
    TaskHandle_t consumerTask = nullptr;
    PIDPriorityQueue()
    {
        lock = xSemaphoreCreateMutex();
    }

    void setConsumerTask(TaskHandle_t task)
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        attachedConsumerTask = task;
        consumerTask = task; // compatibility mirror; notification uses attachedConsumerTask
        if (task != nullptr)
            notifyConsumerLocked();
        xSemaphoreGive(lock);
    }

    TaskHandle_t getConsumerTask()
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        TaskHandle_t task = attachedConsumerTask;
        xSemaphoreGive(lock);
        return task;
    }

    float getFillFactor()
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        float result = (float)size / (float)NUMBER_OF_ITEMS;
        xSemaphoreGive(lock);
        return result;
    }

    int32_t getTopLatency()
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        if (size == 0)
        {
            xSemaphoreGive(lock);
            return 0;
        }
        TickType_t diff = tickDeadlineLatency(heap[0].nextWake, xTaskGetTickCount());
        xSemaphoreGive(lock);
        return static_cast<int32_t>(diff);
    }

    void clear()
    {
        if (xSemaphoreTake(lock, portMAX_DELAY))
        {
            size = 0;
            notifyConsumerLocked();
            xSemaphoreGive(lock);
        }
    }

    void clearRecurring()
    {
        if (xSemaphoreTake(lock, portMAX_DELAY))
        {
            int newSize = 0;
            for (int i = 0; i < size; i++)
            {
                if (!heap[i].isRecurring)
                    heap[newSize++] = heap[i];
            }
            size = newSize;
            heapify();
            notifyConsumerLocked();

            xSemaphoreGive(lock);
        }
    }

    bool push(PollRequest req)
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        if (size >= NUMBER_OF_ITEMS)
        {
            xSemaphoreGive(lock);
            return false;
        }
        int i   = size++;
        heap[i] = req;
        while (i != 0 && heap[i] < heap[(i - 1) / 2])
        {
            swap(i, (i - 1) / 2);
            i = (i - 1) / 2;
        }
        notifyConsumerLocked();
        xSemaphoreGive(lock);
        return true;
    }

    bool tryPop(PollRequest& root)
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        if (size == 0)
        {
            xSemaphoreGive(lock);
            return false;
        }
        root             = heap[0];
        heap[0]          = heap[--size];
        int i            = 0;
        while (true)
        {
            int small = i, l = 2 * i + 1, r = 2 * i + 2;
            if (l < size && heap[l] < heap[small])
                small = l;
            if (r < size && heap[r] < heap[small])
                small = r;
            if (small != i)
            {
                swap(i, small);
                i = small;
            }
            else
                break;
        }
        xSemaphoreGive(lock);
        return true;
    }

    bool tryPopDue(PollRequest& root)
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        if (size == 0 || !tickDeadlineDue(heap[0].nextWake, xTaskGetTickCount()))
        {
            xSemaphoreGive(lock);
            return false;
        }
        root = heap[0];
        heap[0] = heap[--size];
        siftDown(0);
        xSemaphoreGive(lock);
        return true;
    }

    PollRequest pop()
    {
        PollRequest root = {};
        tryPop(root);
        return root;
    }

    // Replace all definition-bound work while preserving raw operational
    // requests. The capacity check and replacement happen under one lock so
    // callers can use this as the queue half of a model transaction.
    bool replaceRecurring(const PollRequest* requests, size_t requestCount)
    {
        if (requests == nullptr && requestCount != 0)
            return false;

        if (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE)
            return false;

        size_t rawRequests = 0;
        for (int i = 0; i < size; ++i)
        {
            if (heap[i].isRaw)
                ++rawRequests;
        }

        if (rawRequests + requestCount > NUMBER_OF_ITEMS)
        {
            xSemaphoreGive(lock);
            return false;
        }

        // Compact in place. Writes only move an already-visited item toward
        // the front, so no temporary heap-sized buffer is needed.
        size_t nextSize = 0;
        for (int i = 0; i < size; ++i)
        {
            if (heap[i].isRaw)
                heap[nextSize++] = heap[i];
        }
        for (size_t i = 0; i < requestCount; ++i)
            heap[nextSize++] = requests[i];
        size = (int)nextSize;
        heapify();
        notifyConsumerLocked();
        xSemaphoreGive(lock);
        return true;
    }

    // Reconcile recurring Mode-1 work for one supported-PID group without
    // allocating. Raw discovery requests and unrelated work remain queued.
    bool replaceRecurringPidRange(uint16_t firstPid, uint16_t lastPid, uint8_t mode, const PollRequest* requests,
                                  size_t requestCount)
    {
        if (requests == nullptr && requestCount != 0)
            return false;

        if (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE)
            return false;

        size_t retained = 0;
        for (int i = 0; i < size; ++i)
        {
            bool duplicate = false;
            if (!heap[i].isRaw && heap[i].isRecurring && heap[i].payload.obd.mode == mode &&
                heap[i].payload.obd.pid >= firstPid &&
                heap[i].payload.obd.pid <= lastPid)
            {
                for (int j = 0; j < i; ++j)
                {
                    if (!heap[j].isRaw && heap[j].isRecurring && heap[j].payload.obd.mode == mode &&
                        heap[j].payload.obd.pid == heap[i].payload.obd.pid)
                    {
                        duplicate = true;
                        break;
                    }
                }
            }
            bool desired = false;
            for (size_t j = 0; !heap[i].isRaw && j < requestCount; ++j)
            {
                if (requests[j].payload.obd.mode == mode && requests[j].payload.obd.pid == heap[i].payload.obd.pid)
                {
                    desired = true;
                    break;
                }
            }
            const bool remove = !heap[i].isRaw && heap[i].isRecurring && heap[i].payload.obd.mode == mode &&
                                heap[i].payload.obd.pid >= firstPid && heap[i].payload.obd.pid <= lastPid &&
                                (!desired || duplicate);
            if (!remove)
                ++retained;
        }

        size_t additions = 0;
        for (size_t i = 0; i < requestCount; ++i)
        {
            bool present = false;
            for (int j = 0; j < size; ++j)
            {
                if (!heap[j].isRaw && heap[j].isRecurring && heap[j].payload.obd.mode == mode &&
                    heap[j].payload.obd.pid == requests[i].payload.obd.pid)
                {
                    present = true;
                    break;
                }
            }
            if (!present)
                ++additions;
        }

        if (retained + additions > NUMBER_OF_ITEMS)
        {
            xSemaphoreGive(lock);
            return false;
        }

        // Refresh already scheduled request parameters only after the
        // capacity check has succeeded.
        for (int i = 0; i < size; ++i)
        {
            if (heap[i].isRaw || !heap[i].isRecurring || heap[i].payload.obd.mode != mode ||
                heap[i].payload.obd.pid < firstPid ||
                heap[i].payload.obd.pid > lastPid)
                continue;
            for (size_t j = 0; j < requestCount; ++j)
            {
                if (requests[j].payload.obd.mode == mode && requests[j].payload.obd.pid == heap[i].payload.obd.pid)
                {
                    heap[i].id                = requests[j].id;
                    heap[i].payload.obd.mode  = requests[j].payload.obd.mode;
                    heap[i].payload.obd.len   = requests[j].payload.obd.len;
                    heap[i].interval          = requests[j].interval;
                    heap[i].priority          = requests[j].priority;
                    break;
                }
            }
        }

        size_t nextSize = 0;
        for (int i = 0; i < size; ++i)
        {
            bool duplicate = false;
            if (!heap[i].isRaw && heap[i].isRecurring && heap[i].payload.obd.mode == mode &&
                heap[i].payload.obd.pid >= firstPid &&
                heap[i].payload.obd.pid <= lastPid)
            {
                for (size_t j = 0; j < nextSize; ++j)
                {
                    if (!heap[j].isRaw && heap[j].isRecurring && heap[j].payload.obd.mode == mode &&
                        heap[j].payload.obd.pid == heap[i].payload.obd.pid)
                    {
                        duplicate = true;
                        break;
                    }
                }
            }
            bool desired = false;
            for (size_t j = 0; j < requestCount; ++j)
            {
                if (requests[j].payload.obd.mode == mode && requests[j].payload.obd.pid == heap[i].payload.obd.pid)
                {
                    desired = true;
                    break;
                }
            }
            const bool remove = !heap[i].isRaw && heap[i].isRecurring && heap[i].payload.obd.mode == mode &&
                                heap[i].payload.obd.pid >= firstPid && heap[i].payload.obd.pid <= lastPid &&
                                (!desired || duplicate);
            if (!remove)
                heap[nextSize++] = heap[i];
        }
        for (size_t i = 0; i < requestCount; ++i)
        {
            bool present = false;
            for (size_t j = 0; j < nextSize; ++j)
            {
                if (!heap[j].isRaw && heap[j].isRecurring && heap[j].payload.obd.mode == mode &&
                    heap[j].payload.obd.pid == requests[i].payload.obd.pid)
                {
                    present = true;
                    break;
                }
            }
            if (!present)
                heap[nextSize++] = requests[i];
        }
        size = (int)nextSize;
        heapify();
        if (additions != 0 || requestCount != 0)
            notifyConsumerLocked();
        xSemaphoreGive(lock);
        return true;
    }

    // Remove definition-bound recurring work for one mode while retaining raw
    // operational work and all other modes.
    void clearRecurringMode(uint8_t mode)
    {
        if (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE)
            return;
        size_t nextSize = 0;
        for (int i = 0; i < size; ++i)
        {
            if (!heap[i].isRaw && heap[i].isRecurring && heap[i].payload.obd.mode == mode)
                continue;
            heap[nextSize++] = heap[i];
        }
        size = (int)nextSize;
        heapify();
        notifyConsumerLocked();
        xSemaphoreGive(lock);
    }

    void removePID(uint16_t targetPid)
    {
        if (xSemaphoreTake(lock, portMAX_DELAY))
        {
            int nextSize = 0;
            for (int i = 0; i < size; i++)
            {
                if (heap[i].isRaw || heap[i].payload.obd.pid != targetPid)
                    heap[nextSize++] = heap[i];
            }
            if (nextSize != size)
            {
                size = nextSize;
                heapify();
                notifyConsumerLocked();
            }
            xSemaphoreGive(lock);
        }
    }

    TickType_t getWait()
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        if (size == 0)
        {
            xSemaphoreGive(lock);
            return pdMS_TO_TICKS(100);
        }
        TickType_t now = xTaskGetTickCount();
        TickType_t result = ticksUntilDeadline(heap[0].nextWake, now);
        xSemaphoreGive(lock);
        return result;
    }

    bool isEmpty()
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        bool result = size == 0;
        xSemaphoreGive(lock);
        return result;
    }

private:
    void heapify()
    {
        for (int i = size / 2 - 1; i >= 0; --i)
        {
            int current = i;
            while (true)
            {
                int small = current, left = current * 2 + 1, right = current * 2 + 2;
                if (left < size && heap[left] < heap[small])
                    small = left;
                if (right < size && heap[right] < heap[small])
                    small = right;
                if (small == current)
                    break;
                swap(current, small);
                current = small;
            }
        }
    }

    void siftDown(int i)
    {
        while (true)
        {
            int small = i, l = 2 * i + 1, r = 2 * i + 2;
            if (l < size && heap[l] < heap[small]) small = l;
            if (r < size && heap[r] < heap[small]) small = r;
            if (small == i) return;
            swap(i, small);
            i = small;
        }
    }
};
