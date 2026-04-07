#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "obd2_utils.hpp"

#define NUMBER_OF_ITEMS 256

class PIDPriorityQueue
{
private:
    PollRequest       heap[NUMBER_OF_ITEMS];  // Adjust size as needed
    int               size = 0;
    SemaphoreHandle_t lock;

    void swap(int i, int j)
    {
        PollRequest t = heap[i];
        heap[i]       = heap[j];
        heap[j]       = t;
    }

public:
    PIDPriorityQueue()
    {
        lock = xSemaphoreCreateMutex();
    }

    float getFillFactor()
    {
        return (float)size / (float)NUMBER_OF_ITEMS;
    }

    int32_t getTopLatency()
    {
        if (size == 0)
            return 0;
        int32_t diff = (int32_t)xTaskGetTickCount() - (int32_t)heap[0].nextWake;
        return (diff > 0) ? diff : 0;
    }

    void clear()
    {
        if (xSemaphoreTake(lock, portMAX_DELAY))
        {
            size = 0;
            xSemaphoreGive(lock);
        }
    }

    void clearRecurring()
    {
        if (xSemaphoreTake(lock, portMAX_DELAY))
        {
            int         newSize = 0;
            PollRequest tempHeap[NUMBER_OF_ITEMS];

            // Keep only the non-recurring (Static/One-Shot) items
            for (int i = 0; i < size; i++)
            {
                if (!heap[i].isRecurring)
                {
                    tempHeap[newSize++] = heap[i];
                }
            }

            // Reset the main heap
            size = 0;
            for (int i = 0; i < newSize; i++)
            {
                this->push(tempHeap[i]);  // Re-pushing handles the heap sorting
            }

            xSemaphoreGive(lock);
        }
    }

    void push(PollRequest req)
    {
        if (size >= NUMBER_OF_ITEMS)
            return;
        xSemaphoreTake(lock, portMAX_DELAY);
        int i   = size++;
        heap[i] = req;
        while (i != 0 && heap[i] < heap[(i - 1) / 2])
        {
            swap(i, (i - 1) / 2);
            i = (i - 1) / 2;
        }
        xSemaphoreGive(lock);
    }

    PollRequest pop()
    {
        xSemaphoreTake(lock, portMAX_DELAY);
        PollRequest root = heap[0];
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
        return root;
    }

    TickType_t getWait()
    {
        if (size == 0)
            return pdMS_TO_TICKS(100);
        TickType_t now = xTaskGetTickCount();
        return (heap[0].nextWake > now) ? (heap[0].nextWake - now) : 0;
    }

    bool isEmpty()
    {
        return size == 0;
    }
};