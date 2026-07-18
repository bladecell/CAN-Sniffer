#pragma once

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class SD_DTC_Database
{
private:
    FILE*             file = nullptr;
    SemaphoreHandle_t lock;
    long              total_records = 0;

    static constexpr size_t RECORD_SIZE = 128;

public:
    SD_DTC_Database()
    {
        lock = xSemaphoreCreateMutex();
    }

    bool begin(const char* vfs_filepath = "/sdcard/config/dtcs.bin")
    {
        file = fopen(vfs_filepath, "rb");
        if (!file)
            return false;

        fseek(file, 0, SEEK_END);
        long file_bytes_len = ftell(file);
        total_records       = file_bytes_len / RECORD_SIZE;

        return total_records > 0;
    }

    bool lookup(const char* target_code, char* out_desc_buffer, size_t buffer_max_len)
    {
        if (!file || total_records == 0)
        {
            strncpy(out_desc_buffer, "DB Not Mounted", buffer_max_len);
            return false;
        }

        xSemaphoreTake(lock, portMAX_DELAY);

        long left  = 0;
        long right = total_records - 1;
        char read_code[6];

        while (left <= right)
        {
            long mid = left + ((right - left) >> 1);

            fseek(file, mid * RECORD_SIZE, SEEK_SET);

            fread(read_code, 1, 6, file);
            read_code[5] = '\0';

            int cmp = strcmp(read_code, target_code);

            if (cmp == 0)
            {
                size_t max_copy = (buffer_max_len - 1 < 122) ? (buffer_max_len - 1) : 122;

                fread(out_desc_buffer, 1, max_copy, file);
                out_desc_buffer[max_copy] = '\0';

                xSemaphoreGive(lock);
                return true;
            }

            if (cmp < 0)
                left = mid + 1;
            else
                right = mid - 1;
        }

        xSemaphoreGive(lock);
        strncpy(out_desc_buffer, "Unknown DTC", buffer_max_len);
        return false;
    }

    ~SD_DTC_Database()
    {
        if (file)
            fclose(file);
        if (lock)
            vSemaphoreDelete(lock);
    }
};