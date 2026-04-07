// obd2_simulator.cpp
#include "obd2_simulator.hpp"

#include <math.h>

#include "esp_random.h"

static const char* TAG = "SIM_TASK";

TaskHandle_t xDataSimTaskHandle = NULL;

void start_sim_task(CanDriver* driver)
{
    // Using xTaskCreate for automatic core selection (Floating Task)
    BaseType_t result = xTaskCreatePinnedToCore(dataSimTaskWrapper, "DataSimTask", 4096, (void*)driver,
                                                tskIDLE_PRIORITY + 2, &xDataSimTaskHandle, CORE_ID_CAN_TASKS);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create DataSimTask!");
    }
}

void stop_sim_task()
{
    if (xDataSimTaskHandle != NULL)
    {
        vTaskDelete(xDataSimTaskHandle);

        xDataSimTaskHandle = NULL;
    }
}

void dataSimTaskWrapper(void* param)
{
    CanDriver* driver = static_cast<CanDriver*>(param);
    dataSimTask(*driver);
}

void dataSimTask(CanDriver& canDriver)
{
    uint32_t              sd            = 0;
    uint8_t               requested_pid = 0, mode = 0;
    static const char     vin[] = "1HGBH41JXMN109186";
    std::vector<uint16_t> dtcs  = {0x0143, 0x8260, 0x4234, 0x0300, 0x0408, 0x0506, 0x0101, 0x0113, 0x0335, 0x0401};
    // uint32_t DTCs_to_send = (esp_random() % 10) + 1;

    while (1)
    {
        // Block until notified with a PID request
        if (xTaskNotifyWait(0x00, ULONG_MAX, &sd, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        requested_pid = sd & 0xFF;
        mode          = (sd >> 8) & 0xFF;

        switch (mode)
        {
            case MODE_CURRENT_DATA:
            {
                CanDriver::CanFrame response_frame = {};

                // Response ID for ECU 0 (0x7E8)
                response_frame.header.id = 0x7E8;
                response_frame.data[1]   = RESPONSE_CURRENT_DATA;  // Response Mode (0x01 + 0x40)
                response_frame.data[2]   = requested_pid;
                response_frame.length    = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                uint32_t t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                uint32_t val;
                switch (requested_pid)
                {
                    case PID_PIDS_SUPPORTED_1_20:
                    case PID_PIDS_SUPPORTED_21_40:
                    case PID_PIDS_SUPPORTED_41_60:
                    case PID_PIDS_SUPPORTED_61_80:
                    case PID_PIDS_SUPPORTED_81_A0:
                    case PID_PIDS_SUPPORTED_A1_C0:
                    case PID_PIDS_SUPPORTED_C1_E0:
                        response_frame.data[0] = 0x06;  // Length 6 (4 data bytes)
                        response_frame.data[3] = 0xFF;  // A
                        response_frame.data[4] = 0xFF;  // B
                        response_frame.data[5] = 0xFF;  // C
                        response_frame.data[6] = 0xFF;  // D
                        break;
                    case PID_ENGINE_RPM:
                        val                    = sinDataSim(t_ms, 0.07f, 65536);
                        response_frame.data[0] = 0x04;
                        response_frame.data[3] = (val >> 8) & 0xFF;
                        response_frame.data[4] = (val >> 16) & 0xFF;
                        break;

                    case PID_ENGINE_LOAD:
                        val                    = sinDataSim(t_ms, 0.03f, 255);
                        response_frame.data[0] = 0x03;
                        response_frame.data[3] = val;
                        response_frame.data[4] = 0x00;
                        break;

                    case PID_COOLANT_TEMP:
                        val                    = sinDataSim(t_ms, 0.001f, 255);
                        response_frame.data[0] = 0x03;
                        response_frame.data[3] = val;
                        response_frame.data[4] = 0x00;
                        break;

                    default:
                        ESP_LOGW(TAG, "PID 0x%02lX not simulated.", requested_pid);
                        continue;  // Ignore unknown PIDs
                }

                // Inject the response frame into the RX queue
                if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Failed to inject simulated frame.");
                }
                break;
            }
            case MODE_CLEAR_DTCS:
            {
                CanDriver::CanFrame response_frame = {};
                response_frame.header.id           = 0x7E8;
                response_frame.data[0]             = 0x01;
                response_frame.data[1]             = RESPONSE_CLEAR_DTCS;  // Response Mode (0x04 + 0x40)
                response_frame.length              = PID_DATA_LENGTH;      // Max length for CAN 2.0A

                if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Failed to inject simulated frame.");
                }
                break;
            }
            case MODE_DTCS:
            case MODE_PENDING_DTCS:
            case MODE_PERMANENT_DTCS:
            {
                uint32_t DTCs_to_send = (esp_random() % 10) + 1;
                if (DTCs_to_send < 3)
                {
                    CanDriver::CanFrame response_frame = {};
                    response_frame.header.id           = 0x7E8;
                    response_frame.data[0]             = DTCs_to_send > 1 ? 0x07 : 0x05;
                    response_frame.data[1]             = mode | 0x40;
                    response_frame.data[2]             = (uint8_t)DTCs_to_send;
                    response_frame.data[3]             = (dtcs[0] >> 8) & 0xFF;
                    response_frame.data[4]             = dtcs[0] & 0xFF;
                    response_frame.data[5]             = DTCs_to_send > 1 ? (dtcs[1] >> 8) & 0xFF : 0x00;
                    response_frame.data[6]             = DTCs_to_send > 1 ? dtcs[1] & 0xFF : 0x00;
                    response_frame.data[7]             = 0x00;
                    response_frame.length              = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                    if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "Failed to inject simulated frame.");
                        continue;
                    }
                }
                else
                {
                    std::vector<uint8_t> bytes_to_send;
                    for (int i = 0; i < DTCs_to_send; i++)
                    {
                        bytes_to_send.push_back((dtcs[i] >> 8) & 0xFF);
                        bytes_to_send.push_back(dtcs[i] & 0xFF);
                    }
                    uint16_t            total_length   = bytes_to_send.size() + 2;
                    CanDriver::CanFrame response_frame = {};
                    response_frame.header.id           = 0x7E8;
                    response_frame.data[0]             = 0x10 | ((uint8_t)((total_length >> 8) & 0x0F));
                    response_frame.data[1]             = (uint8_t)(total_length & 0xFF);
                    response_frame.data[2]             = mode | 0x40;
                    response_frame.data[3]             = (uint8_t)DTCs_to_send;
                    response_frame.data[4]             = bytes_to_send[0];
                    response_frame.data[5]             = bytes_to_send[1];
                    response_frame.data[6]             = bytes_to_send[2];
                    response_frame.data[7]             = bytes_to_send[3];
                    response_frame.length              = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                    if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "Failed to inject simulated frame.");
                        continue;
                    }

                    TickType_t stop_tick = xTaskGetTickCount() + pdMS_TO_TICKS(200);
                    bool       found     = false;
                    while (stop_tick > xTaskGetTickCount())
                    {
                        if (xTaskNotifyWait(0x00, ULONG_MAX, &sd, stop_tick - xTaskGetTickCount()) != pdPASS)
                        {
                            break;
                        }

                        if (sd == 0x0000)
                        {
                            found = true;
                        }
                    }
                    if (!found)
                        continue;

                    uint8_t bts_idx = 4;
                    uint8_t cf_idx  = 1;

                    while (bts_idx < bytes_to_send.size())
                    {
                        CanDriver::CanFrame response_frame = {};
                        response_frame.header.id           = 0x7E8;
                        response_frame.data[0]             = 0x20 + cf_idx++;
                        for (int j = 1; (bts_idx < bytes_to_send.size()) && (j <= 7); j++)
                        {
                            response_frame.data[j] = bytes_to_send[bts_idx++];
                        }

                        response_frame.length = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                        if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                        {
                            ESP_LOGW(TAG, "Failed to inject simulated frame.");
                            continue;
                        }
                    }
                }
                break;
            }
            case MODE_VEHICLE_INFO:
            {
                if (requested_pid == PID_VIN)  // VIN with multiframe
                {
                    CanDriver::CanFrame response_frame = {};
                    response_frame.header.id           = 0x7E8;
                    response_frame.data[0]             = 0x10;
                    response_frame.data[1]             = 0x14;
                    response_frame.data[2]             = 0x49;
                    response_frame.data[3]             = (uint8_t)requested_pid;
                    response_frame.data[4]             = 0x01;
                    response_frame.data[5]             = vin[0];
                    response_frame.data[6]             = vin[1];
                    response_frame.data[7]             = vin[2];
                    response_frame.length              = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                    if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "Failed to inject simulated frame.");
                        continue;
                    }

                    TickType_t stop_tick = xTaskGetTickCount() + pdMS_TO_TICKS(200);
                    bool       found     = false;
                    while (stop_tick > xTaskGetTickCount())
                    {
                        if (xTaskNotifyWait(0x00, ULONG_MAX, &sd, stop_tick - xTaskGetTickCount()) != pdPASS)
                        {
                            break;
                        }

                        if (sd == 0x0000)
                        {
                            found = true;
                        }
                    }
                    if (!found)
                        continue;

                    CanDriver::CanFrame consecutiveFrame1 = {};
                    consecutiveFrame1.header.id           = 0x7E8;
                    consecutiveFrame1.data[0]             = 0x21;  // Consecutive Frame
                    consecutiveFrame1.data[1]             = vin[3];
                    consecutiveFrame1.data[2]             = vin[4];
                    consecutiveFrame1.data[3]             = vin[5];
                    consecutiveFrame1.data[4]             = vin[6];
                    consecutiveFrame1.data[5]             = vin[7];
                    consecutiveFrame1.data[6]             = vin[8];
                    consecutiveFrame1.data[7]             = vin[9];
                    consecutiveFrame1.length              = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                    if (xQueueSend(canDriver.getRxQueueHandle(), &consecutiveFrame1, 0) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "Failed to inject simulated frame.");
                    }

                    CanDriver::CanFrame consecutiveFrame2 = {};
                    consecutiveFrame2.header.id           = 0x7E8;
                    consecutiveFrame2.data[0]             = 0x22;  // Consecutive Frame
                    consecutiveFrame2.data[1]             = vin[10];
                    consecutiveFrame2.data[2]             = vin[11];
                    consecutiveFrame2.data[3]             = vin[12];
                    consecutiveFrame2.data[4]             = vin[13];
                    consecutiveFrame2.data[5]             = vin[14];
                    consecutiveFrame2.data[6]             = vin[15];
                    consecutiveFrame1.data[7]             = vin[16];
                    consecutiveFrame2.length              = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                    if (xQueueSend(canDriver.getRxQueueHandle(), &consecutiveFrame2, 0) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "Failed to inject simulated frame.");
                    }
                    continue;
                }
                break;
            }
            default:
                ESP_LOGW(TAG, "Mode %u not simulated", mode);
                break;
        }
    }
}

uint32_t sinDataSim(uint32_t t_ms, float frequency, uint32_t size)
{
    float omega     = 2 * M_PI * frequency * 0.001f;
    float amplitude = (float)size / 2.0f;
    float offset    = amplitude;

    float value = offset + amplitude * sinf(omega * (float)t_ms);
    return (uint32_t)value;
}