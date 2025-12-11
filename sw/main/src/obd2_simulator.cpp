// obd2_simulator.cpp
#include "obd2_simulator.hpp"
#include <math.h>

static const char *TAG = "SIM_TASK";

TaskHandle_t xDataSimTaskHandle = NULL;

void start_sim_task(CanDriver *driver)
{

    // Using xTaskCreate for automatic core selection (Floating Task)
    BaseType_t result = xTaskCreatePinnedToCore(
        dataSimTaskWrapper,
        "DataSimTask",
        4096,
        (void *)driver,
        tskIDLE_PRIORITY + 2,
        &xDataSimTaskHandle,
        1);

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

void dataSimTaskWrapper(void *param)
{
    CanDriver *driver = static_cast<CanDriver *>(param);
    dataSimTask(*driver);
}

void dataSimTask(CanDriver &canDriver)
{
    uint32_t requested_pid = 0;
    bool multiframe_in_progress = false;
    static const char vin[] = "1HGBH41JXMN109186";

    while (1)
    {
        // Block until notified with a PID request
        if (xTaskNotifyWait(0x00, ULONG_MAX, &requested_pid, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        if ((requested_pid == PID_VIN) || ((requested_pid == 0x00) && multiframe_in_progress)) // VIN with multiframe
        {
            if (!multiframe_in_progress)
            {
                multiframe_in_progress = true;
                CanDriver::CanFrame response_frame = {};
                response_frame.id = 0x7E8;
                response_frame.data[0] = 0x10;
                response_frame.data[1] = 0x14;
                response_frame.data[2] = 0x49;
                response_frame.data[3] = (uint8_t)requested_pid;
                response_frame.data[4] = 0x01;
                response_frame.data[5] = vin[0];
                response_frame.data[6] = vin[1];
                response_frame.data[7] = vin[2];
                response_frame.length = PID_DATA_LENGTH; // Max length for CAN 2.0A

                if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Failed to inject simulated frame.");
                }
                continue;
            }
            else
            {
                CanDriver::CanFrame consecutiveFrame1 = {};
                consecutiveFrame1.id = 0x7E8;
                consecutiveFrame1.data[0] = 0x21; // Consecutive Frame
                consecutiveFrame1.data[1] = vin[3];
                consecutiveFrame1.data[2] = vin[4];
                consecutiveFrame1.data[3] = vin[5];
                consecutiveFrame1.data[4] = vin[6];
                consecutiveFrame1.data[5] = vin[7];
                consecutiveFrame1.data[6] = vin[8];
                consecutiveFrame1.data[7] = vin[9];
                consecutiveFrame1.length = PID_DATA_LENGTH; // Max length for CAN 2.0A

                if (xQueueSend(canDriver.getRxQueueHandle(), &consecutiveFrame1, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Failed to inject simulated frame.");
                }

                CanDriver::CanFrame consecutiveFrame2 = {};
                consecutiveFrame2.id = 0x7E8;
                consecutiveFrame2.data[0] = 0x22; // Consecutive Frame
                consecutiveFrame2.data[1] = vin[10];
                consecutiveFrame2.data[2] = vin[11];
                consecutiveFrame2.data[3] = vin[12];
                consecutiveFrame2.data[4] = vin[13];
                consecutiveFrame2.data[5] = vin[14];
                consecutiveFrame2.data[6] = vin[15];
                consecutiveFrame1.data[7] = vin[16];
                consecutiveFrame2.length = PID_DATA_LENGTH; // Max length for CAN 2.0A

                if (xQueueSend(canDriver.getRxQueueHandle(), &consecutiveFrame2, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Failed to inject simulated frame.");
                }
                multiframe_in_progress = false;
                continue;
            }
        }

        CanDriver::CanFrame response_frame = {};

        // Response ID for ECU 0 (0x7E8)
        response_frame.id = 0x7E8;
        response_frame.data[1] = 0x41; // Response Mode (0x01 + 0x40)
        response_frame.data[2] = (uint8_t)requested_pid;
        response_frame.length = PID_DATA_LENGTH; // Max length for CAN 2.0A

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
            response_frame.data[0] = 0x06; // Length 6 (4 data bytes)
            response_frame.data[3] = 0xFF; // A
            response_frame.data[4] = 0xFF; // B
            response_frame.data[5] = 0xFF; // C
            response_frame.data[6] = 0xFF; // D
            break;
        case PID_ENGINE_RPM:
            val = sinDataSim(t_ms, 0.07f, 65536);
            response_frame.data[0] = 0x04;
            response_frame.data[3] = (val >> 8) & 0xFF;
            response_frame.data[4] = (val >> 16) & 0xFF;
            break;

        case PID_ENGINE_LOAD:
            val = sinDataSim(t_ms, 0.03f, 255);
            response_frame.data[0] = 0x03;
            response_frame.data[3] = val;
            response_frame.data[4] = 0x00;
            break;

        case PID_COOLANT_TEMP:
            val = sinDataSim(t_ms, 0.001f, 255);
            response_frame.data[0] = 0x03;
            response_frame.data[3] = val;
            response_frame.data[4] = 0x00;
            break;

        default:
            ESP_LOGW(TAG, "PID 0x%02lX not simulated.", requested_pid);
            continue; // Ignore unknown PIDs
        }

        // Inject the response frame into the RX queue
        if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Failed to inject simulated frame.");
        }
    }
}

uint32_t sinDataSim(uint32_t t_ms, float frequency, uint32_t size)
{
    float omega = 2 * M_PI * frequency * 0.001f;
    float amplitude = (float)size / 2.0f;
    float offset = amplitude;

    float value = offset + amplitude * sinf(omega * (float)t_ms);
    return (uint32_t)value;
}