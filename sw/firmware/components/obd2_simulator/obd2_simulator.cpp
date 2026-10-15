// obd2_simulator.cpp
#include "obd2_simulator.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#include "can_driver.hpp"
#include "esp_log.h"
#include "freertos/queue.h"
#include "esp_random.h"
#include "obd2_common.hpp"

static const char* TAG = "SIM_TASK";

TaskHandle_t xDataSimTaskHandle = NULL;

// Frames seen by the CAN RX ISR (requests and flow control, loopback). Static storage: no allocation, nothing to free.
struct SimRequest
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  length;
};
static constexpr size_t SIM_QUEUE_DEPTH = 16;
static StaticQueue_t    simQueueBuffer;
static uint8_t          simQueueStorage[SIM_QUEUE_DEPTH * sizeof(SimRequest)];
static QueueHandle_t    simQueue = NULL;

volatile uint32_t simDroppedFrames = 0;

// Test aid, debug builds only: when > 0, ECU 1 answers DTC requests with a 40-DTC list whose consecutive frames are this many
// ms apart (50 gives about 600 ms, longer than the requester's base wait). 0 keeps the default behaviour.
#ifndef SIM_ECU1_CF_DELAY_MS
#define SIM_ECU1_CF_DELAY_MS 0
#endif

static constexpr uint32_t SIM_ECU_COUNT       = 2;
static constexpr uint32_t SIM_REQUEST_BASE_ID = OBD2_RESPONSE_BASE_ID - RESPONSE_ID_OFFSET;  // 0x7E0

void IRAM_ATTR sim_notify(uint32_t id, const uint8_t* data, uint8_t length, BaseType_t* woken)
{
    if (simQueue == NULL || data == NULL || length > sizeof(SimRequest::data))
    {
        return;
    }
    SimRequest r;
    r.id     = id;
    r.length = length;
    memset(r.data, 0, sizeof(r.data));
    memcpy(r.data, data, length);
    if (xQueueSendFromISR(simQueue, &r, woken) != pdTRUE)
    {
        simDroppedFrames = simDroppedFrames + 1;
    }
}

void start_sim_task(CanDriver* driver)
{
    if (xDataSimTaskHandle != NULL)
    {
        return;
    }
    if (simQueue == NULL)
    {
        simQueue = xQueueCreateStatic(SIM_QUEUE_DEPTH, sizeof(SimRequest), simQueueStorage, &simQueueBuffer);
    }
    xQueueReset(simQueue);
    // Using xTaskCreate for automatic core selection (Floating Task)
    BaseType_t result = xTaskCreatePinnedToCore(dataSimTaskWrapper, "DataSimTask", 4096, (void*)driver,
                                                tskIDLE_PRIORITY + 2, &xDataSimTaskHandle, CORE_ID_CAN_TASKS);

    if (result != pdPASS)
    {
        xDataSimTaskHandle = NULL;
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

// A multi-frame response that waits for the tester's flow control (sent to the ECU's physical request ID).
struct SimTransfer
{
    bool     active = false;
    uint8_t  bytes[96];
    uint8_t  total = 0;
    uint8_t  pos   = 0;
    uint8_t  seq   = 1;
};

static void injectFrame(CanDriver& canDriver, uint32_t id, const uint8_t (&bytes)[8])
{
    CanDriver::CanFrame f = {};
    f.header.id           = id;
    memcpy(f.data, bytes, sizeof(bytes));
    f.length = PID_DATA_LENGTH;  // Max length for CAN 2.0A
    if (xQueueSend(canDriver.getRxQueueHandle(), &f, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Failed to inject simulated frame.");
    }
}

// Single frame, or first frame plus a pending transfer that the flow control releases.
static void sendPayload(CanDriver& canDriver, uint32_t id, SimTransfer& t, const uint8_t* payload, uint8_t length)
{
    uint8_t f[8] = {};
    t.active     = false;
    if (length <= 7)
    {
        f[0] = length;
        memcpy(f + 1, payload, length);
        injectFrame(canDriver, id, f);
        return;
    }
    if (length > sizeof(t.bytes))
    {
        return;
    }
    f[0] = 0x10 | ((length >> 8) & 0x0F);
    f[1] = length;
    memcpy(f + 2, payload, 6);
    memcpy(t.bytes, payload, length);
    t.total  = length;
    t.pos    = 6;
    t.seq    = 1;
    t.active = true;
    injectFrame(canDriver, id, f);
}

static void sendConsecutiveFrames(CanDriver& canDriver, uint32_t id, SimTransfer& t, uint32_t gapMs)
{
    while (t.active && t.pos < t.total)
    {
        if (gapMs != 0)
        {
            vTaskDelay(pdMS_TO_TICKS(gapMs));
        }
        uint8_t f[8] = {};
        f[0]         = 0x20 | (t.seq++ & 0x0F);
        for (int j = 1; j <= 7 && t.pos < t.total; j++)
        {
            f[j] = t.bytes[t.pos++];
        }
        injectFrame(canDriver, id, f);
    }
    t.active = false;
}

void dataSimTask(CanDriver& canDriver)
{
    static const char     vin[] = "1HGBH41JXMN109186";
    std::vector<uint16_t> dtcs  = {0x0143, 0x8260, 0x4234, 0x0300, 0x0408, 0x0506, 0x0101, 0x0113, 0x0335, 0x0401};
    // ECU 1 (0x7E9) only answers DTC and VIN requests. 0x0300 and 0x0408 are also reported by ECU 0, to exercise the merge.
    static const uint16_t dtcsEcu1[] = {0x0300, 0x0408, 0x1A10, 0x2345};
    SimTransfer           transfer[SIM_ECU_COUNT];

    while (1)
    {
        SimRequest req;
        if (xQueueReceive(simQueue, &req, portMAX_DELAY) != pdPASS)
        {
            continue;
        }
        if (req.length < 2)
        {
            continue;
        }

        // Flow control frames the firmware sends to a physical request ID (response ID minus 8).
        if ((req.data[0] >> 4) == 0x3)
        {
            if (req.id >= SIM_REQUEST_BASE_ID && req.id < SIM_REQUEST_BASE_ID + SIM_ECU_COUNT)
            {
                const uint32_t ecu = req.id - SIM_REQUEST_BASE_ID;
                uint32_t gapMs = 0;
#if SIM_ECU1_CF_DELAY_MS > 0
                if (ecu == 1)
                {
                    gapMs = SIM_ECU1_CF_DELAY_MS;
                }
#endif
                sendConsecutiveFrames(canDriver, OBD2_RESPONSE_BASE_ID + ecu, transfer[ecu], gapMs);
            }
            continue;
        }
        // Only single-frame requests carry a mode.
        if ((req.data[0] >> 4) != 0x0)
        {
            continue;
        }

        const uint8_t requested_pid = req.data[2];
        const uint8_t mode          = req.data[1];

        // ECU 1 answers the functional ID and its own physical ID.
        const bool ecu1Addressed = req.id == OBD2_FUNCTIONAL_ID || req.id == SIM_REQUEST_BASE_ID + 1;
        if (ecu1Addressed)
        {
            uint8_t payload[96];
            switch (mode)
            {
                case MODE_DTCS:
                case MODE_PENDING_DTCS:
                case MODE_PERMANENT_DTCS:
                {
#if SIM_ECU1_CF_DELAY_MS > 0
                    const uint8_t n = 40;
#else
                    const uint8_t n = sizeof(dtcsEcu1) / sizeof(dtcsEcu1[0]);
#endif
                    payload[0] = mode | 0x40;
                    payload[1] = n;
                    for (uint8_t i = 0; i < n; i++)
                    {
#if SIM_ECU1_CF_DELAY_MS > 0
                        const uint16_t dtc = i < 4 ? dtcsEcu1[i] : static_cast<uint16_t>(0x1000 + i);
#else
                        const uint16_t dtc = dtcsEcu1[i];
#endif
                        payload[2 + 2 * i] = (dtc >> 8) & 0xFF;
                        payload[3 + 2 * i] = dtc & 0xFF;
                    }
                    sendPayload(canDriver, OBD2_RESPONSE_BASE_ID + 1, transfer[1], payload, 2 + 2 * n);
                    break;
                }
                case MODE_VEHICLE_INFO:
                    if (requested_pid == PID_VIN)
                    {
                        payload[0] = RESPONSE_VEHICLE_INFO;
                        payload[1] = PID_VIN;
                        payload[2] = 0x01;
                        for (uint8_t i = 0; i < 17; i++)
{
    payload[3 + i] = static_cast<uint8_t>(vin[i]);
}
                        sendPayload(canDriver, OBD2_RESPONSE_BASE_ID + 1, transfer[1], payload, 20);
                    }
                    break;
                default:
                    break;
            }
        }
        if (req.id == SIM_REQUEST_BASE_ID + 1)
        {
            continue;  // a request for ECU 1 alone: ECU 0 stays quiet
        }

        switch (mode)
        {
            case MODE_CURRENT_DATA:
            {
                CanDriver::CanFrame response_frame = {};

                // Response ID for ECU 0 (0x7E8)
                response_frame.header.id = OBD2_RESPONSE_BASE_ID;
                response_frame.data[1]   = RESPONSE_CURRENT_DATA;  // Response Mode (0x01 + 0x40)
                response_frame.data[2]   = requested_pid;
                response_frame.length    = PID_DATA_LENGTH;  // Max length for CAN 2.0A

                uint32_t t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                uint32_t val;
                switch (requested_pid)
                {
                    case PID_PIDS_SUPPORTED_01_20:
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
                        ESP_LOGW(TAG, "PID 0x%02lX not simulated.", (unsigned long)requested_pid);
                        continue;  // Ignore unknown PIDs
                }

                // Inject the response frame into the RX queue
                if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Failed to inject simulated frame.");
                }
                break;
            }
            case MODE_READ_DATA_BY_IDENTIFIER:
            {
                // Request {03, 22, pidHi, pidLo}.
                const uint16_t rdbiPid = (uint16_t)((req.data[2] << 8) | req.data[3]);
                const uint32_t tNow    = xTaskGetTickCount() * portTICK_PERIOD_MS;

                CanDriver::CanFrame response_frame = {};
                response_frame.header.id           = OBD2_RESPONSE_BASE_ID;
                response_frame.data[1]             = RESPONSE_READ_DATA_BY_IDENTIFIER;
                response_frame.data[2]             = (rdbiPid >> 8) & 0xFF;
                response_frame.data[3]             = rdbiPid & 0xFF;
                response_frame.length              = PID_DATA_LENGTH;

                switch (rdbiPid)
                {
                    case 0x2203:  // Odometer: ((A*256)+B)*10 km, fixed 123450 km
                        response_frame.data[0] = 0x05;
                        response_frame.data[4] = 0x30;
                        response_frame.data[5] = 0x39;  // 0x3039 = 12345
                        break;
                    case 0x2613:  // Interior temp: ((A*256)+B)*0.1 C, swings between 15.0 and 30.0
                    {
                        const uint32_t raw     = 150 + sinDataSim(tNow, 0.02f, 150);
                        response_frame.data[0] = 0x05;
                        response_frame.data[4] = (raw >> 8) & 0xFF;
                        response_frame.data[5] = raw & 0xFF;
                        break;
                    }
                    case 0x2206:  // Fuel amount: A litres, 0..50
                        response_frame.data[0] = 0x04;
                        response_frame.data[4] = (uint8_t)sinDataSim(tNow, 0.005f, 50);
                        break;
                    default:
                        ESP_LOGW(TAG, "RDBI PID 0x%04X not simulated.", rdbiPid);
                        continue;
                }

                if (xQueueSend(canDriver.getRxQueueHandle(), &response_frame, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Failed to inject simulated frame.");
                }
                break;
            }
            case MODE_CLEAR_DTCS:
            {
                CanDriver::CanFrame response_frame = {};
                response_frame.header.id           = OBD2_RESPONSE_BASE_ID;
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
                const uint8_t n = (esp_random() % 10) + 1;
                uint8_t       payload[32];
                payload[0] = mode | 0x40;
                payload[1] = n;
                for (uint8_t i = 0; i < n; i++)
                {
                    payload[2 + 2 * i] = (dtcs[i] >> 8) & 0xFF;
                    payload[3 + 2 * i] = dtcs[i] & 0xFF;
                }
                sendPayload(canDriver, OBD2_RESPONSE_BASE_ID, transfer[0], payload, 2 + 2 * n);
                break;
            }
            case MODE_VEHICLE_INFO:
            {
                if (requested_pid == PID_VIN)  // VIN with multiframe
                {
                    uint8_t payload[20] = {RESPONSE_VEHICLE_INFO, PID_VIN, 0x01};
                    for (uint8_t i = 0; i < 17; i++)
{
    payload[3 + i] = static_cast<uint8_t>(vin[i]);
}
                    sendPayload(canDriver, OBD2_RESPONSE_BASE_ID, transfer[0], payload, 20);
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