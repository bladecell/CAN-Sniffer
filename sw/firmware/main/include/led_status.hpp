#pragma once
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class LedError
{
public:
    // Meyers Singleton
    static LedError& getInstance()
    {
        static LedError instance;
        return instance;
    }

    // Constructor/Destructor
    LedError() : pin_(GPIO_NUM_NC), on_ticks_(0), off_ticks_(0), count_(0), task_handle_(nullptr), state_(false)
    {
    }
    ~LedError()
    {
        stop();
    }

    void init(gpio_num_t pin, uint32_t on_ms = 100, uint32_t off_ms = 100)
    {
        stop();
        pin_       = pin;
        on_ticks_  = pdMS_TO_TICKS(on_ms);
        off_ticks_ = pdMS_TO_TICKS(off_ms);

        gpio_reset_pin(pin_);
        gpio_set_direction(pin_, GPIO_MODE_OUTPUT);
        gpio_set_level(pin_, 0);
    }

    // Simple Controls
    void on()
    {
        stop();
        setLevel(true);
    }
    void off()
    {
        stop();
        setLevel(false);
    }
    void toggle()
    {
        stop();
        setLevel(!state_);
    }

    // Async Patterns
    void error()
    {
        startBlinkTask(0);
    }  // Infinite
    void blink(uint32_t cnt = 1)
    {
        startBlinkTask(cnt);
    }

    void stop()
    {
        if (task_handle_ != nullptr)
        {
            TaskHandle_t to_notify = task_handle_;
            // Signal the task loop to exit
            task_handle_ = nullptr;
            xTaskNotifyGive(to_notify);

            // Give it a moment to self-delete
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

private:
    // Delete copy/assignment
    LedError(const LedError&)            = delete;
    LedError& operator=(const LedError&) = delete;

    void setLevel(bool level)
    {
        state_ = level;
        gpio_set_level(pin_, state_ ? 1 : 0);
    }

    void startBlinkTask(uint32_t cnt)
    {
        stop();
        count_ = cnt;
        // High priority (5) to ensure timing accuracy
        xTaskCreate(taskTrampoline, "led_tsk", 2048, this, 5, &task_handle_);
    }

    static void taskTrampoline(void* arg)
    {
        static_cast<LedError*>(arg)->run();
    }

    void run()
    {
        uint32_t iterations = 0;

        while (true)
        {
            // 1. Check if we should exit (stop() called or count reached)
            if (task_handle_ == nullptr)
                break;
            if (count_ > 0 && iterations >= count_)
                break;

            // 2. LED ON
            setLevel(true);
            if (ulTaskNotifyTake(pdTRUE, on_ticks_) != 0)
                break;  // Interrupted by stop()

            // 3. LED OFF
            setLevel(false);
            if (ulTaskNotifyTake(pdTRUE, off_ticks_) != 0)
                break;  // Interrupted by stop()

            iterations++;
        }

        setLevel(false);
        task_handle_ = nullptr;
        vTaskDelete(nullptr);
    }

    gpio_num_t   pin_;
    TickType_t   on_ticks_;
    TickType_t   off_ticks_;
    uint32_t     count_;
    TaskHandle_t task_handle_;
    bool         state_;
};