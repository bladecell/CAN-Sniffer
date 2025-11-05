// led_status.hpp
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

class LedError
{
public:
    explicit LedError(gpio_num_t pin, TickType_t on_ms = 100, TickType_t off_ms = 100, uint32_t cnt = 0)
        : pin(pin),
          onticks(pdMS_TO_TICKS(on_ms)),
          offticks(pdMS_TO_TICKS(off_ms)),
          count(cnt) {}

    void init()
    {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
        gpio_set_level(pin, 0);
        state_ = false;
    }

    // --- LED Control ---
    void on()
    {
        stop(); // stop task if running
        gpio_set_level(pin, 1);
        state_ = true;
    }

    void off()
    {
        stop(); // stop task if running
        gpio_set_level(pin, 0);
        state_ = false;
    }

    void toggle()
    {
        stop(); // stop task if running
        state_ = !state_;
        gpio_set_level(pin, state_ ? 1 : 0);
    }

    bool isOn() const { return state_; }

    // --- Error Blink Task ---
    void error()
    {
        stop();
        if (task != nullptr)
            return;
        setCount(0);
        xTaskCreate(&LedError::taskTrampoline, "lederror", 1024, this, 5, &task);
    }

    void blink(uint32_t cnt = 1)
    {
        stop();
        if (task != nullptr)
            return;
        setCount(cnt);
        xTaskCreate(&LedError::taskTrampoline, "ledblink", 1024, this, 5, &task);
    }

    void stop()
    {
        if (task)
        {
            TaskHandle_t t = task;
            task = nullptr;     // signal exit
            xTaskNotifyGive(t); // wake it if blocked
            vTaskDelay(pdMS_TO_TICKS(1));
            if (eTaskGetState(t) != eDeleted)
            {
                vTaskDelete(t);
            }
        }
    }

    bool isRunning() const { return task != nullptr; }

    void setPeriodMs(uint32_t on_ms, uint32_t off_ms)
    {
        onticks = pdMS_TO_TICKS(on_ms);
        offticks = pdMS_TO_TICKS(off_ms);
    }

    void setCount(uint32_t cnt)
    {
        count = cnt;
    }

private:
    static void taskTrampoline(void *arg)
    {
        static_cast<LedError *>(arg)->run();
    }

    void run()
    {

        for (uint32_t i = 0; (count == 0) || (i < count); i++)
        {
            if (ulTaskNotifyTake(pdTRUE, 0) > 0 || task == nullptr)
                break;

            gpio_set_level(pin, 1);
            vTaskDelay(onticks);
            gpio_set_level(pin, 0);
            vTaskDelay(offticks);
        }
        task = nullptr;
        vTaskDelete(nullptr);
    }

    gpio_num_t pin;
    TickType_t onticks;
    TickType_t offticks;
    uint32_t count;
    TaskHandle_t task = nullptr;
    bool state_ = false;
};
