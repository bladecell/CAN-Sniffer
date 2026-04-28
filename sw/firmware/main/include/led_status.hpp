#pragma once
#include <atomic>
#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

class LedStatus
{
public:
    static LedStatus& getInstance()
    {
        static LedStatus instance;
        return instance;
    }

    static void staticBlink(void* arg)
    {
        getInstance().blink(1, 4, 4);
    }

    enum class State
    {
        NOT_INITIALIZED,
        ON,
        OFF,
        BLINK,
        ERROR,
    };

    void init(gpio_num_t pin, uint32_t active_low = 0)
    {
        if (state_.load() != State::NOT_INITIALIZED)
        {
            ESP_LOGW("LED_STATUS", "LED already initialized");
            return;
        }
        pin_ = pin;
        gpio_reset_pin(pin_);
        gpio_set_direction(pin_, GPIO_MODE_OUTPUT);
        sem_ = xSemaphoreCreateBinary();
        if (task_handle_ == nullptr)
        {
            xTaskCreate(runWrapper, "led_worker", 1536, this, 2, &task_handle_);
        }
        active_low_ = active_low;
        turn_off();
    }

    ~LedStatus()
    {
        if (task_handle_ != nullptr)
        {
            vTaskDelete(task_handle_);
        }
        if (sem_ != nullptr)
        {
            vSemaphoreDelete(sem_);
        }
    }

    void turn_on()
    {
        state_.store(State::ON);
        wait_time_.store(portMAX_DELAY);
        xSemaphoreGive(sem_);
    }

    void turn_off()
    {
        state_.store(State::OFF);
        wait_time_.store(portMAX_DELAY);
        xSemaphoreGive(sem_);
    }

    void toggle()
    {
        int current = ledState_.load();
        if (current == 0)
        {
            state_.store(State::ON);
        }
        else
        {
            state_.store(State::OFF);
        }
        xSemaphoreGive(sem_);
    }

    void blink(uint32_t count, uint32_t on_time_ms, uint32_t off_time_ms)
    {
        if (state_.load() == State::BLINK)
            return;
        state_.store(State::BLINK);
        on_time_.store(pdMS_TO_TICKS(on_time_ms));
        off_time_.store(pdMS_TO_TICKS(off_time_ms));
        count_.store(count * 2);
        xSemaphoreGive(sem_);
    }

private:
    LedStatus()
    {
    }
    LedStatus(const LedStatus&)            = delete;
    LedStatus& operator=(const LedStatus&) = delete;

    static void runWrapper(void* arg)
    {
        static_cast<LedStatus*>(arg)->run();
    }
    void run()
    {
        while (true)
        {
            TickType_t delay = wait_time_.load();
            xSemaphoreTake(sem_, delay > 0 ? delay : 1);

            State state = static_cast<State>(state_.load());
            switch (state)
            {
                case State::ON:
                    on();
                    wait_time_.store(portMAX_DELAY);
                    break;
                case State::OFF:
                    off();
                    wait_time_.store(portMAX_DELAY);
                    break;
                case State::BLINK:
                    blink_runner();
                    break;
                case State::ERROR:
                    // error();
                    break;
                default:
                    wait_time_.store(portMAX_DELAY);
                    break;
            }
        }
    };

    void on()
    {
        gpio_set_level(pin_, active_low_ ? 0 : 1);
        ledState_.store(1);
    }

    void off()
    {
        gpio_set_level(pin_, active_low_ ? 1 : 0);
        ledState_.store(0);
    }

    void blink_runner()
    {
        uint32_t count = count_.load();

        // Stop condition
        if (count == 0)
        {
            wait_time_.store(portMAX_DELAY);
            if (ledState_.load() == 0)
            {
                state_.store(State::OFF);
            }
            else
            {
                state_.store(State::ON);
            }
            return;
        }

        uint32_t led_state = ledState_.load();
        if (led_state == 0)
        {
            on();
            wait_time_.store(on_time_.load());
        }
        else
        {
            off();
            wait_time_.store(off_time_.load());
        }
        if (count != 0xFFFFFFFF)
        {
            count_.store(count - 1);
        }
    }

    gpio_num_t              pin_;
    uint32_t                active_low_  = 0;
    std::atomic<uint32_t>   ledState_    = 0;
    std::atomic<State>      state_       = State::NOT_INITIALIZED;
    std::atomic<TickType_t> wait_time_   = portMAX_DELAY;
    std::atomic<TickType_t> on_time_     = 0;
    std::atomic<TickType_t> off_time_    = 0;
    std::atomic<TickType_t> count_       = 0;
    TaskHandle_t            task_handle_ = nullptr;
    SemaphoreHandle_t       sem_         = nullptr;
};