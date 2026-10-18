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

    // Idempotent. On failure nothing stays allocated and every other call is a no-op.
    esp_err_t init(gpio_num_t pin, uint32_t active_low = 0)
    {
        if (state_.load() != State::NOT_INITIALIZED)
        {
            ESP_LOGW("LED_STATUS", "LED already initialized");
            return ESP_OK;
        }
        // Created once and never deleted: blink() from another task may still hold the handle while deinit() runs, and
        // a deleted semaphore would be a use-after-free. One binary semaphore is not worth a lock.
        if (sem_ == nullptr)
        {
            sem_ = xSemaphoreCreateBinary();
            if (sem_ == nullptr)
            {
                return ESP_ERR_NO_MEM;
            }
        }
        pin_        = pin;
        active_low_ = active_low;
        stop_.store(false);
        exited_.store(false);
        if (xTaskCreate(runWrapper, "led_worker", 1536, this, 2, &task_handle_) != pdPASS)
        {
            task_handle_ = nullptr;
            return ESP_ERR_NO_MEM;
        }
        gpio_reset_pin(pin_);
        gpio_set_direction(pin_, GPIO_MODE_OUTPUT);
        state_.store(State::OFF);
        turn_off();
        return ESP_OK;
    }

    // LED off, worker task stopped (it leaves on its own, never vTaskDelete from outside), pin released; the semaphore stays. Safe to call
    // when not initialised; init() works again afterwards. ESP_ERR_TIMEOUT: the worker did not leave, nothing freed.
    esp_err_t deinit()
    {
        if (state_.load() == State::NOT_INITIALIZED)
        {
            return ESP_OK;
        }
        state_.store(State::NOT_INITIALIZED);  // blink()/turn_on()... become no-ops from here
        stop_.store(true);
        xSemaphoreGive(sem_);
        for (int i = 0; i < 50 && !exited_.load(); i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!exited_.load())
        {
            state_.store(State::OFF);  // still owned by the worker; leave the object usable
            stop_.store(false);
            return ESP_ERR_TIMEOUT;
        }
        task_handle_ = nullptr;
        state_.store(State::NOT_INITIALIZED);  // the worker may have stored ON/OFF while it was finishing a blink
        gpio_set_level(pin_, active_low_ ? 1 : 0);
        gpio_reset_pin(pin_);
        (void)xSemaphoreTake(sem_, 0);  // drop a token left by the stop request so the next worker starts clean
        ledState_.store(0);
        return ESP_OK;
    }

    ~LedStatus()
    {
        deinit();
    }

    void turn_on()
    {
        if (!ready())
        {
            return;
        }
        state_.store(State::ON);
        wait_time_.store(portMAX_DELAY);
        xSemaphoreGive(sem_);
    }

    void turn_off()
    {
        if (!ready())
        {
            return;
        }
        state_.store(State::OFF);
        wait_time_.store(portMAX_DELAY);
        xSemaphoreGive(sem_);
    }

    void toggle()
    {
        if (!ready())
        {
            return;
        }
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
        if (!ready() || state_.load() == State::BLINK)
        {
            return;
        }
        state_.store(State::BLINK);
        on_time_.store(pdMS_TO_TICKS(on_time_ms));
        off_time_.store(pdMS_TO_TICKS(off_time_ms));
        count_.store(count * 2);
        xSemaphoreGive(sem_);
    }

private:
    bool ready() const
    {
        return state_.load() != State::NOT_INITIALIZED && sem_ != nullptr;
    }

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
        while (!stop_.load())
        {
            TickType_t delay = wait_time_.load();
            xSemaphoreTake(sem_, delay > 0 ? delay : 1);
            if (stop_.load())
            {
                break;
            }

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
        exited_.store(true);
        vTaskDelete(nullptr);
    }

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

    gpio_num_t              pin_ = GPIO_NUM_NC;
    uint32_t                active_low_  = 0;
    std::atomic<uint32_t>   ledState_    = 0;
    std::atomic<State>      state_       = State::NOT_INITIALIZED;
    std::atomic<TickType_t> wait_time_   = portMAX_DELAY;
    std::atomic<TickType_t> on_time_     = 0;
    std::atomic<TickType_t> off_time_    = 0;
    std::atomic<TickType_t> count_       = 0;
    std::atomic<bool>       stop_        = false;
    std::atomic<bool>       exited_      = false;
    TaskHandle_t            task_handle_ = nullptr;
    SemaphoreHandle_t       sem_         = nullptr;
};