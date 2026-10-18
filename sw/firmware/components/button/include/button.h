#pragma once

// BOOT button (GPIO0) reader. GPIO0 is a strapping pin: this driver only ever reads it (input, pull-up), never drives it.
//
// Interrupt on both edges wakes one task; the task waits 30 ms for the contact to settle, samples once and sleeps again.
// While the button is untouched nothing runs (no polling, no timer). A button already down at init is ignored until it
// has been released once, so holding BOOT through a reset or a wake cannot count as a press.
//
// Lifecycle: init() / deinit() are idempotent and may be called from the supervisor task only. deinit() releases the
// ISR handler, the task, the mutex and the pin; nothing is left after it.

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Called from the button task (not from the ISR) once per press that is held for button_hold_us(). Must be short and
// must not call button_set_hold_hook().
typedef void (*button_hold_fn)(void* arg);

esp_err_t button_init(void);
// ESP_ERR_TIMEOUT: the task did not leave in time; nothing was freed, call again later.
esp_err_t button_deinit(void);

// esp_timer time (microseconds) of the last debounced press since init(), 0 when there was none or the driver is down.
int64_t button_last_press_us(void);

// Holding the button this long fires the hold hook.
int64_t button_hold_us(void);

// Replaces (nullptr clears) the hold hook. When it returns ESP_OK no call of the previous hook is running or can start.
// ESP_ERR_INVALID_STATE: the driver is not initialised. ESP_ERR_TIMEOUT: a hook call did not finish in 1 s.
esp_err_t button_set_hold_hook(button_hold_fn fn, void* arg);

#ifdef __cplusplus
}
#endif
