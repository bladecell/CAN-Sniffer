#include "button.h"

#include <stddef.h>

#include "button_logic.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define BUTTON_GPIO GPIO_NUM_0
#define DEBOUNCE_MS 30
#define HOLD_US (10LL * 1000 * 1000)  // recovery gesture: hold BOOT for 10 s
#define TASK_STACK 4096               // the hold hook writes NVS
#define TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define EXIT_WAIT_MS 500
#define HOOK_WAIT_MS 1000

static const char* TAG = "BUTTON";

static portMUX_TYPE      s_lock = portMUX_INITIALIZER_UNLOCKED;  // guards s_lastPressUs (64-bit, not atomic on this core)
static int64_t           s_lastPressUs;
static bool              s_up;
static bool              s_isrAttached;
static volatile bool     s_stop;
static TaskHandle_t      s_task;
static SemaphoreHandle_t s_exitSem;
static SemaphoreHandle_t s_hookMutex;  // held by the task for the whole hook call
static button_hold_fn    s_hook;
static void*             s_hookArg;

static void IRAM_ATTR onEdge(void* arg)
{
  (void)arg;
  BaseType_t woken = pdFALSE;
  if (s_task != NULL)
  {
    vTaskNotifyGiveFromISR(s_task, &woken);
  }
  if (woken == pdTRUE)
  {
    portYIELD_FROM_ISR();
  }
}

static void buttonTask(void* arg)
{
  (void)arg;
  BtnState st;
  btnInit(&st, gpio_get_level(BUTTON_GPIO) == 0);
  int64_t waitUs = BTN_WAIT_FOREVER;

  while (!s_stop)
  {
    const TickType_t ticks = waitUs == BTN_WAIT_FOREVER ? portMAX_DELAY : pdMS_TO_TICKS(waitUs / 1000 + 1);
    const uint32_t   edges = ulTaskNotifyTake(pdTRUE, ticks);
    if (s_stop)
    {
      break;
    }
    if (edges != 0)
    {
      vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
      (void)ulTaskNotifyTake(pdTRUE, 0);  // contact bounce during the wait is the same event
    }
    const int64_t  now = esp_timer_get_time();
    const uint32_t ev  = btnSample(&st, gpio_get_level(BUTTON_GPIO) == 0, now, HOLD_US);
    if ((ev & BTN_EVENT_PRESS) != 0)
    {
      portENTER_CRITICAL(&s_lock);
      s_lastPressUs = st.lastPressUs;
      portEXIT_CRITICAL(&s_lock);
    }
    if ((ev & BTN_EVENT_HOLD) != 0 && s_hookMutex != NULL && xSemaphoreTake(s_hookMutex, portMAX_DELAY) == pdTRUE)
    {
      if (s_hook != NULL)
      {
        s_hook(s_hookArg);
      }
      xSemaphoreGive(s_hookMutex);
    }
    waitUs = btnWaitUs(&st, esp_timer_get_time(), HOLD_US);
  }
  xSemaphoreGive(s_exitSem);
  vTaskDelete(NULL);
}

esp_err_t button_init(void)
{
  if (s_up)
  {
    return ESP_OK;
  }
  esp_err_t err = ESP_ERR_NO_MEM;
  s_exitSem     = xSemaphoreCreateBinary();
  s_hookMutex   = xSemaphoreCreateMutex();
  if (s_exitSem == NULL || s_hookMutex == NULL)
  {
    goto fail;
  }
  s_stop        = false;
  s_hook        = NULL;
  s_hookArg     = NULL;
  s_lastPressUs = 0;

  // Input only, pull-up on, interrupt on both edges. Never an output: GPIO0 is a strapping pin.
  const gpio_config_t cfg = {
      .pin_bit_mask = 1ULL << BUTTON_GPIO,
      .mode         = GPIO_MODE_INPUT,
      .pull_up_en   = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  err = gpio_config(&cfg);
  if (err != ESP_OK)
  {
    goto fail;
  }
  if (xTaskCreate(buttonTask, "button", TASK_STACK, NULL, TASK_PRIORITY, &s_task) != pdPASS)
  {
    s_task = NULL;
    err    = ESP_ERR_NO_MEM;
    goto fail;
  }
  // Another driver may have installed the shared ISR service already; that is fine, and we never uninstall it.
  err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
  {
    goto fail;
  }
  err = gpio_isr_handler_add(BUTTON_GPIO, onEdge, NULL);
  if (err != ESP_OK)
  {
    goto fail;
  }
  s_isrAttached = true;
  err           = gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_ANYEDGE);
  if (err != ESP_OK)
  {
    goto fail;
  }
  s_up = true;
  return ESP_OK;

fail:
  ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
  if (button_deinit() != ESP_OK)
  {
    ESP_LOGE(TAG, "cleanup after failed init timed out");
  }
  return err;
}

esp_err_t button_deinit(void)
{
  if (s_isrAttached)
  {
    (void)gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_DISABLE);
    (void)gpio_isr_handler_remove(BUTTON_GPIO);
    s_isrAttached = false;
  }
  if (s_task != NULL)
  {
    s_stop = true;
    xTaskNotifyGive(s_task);
    if (xSemaphoreTake(s_exitSem, pdMS_TO_TICKS(EXIT_WAIT_MS)) != pdTRUE)
    {
      return ESP_ERR_TIMEOUT;  // the task may be inside the hook; nothing is freed, call again
    }
    s_task = NULL;
  }
  if (s_exitSem != NULL)
  {
    vSemaphoreDelete(s_exitSem);
    s_exitSem = NULL;
  }
  if (s_hookMutex != NULL)
  {
    vSemaphoreDelete(s_hookMutex);
    s_hookMutex = NULL;
  }
  s_hook = NULL;
  gpio_reset_pin(BUTTON_GPIO);  // back to the reset default (input, pull-up); never an output
  portENTER_CRITICAL(&s_lock);
  s_lastPressUs = 0;
  portEXIT_CRITICAL(&s_lock);
  s_up = false;
  return ESP_OK;
}

int64_t button_last_press_us(void)
{
  portENTER_CRITICAL(&s_lock);
  const int64_t t = s_lastPressUs;
  portEXIT_CRITICAL(&s_lock);
  return t;
}

int64_t button_hold_us(void)
{
  return HOLD_US;
}

esp_err_t button_set_hold_hook(button_hold_fn fn, void* arg)
{
  if (!s_up || s_hookMutex == NULL)
  {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_hookMutex, pdMS_TO_TICKS(HOOK_WAIT_MS)) != pdTRUE)
  {
    return ESP_ERR_TIMEOUT;
  }
  s_hook    = fn;
  s_hookArg = arg;
  xSemaphoreGive(s_hookMutex);
  return ESP_OK;
}
