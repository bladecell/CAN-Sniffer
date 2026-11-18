#include "sim_can.hpp"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>

namespace sim::can {
namespace {
constexpr size_t rxDepth = 96;
StaticQueue_t rxBuffer, doneBuffer;
uint8_t rxStorage[rxDepth * sizeof(Frame)];
struct Done {
  int64_t ts;
  bool success;
};
uint8_t doneStorage[sizeof(Done)];
QueueHandle_t rxQueue, doneQueue;
twai_node_handle_t node;
bool ready, ackOn, busy;
uint32_t rate;
Frame txFrame;
twai_frame_t wireTx;
int64_t txStart;
CanStats counts;
std::atomic<uint32_t> rxDropped, rxInvalid;
static_assert(std::atomic<uint32_t>::is_always_lock_free);

bool onRx(twai_node_handle_t handle, const twai_rx_done_event_data_t *,
          void *) {
  Frame frame;
  twai_frame_t msg{};
  msg.buffer = frame.data;
  msg.buffer_len = sizeof(frame.data);
  if (twai_node_receive_from_isr(handle, &msg) != ESP_OK || msg.header.fdf ||
      msg.header.dlc > 8 ||
      msg.header.id > (msg.header.ide ? 0x1fffffffU : 0x7ffU)) {
    rxInvalid.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  frame.ts = esp_timer_get_time();
  frame.addr = {msg.header.id, static_cast<bool>(msg.header.ide)};
  frame.rtr = msg.header.rtr;
  frame.len = frame.rtr ? 0 : msg.header.dlc;
  BaseType_t wake = pdFALSE;
  if (xQueueSendFromISR(rxQueue, &frame, &wake) != pdTRUE)
    rxDropped.fetch_add(1, std::memory_order_relaxed);
  return wake == pdTRUE;
}

bool onTx(twai_node_handle_t, const twai_tx_done_event_data_t *event, void *) {
  Done done{esp_timer_get_time(), event->is_tx_success};
  BaseType_t wake = pdFALSE;
  xQueueSendFromISR(doneQueue, &done, &wake);
  return wake == pdTRUE;
}

esp_err_t pins() {
  constexpr int tx = CONFIG_SIM_CAN_TX, rx = CONFIG_SIM_CAN_RX;
  constexpr int rs = CONFIG_SIM_CAN_RS, lbk = CONFIG_SIM_CAN_LBK;
  if (!GPIO_IS_VALID_OUTPUT_GPIO(tx) || !GPIO_IS_VALID_GPIO(rx) ||
      !GPIO_IS_VALID_OUTPUT_GPIO(rs) || !GPIO_IS_VALID_OUTPUT_GPIO(lbk) ||
      tx == rx || tx == rs || tx == lbk || rx == rs || rx == lbk || rs == lbk ||
      tx == 19 || tx == 20 || rx == 19 || rx == 20 || rs == 19 || rs == 20 ||
      lbk == 19 || lbk == 20)
    return ESP_ERR_INVALID_ARG;
  const auto rsPin = static_cast<gpio_num_t>(rs),
             lbkPin = static_cast<gpio_num_t>(lbk);
  esp_err_t err = gpio_set_level(rsPin, 0);
  if (err == ESP_OK)
    err = gpio_set_level(lbkPin, 0);
  if (err == ESP_OK)
    err = gpio_set_direction(lbkPin, GPIO_MODE_OUTPUT);
  if (err == ESP_OK)
    err = gpio_set_direction(rsPin, GPIO_MODE_OUTPUT);
  return err;
}
} // namespace

esp_err_t deinit() {
  ready = false;
  if (!node)
    return ESP_OK;
  auto err = twai_node_disable(node);
  // The driver also returns INVALID_STATE for a stopped node in bus-off.
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE)
    err = twai_node_delete(node);
  if (err != ESP_OK)
    return err;
  node = nullptr;
  if (busy)
    ++counts.txFailed;
  busy = false;
  xQueueReset(rxQueue);
  xQueueReset(doneQueue);
  return ESP_OK;
}

esp_err_t configure(uint32_t bitrate, bool ack) {
  if (ready && bitrate == rate && ack == ackOn)
    return ESP_OK;
  auto err = deinit();
  if (err != ESP_OK)
    return err;
  if (!rxQueue)
    rxQueue = xQueueCreateStatic(rxDepth, sizeof(Frame), rxStorage, &rxBuffer);
  if (!doneQueue)
    doneQueue = xQueueCreateStatic(1, sizeof(Done), doneStorage, &doneBuffer);
  if (!rxQueue || !doneQueue)
    return ESP_ERR_NO_MEM;
  err = pins();
  if (err != ESP_OK)
    return err;
  twai_onchip_node_config_t cfg{};
  cfg.io_cfg.tx = static_cast<gpio_num_t>(CONFIG_SIM_CAN_TX);
  cfg.io_cfg.rx = static_cast<gpio_num_t>(CONFIG_SIM_CAN_RX);
  cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  cfg.bit_timing.bitrate = bitrate;
  cfg.fail_retry_cnt = 0;
  cfg.tx_queue_depth = 1;
  cfg.flags.enable_listen_only = !ack;
  err = twai_new_node_onchip(&cfg, &node);
  if (err != ESP_OK)
    return err;
  twai_event_callbacks_t cb{};
  cb.on_rx_done = onRx;
  cb.on_tx_done = onTx;
  err = twai_node_register_event_callbacks(node, &cb, nullptr);
  if (err == ESP_OK)
    err = twai_node_enable(node);
  if (err != ESP_OK) {
    (void)deinit();
    return err;
  }
  rate = bitrate;
  ackOn = ack;
  ready = true;
  return ESP_OK;
}

bool receive(Frame &frame) {
  return ready && xQueueReceive(rxQueue, &frame, 0) == pdTRUE;
}

bool canTransmit() { return ready && ackOn && !busy; }

bool transmit(const Frame &frame, int64_t now) {
  if (!canTransmit() || frame.len > 8 ||
      frame.addr.id > (frame.addr.ext ? 0x1fffffffU : 0x7ffU))
    return false;
  txFrame = frame;
  wireTx = {};
  wireTx.header.id = frame.addr.id;
  wireTx.header.ide = frame.addr.ext;
  wireTx.header.dlc = frame.len;
  wireTx.buffer = txFrame.data;
  wireTx.buffer_len = frame.len;
  if (twai_node_transmit(node, &wireTx, 0) != ESP_OK) {
    ++counts.txFailed;
    return false;
  }
  busy = true;
  txStart = now;
  return true;
}

TxResult service(int64_t now, Frame &completed) {
  if (!ready)
    return TxResult::none;
  // A stuck transmission (e.g. bus off or a bus held dominant) must not hold
  // the TX slot forever.
  if (busy && now - txStart >= 35'000 &&
      uxQueueMessagesWaiting(doneQueue) == 0) {
    ++counts.busOff;
    (void)deinit();
    return TxResult::failed;
  }
  Done done{};
  if (!busy || xQueueReceive(doneQueue, &done, 0) != pdTRUE)
    return TxResult::none;
  busy = false;
  if (!done.success) {
    ++counts.txFailed;
    return TxResult::failed;
  }
  ++counts.tx;
  completed = txFrame;
  completed.ts = done.ts;
  return TxResult::success;
}

CanStats stats() {
  auto result = counts;
  result.ready = ready;
  result.rxDropped = rxDropped.load(std::memory_order_relaxed);
  result.rxInvalid = rxInvalid.load(std::memory_order_relaxed);
  return result;
}
} // namespace sim::can
