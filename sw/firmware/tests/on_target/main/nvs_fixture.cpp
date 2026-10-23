#include "harness.hpp"
#include <cstdio>
#include <inttypes.h>
#include "esp_mac.h"
#include "esp_random.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "obd2_simulator.hpp"
#include "settings.hpp"

namespace {
struct Backup { p28::SavedKey rx, tx; unsigned token = 0; bool valid = false; } backup;
bool pending = false, initialized = false;
bool initNvs() {
  if (initialized) return true;
  const esp_err_t err = nvs_flash_init();
  std::printf("P28 nvs_init=%s (never erase/retry)\n", esp_err_to_name(err));
  initialized = err == ESP_OK;
  return initialized;
}
bool readKey(nvs_handle_t h, const char* name, p28::SavedKey& out) {
  nvs_type_t type;
  const esp_err_t err = nvs_find_key(h, name, &type);
  if (err == ESP_ERR_NVS_NOT_FOUND) { out = {}; return true; }
  if (err != ESP_OK || type != NVS_TYPE_U32) {
    std::printf("P28 REFUSED: %s unreadable or not u32; obtain typed external backup\n", name);
    return false;
  }
  uint32_t value;
  if (nvs_get_u32(h, name, &value) != ESP_OK) return false;
  out = {true, value};
  return true;
}
bool readPair(p28::SavedKey& rx, p28::SavedKey& tx) {
  if (!initNvs()) return false;
  nvs_handle_t h;
  const esp_err_t err = nvs_open("storage", NVS_READONLY, &h);
  if (err == ESP_ERR_NVS_NOT_FOUND) { rx = {}; tx = {}; return true; }
  if (err != ESP_OK) return false;
  const bool ok = readKey(h, "c_rx_q", rx) && readKey(h, "c_tx_q", tx);
  nvs_close(h);
  return ok;
}
esp_err_t putKey(nvs_handle_t h, const char* name, const p28::SavedKey& value) {
  if (value.present) return nvs_set_u32(h, name, value.value);
  const esp_err_t err = nvs_erase_key(h, name);
  return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}
bool writePair(const p28::SavedKey& rx, const p28::SavedKey& tx) {
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) != ESP_OK) return false;
  esp_err_t err = putKey(h, "c_rx_q", rx);
  if (err == ESP_OK) err = putKey(h, "c_tx_q", tx);
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  p28::SavedKey checkRx, checkTx;
  return err == ESP_OK && readPair(checkRx, checkTx) && checkRx == rx && checkTx == tx;
}
void printKey(const p28::SavedKey& key) {
  if (key.present) std::printf("u32:%" PRIu32, key.value);
  else std::printf("absent");
}
bool boardPins(const CanDriver::Config& c) {
  const auto d = Settings::defaultCanConfig();
  return c.tx_pin == d.tx_pin && c.rx_pin == d.rx_pin && c.rs_pin == d.rs_pin && c.lbk_pin == d.lbk_pin;
}
bool restoreBackup() {
  const bool ok = writePair(backup.rx, backup.tx);
  pending = !ok;
  if (ok) backup.valid = false;
  std::printf("P28 RESTORE %s (typed values/presence read back)\n", ok ? "PASS" : "FAIL; backup retained");
  return ok;
}
bool invalidCase(uint32_t rx, uint32_t tx, uint32_t expectedRx, uint32_t expectedTx) {
  if (!writePair({true, rx}, {true, tx})) return false;
  auto c = Settings::defaultCanConfig();
  const esp_err_t loaded = Settings::getInstance().getCanConfig(c);
  std::printf("P28 loader rx=%" PRIu32 " tx=%" PRIu32 " => rx=%u tx=%" PRIu32 " err=%s\n",
              rx, tx, static_cast<unsigned>(c.rx_queue_size), c.tx_queue_depth, esp_err_to_name(loaded));
  if (loaded != ESP_OK || c.rx_queue_size != expectedRx || c.tx_queue_depth != expectedTx || !boardPins(c)) return false;
  c.debug = true;  // Loopback only in RAM; never change saved debug or pins.
  auto& can = CanDriver::getInstance();
  can.setSimHooks(start_sim_task, stop_sim_task, sim_notify);
  const esp_err_t started = can.init(c);
  const bool clean = p28::cleanupCan();
  return started == ESP_OK && clean;
}
}
namespace p28 {
bool restorationPending() { return pending; }
bool saveBackup() {
  if (pending) return false;
  backup.valid = false;
  if (!readPair(backup.rx, backup.tx)) return false;
  uint8_t mac[6];
  if (esp_efuse_mac_get_default(mac) != ESP_OK) return false;
  backup.token = static_cast<unsigned>(esp_random());
  if (!backup.token) backup.token = 1;
  backup.valid = true;
  std::printf("P28_BACKUP v1 mac=%02x%02x%02x%02x%02x%02x namespace=storage c_rx_q=",
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  printKey(backup.rx); std::printf(" c_tx_q="); printKey(backup.tx);
  std::printf(" token=%08x\n", backup.token);
  std::puts("P28 Save record durably OFF DEVICE; 'nvs <token>' attests backup saved and mutation authorized.");
  std::fflush(stdout);
  return true;
}
bool runNvs(unsigned token) {
  if (!backup.valid || pending || token != backup.token) {
    std::puts("P28 REFUSED: fresh external-backup acknowledgment required"); return false;
  }
  SavedKey rx, tx;
  if (!readPair(rx, tx) || !(rx == backup.rx) || !(tx == backup.tx)) return false;
  auto baseline = Settings::defaultCanConfig();
  if (Settings::getInstance().getCanConfig(baseline) != ESP_OK || !boardPins(baseline)) {
    std::puts("P28 REFUSED: needs complete readable CAN section with V1_1 pins; no defaults written"); return false;
  }
  if (!cleanupCan()) return false;
  pending = true;
  const auto result = withRestoration([] {
    return invalidCase(0, 0, 20, 20) && invalidCase(UINT32_MAX, UINT32_MAX, 20, 20) &&
           invalidCase(0, 7, 20, 7) && invalidCase(31, UINT32_MAX, 31, 20);
  }, restoreBackup);
  std::printf("P28 CASE invalid-saved-queues %s restored=%s\n",
              result.scenario && result.restored ? "PASS" : "FAIL", result.restored ? "yes" : "NO");
  std::puts("P28 Same-boot loader/init only; normal-image reboot-loop proof UNVERIFIED");
  return result.scenario && result.restored;
}
bool retryRestoration() {
  return pending && backup.valid && cleanupCan() && restoreBackup();
}
bool restoreExternal(const char* rxText, const char* txText) {
  SavedKey rx, tx;
  if (!parseKey(rxText, rx) || !parseKey(txText, tx) || !initNvs() || !cleanupCan()) return false;
  const bool ok = writePair(rx, tx);
  pending = !ok;
  if (ok) backup.valid = false;
  std::printf("P28 EXTERNAL_RESTORE %s\n", ok ? "PASS (readback matched)" : "FAIL; known backup retained");
  return ok;
}
}
