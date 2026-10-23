#include "harness.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "esp_system.h"
#include "sdkconfig.h"

#if !defined(CONFIG_SNIFFER_HW_REV_V1_1) || !defined(CONFIG_ESPTOOLPY_FLASHSIZE_16MB)
#error "Canonical lifecycle harness requires V1_1/16MB"
#endif
namespace p28 {
[[noreturn]] void stopUnsafe(const char* reason) {
  std::printf("P28 HALTED: %s; retain resources/evidence, no automatic restart\n", reason);
  std::fflush(stdout);
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
}
namespace {
void command(char* line) {
  if (std::strcmp(line, "unity") == 0 && !p28::restorationPending()) {
    runUnityMenu();
  } else if (std::strcmp(line, "lifecycle") == 0 && !p28::restorationPending()) {
    std::printf("P28 LIFECYCLE %s\n", p28::runLifecycle() ? "PASS" : "FAIL");
  } else if (std::strcmp(line, "backup") == 0) {
    std::printf("P28 BACKUP %s\n", p28::saveBackup() ? "READY" : "REFUSED");
  } else if (std::strcmp(line, "recover") == 0) {
    std::printf("P28 RECOVERY %s\n", p28::retryRestoration() ? "PASS" : "FAIL/REFUSED");
  } else if (std::strncmp(line, "nvs ", 4) == 0) {
    const char* token = line + 4;
    bool valid = std::strlen(token) == 8;
    for (const char* p = token; *p; ++p) {
      valid = valid && ((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F'));
    }
    if (valid) std::printf("P28 NVS %s\n", p28::runNvs(static_cast<unsigned>(std::strtoul(token, nullptr, 16))) ? "PASS" : "FAIL/REFUSED");
    else std::puts("P28 REFUSED: token must be eight hex digits");
  } else if (std::strncmp(line, "restore ", 8) == 0) {
    char* rx = line + 8;
    char* split = std::strchr(rx, ' ');
    if (split && !std::strchr(split + 1, ' ')) {
      *split = '\0';
      std::printf("P28 RECOVERY %s\n", p28::restoreExternal(rx, split + 1) ? "PASS" : "FAIL/REFUSED");
    } else std::puts("P28 REFUSED: restore <rx: absent|u32:N> <tx: absent|u32:N>");
  } else std::puts("P28 Commands: unity | lifecycle | backup | nvs <8hex-token> | recover | restore <rx> <tx>");
  std::fflush(stdout);
}
}
extern "C" void app_main() {
  std::printf("P28 BENCH ONLY reset_reason=%d; no automatic tests, NVS init/write or reboot\n", static_cast<int>(esp_reset_reason()));
  std::puts("P28 Physically disconnect vehicle/CAN; obtain operator approval before commands.");
  std::puts("P28 Commands: unity | lifecycle | backup | nvs <8hex-token> | recover | restore <rx> <tx>");
  std::fflush(stdout);
  char line[128];
  size_t used = 0;
  bool overflow = false;
  for (;;) {
    const int c = std::getchar();
    if (c == EOF) { std::clearerr(stdin); vTaskDelay(pdMS_TO_TICKS(20)); }
    else if (c == '\n' || c == '\r') {
      if (!overflow && used) { line[used] = '\0'; command(line); }
      else if (overflow) std::puts("P28 REFUSED: overlong command");
      used = 0; overflow = false;
    } else if (c < 32 || c > 126) overflow = true;
    else if (used + 1 < sizeof(line)) line[used++] = static_cast<char>(c);
    else overflow = true;
  }
}
