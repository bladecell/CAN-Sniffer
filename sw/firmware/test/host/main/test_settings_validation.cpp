// P2.9a: settings validation. A bad Wi-Fi or CAN value must be rejected before it reaches NVS or a driver.
#include <cstdint>
#include <limits>
#include <string>

#include "../../../../main/include/settings_validation.hpp"
#include "unity.h"

namespace
{
namespace sv = settings_validation;

const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

void test_checked_int(void)
{
  int64_t out = -1;
  TEST_ASSERT_TRUE(sv::checkedInt(6, 1, 13, out));
  TEST_ASSERT_EQUAL_INT64(6, out);
  TEST_ASSERT_FALSE(sv::checkedInt(0, 1, 13, out));
  TEST_ASSERT_FALSE(sv::checkedInt(14, 1, 13, out));
  TEST_ASSERT_FALSE(sv::checkedInt(6.5, 1, 13, out));
  TEST_ASSERT_FALSE(sv::checkedInt(-1, 0, 255, out));
  TEST_ASSERT_FALSE(sv::checkedInt(kNan, 0, 255, out));
  TEST_ASSERT_FALSE(sv::checkedInt(kInf, 0, 255, out));
  TEST_ASSERT_FALSE(sv::checkedInt(-kInf, 0, 255, out));
  TEST_ASSERT_FALSE(sv::checkedInt(1e30, 0, 4294967295.0, out));
  TEST_ASSERT_TRUE(sv::checkedInt(4294967295.0, 0, 4294967295.0, out));
}

void test_wifi_default_like_valid(void)
{
  TEST_ASSERT_NULL(sv::wifiCheck("Can Sniffer", "", 0, 1, 6, 4, "", "", 3));
}

void test_ssid_length(void)
{
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("", "", 0, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NULL(sv::wifiCheck(std::string(32, 'a'), "", 0, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck(std::string(33, 'a'), "", 0, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 4, std::string(33, 'a'), "", 3));
}

void test_password_rules(void)
{
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "short", 3, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NULL(sv::wifiCheck("a", "12345678", 3, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NULL(sv::wifiCheck("a", std::string(63, 'p'), 3, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", std::string(64, 'p'), 3, 2, 6, 4, "", "", 3));
  // WPA2 AP without a password: esp_wifi_set_config would reject it
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 3, 2, 6, 4, "", "", 3));
  // open AP with a 3-char password is a mismatch the driver rejects too
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "abc", 0, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 4, "net", std::string(64, 'p'), 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 4, "net", "short", 3));
  TEST_ASSERT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 4, "net", "12345678", 3));
}

void test_wifi_enums_and_ranges(void)
{
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 0, 6, 4, "", "", 3));    // mode NULL
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 4, 6, 4, "", "", 3));    // NAN
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 0, 4, "", "", 3));    // channel 0
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 14, 4, "", "", 3));   // channel 14
  TEST_ASSERT_NULL(sv::wifiCheck("a", "", 0, 2, 13, 4, "", "", 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 0, "", "", 3));    // no clients allowed
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 11, "", "", 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 1, 2, 6, 4, "", "", 3));    // WEP
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 5, 2, 6, 4, "", "", 3));    // enterprise
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 99, 2, 6, 4, "", "", 3));
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 4, "", "", 5));    // STA threshold enterprise
  TEST_ASSERT_NOT_NULL(sv::wifiCheck("a", "", 0, 2, 6, 4, "", "", -1));
}

void test_can_values(void)
{
  TEST_ASSERT_TRUE(sv::bitrateValid(125000));
  TEST_ASSERT_TRUE(sv::bitrateValid(250000));
  TEST_ASSERT_TRUE(sv::bitrateValid(500000));
  TEST_ASSERT_TRUE(sv::bitrateValid(1000000));
  TEST_ASSERT_FALSE(sv::bitrateValid(0));
  TEST_ASSERT_FALSE(sv::bitrateValid(800000));
  TEST_ASSERT_FALSE(sv::bitrateValid(-500000));
  TEST_ASSERT_TRUE(sv::rsModeValid(0));
  TEST_ASSERT_TRUE(sv::rsModeValid(1));
  TEST_ASSERT_FALSE(sv::rsModeValid(2));
  TEST_ASSERT_FALSE(sv::rsModeValid(-1));
}

void test_gpio_whitelist(void)
{
  for (int p : {1, 4, 5, 18, 21, 38, 42, 47, 48}) TEST_ASSERT_TRUE_MESSAGE(sv::gpioUsable(p), "usable");
  // BOOT, USB, flash/PSRAM (26-37), console UART0, input-only 46, out of range, NC
  for (int p : {-1, 0, 19, 20, 22, 25, 26, 30, 32, 33, 35, 37, 43, 44, 46, 49, 100, -2}) TEST_ASSERT_FALSE_MESSAGE(sv::gpioUsable(p), "reserved");
}

void test_can_pins_distinct_and_not_board(void)
{
  const int board[] = {9, 10, 11, 12, 13, 47};
  TEST_ASSERT_NULL(sv::canPinsCheck(5, 4, 7, 6, board, 6));
  TEST_ASSERT_NOT_NULL(sv::canPinsCheck(5, 5, 7, 6, board, 6));    // tx == rx
  TEST_ASSERT_NOT_NULL(sv::canPinsCheck(5, 4, 4, 6, board, 6));
  TEST_ASSERT_NOT_NULL(sv::canPinsCheck(5, 4, 7, 5, board, 6));
  TEST_ASSERT_NOT_NULL(sv::canPinsCheck(10, 4, 7, 6, board, 6));   // SD MOSI
  TEST_ASSERT_NOT_NULL(sv::canPinsCheck(5, 4, 7, 47, board, 6));   // LED
  TEST_ASSERT_NOT_NULL(sv::canPinsCheck(26, 4, 7, 6, board, 6));   // flash
  TEST_ASSERT_NOT_NULL(sv::canPinsCheck(5, 33, 7, 6, board, 6));   // PSRAM
  TEST_ASSERT_NULL(sv::canPinsCheck(5, 4, 7, 6, nullptr, 0));
}

void test_mask_filter(void)
{
  TEST_ASSERT_TRUE(sv::filterValid(0x700, 0x700, false));
  TEST_ASSERT_FALSE(sv::filterValid(0x800, 0x700, false));
  TEST_ASSERT_FALSE(sv::filterValid(0x700, 0x800, false));
  TEST_ASSERT_TRUE(sv::filterValid(0x1FFFFFFF, 0x1FFFFFFF, true));
  TEST_ASSERT_FALSE(sv::filterValid(0x20000000, 0, true));
}

void test_path(void)
{
  TEST_ASSERT_TRUE(sv::sdPathValid("/sdcard/config/pid_def.json"));
  TEST_ASSERT_TRUE(sv::sdPathValid("/sdcard/" + std::string(247, 'a')));  // 255 total
  TEST_ASSERT_FALSE(sv::sdPathValid("/sdcard/" + std::string(248, 'a')));  // 256: would not fit the NVS read buffer
  TEST_ASSERT_FALSE(sv::sdPathValid(""));
  TEST_ASSERT_FALSE(sv::sdPathValid("/sdcard/"));
  TEST_ASSERT_FALSE(sv::sdPathValid("/etc/passwd"));
  TEST_ASSERT_FALSE(sv::sdPathValid("/sdcard/../x"));
  TEST_ASSERT_FALSE(sv::sdPathValid("/sdcard/a\nb"));
}

void test_write_limits_fit_read_buffers(void)
{
  // A string that passes validation must always fit the buffer the NVS loader reads it into (value + NUL).
  TEST_ASSERT_TRUE(sv::kPassMax + 1 <= sv::kWifiStrBuf);
  TEST_ASSERT_TRUE(sv::kSsidMax + 1 <= sv::kWifiStrBuf);
  TEST_ASSERT_TRUE(sv::kPathMax + 1 <= sv::kPathBuf);
}
}  // namespace

extern "C" void run_settings_validation_tests(void)
{
  RUN_TEST(test_checked_int);
  RUN_TEST(test_wifi_default_like_valid);
  RUN_TEST(test_ssid_length);
  RUN_TEST(test_password_rules);
  RUN_TEST(test_wifi_enums_and_ranges);
  RUN_TEST(test_can_values);
  RUN_TEST(test_gpio_whitelist);
  RUN_TEST(test_can_pins_distinct_and_not_board);
  RUN_TEST(test_mask_filter);
  RUN_TEST(test_path);
  RUN_TEST(test_write_limits_fit_read_buffers);
}
