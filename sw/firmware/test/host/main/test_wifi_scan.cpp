#include "../../../components/wifi/include/wifi_scan_policy.hpp"
#include "../../../main/include/wifi_scan_http.hpp"
#include <cstring>

#ifdef WIFI_SCAN_STANDALONE
#include <cassert>
#define TEST_ASSERT_EQUAL_INT(a, b) assert((a) == (b))
#define RUN_TEST(fn) fn()
#else
#include "unity.h"
#endif

static void test_scan_admission(void)
{
  using namespace wifi_scan;
  TEST_ASSERT_EQUAL_INT(Admission::Ready, admit(true, false, false, 0, -1));
  TEST_ASSERT_EQUAL_INT(Admission::Busy, admit(true, true, false, 100, -1));
  TEST_ASSERT_EQUAL_INT(Admission::Down, admit(false, false, false, 100, -1));
  TEST_ASSERT_EQUAL_INT(Admission::Down, admit(true, false, true, 100, -1));
  TEST_ASSERT_EQUAL_INT(Admission::Cooldown, admit(true, false, false, 9999999, 0));
  TEST_ASSERT_EQUAL_INT(Admission::Ready, admit(true, false, false, 10000000, 0));
}

static void test_scan_expiry_and_hostile_ssid(void)
{
  using namespace wifi_scan;
  TEST_ASSERT_EQUAL_INT(false, expired(29999999, 0));
  TEST_ASSERT_EQUAL_INT(true, expired(30000000, 0));
  TEST_ASSERT_EQUAL_INT(false, deadlineReached(4999999, 0));
  TEST_ASSERT_EQUAL_INT(true, deadlineReached(5000000, 0));
  const uint8_t input[] = {'a', '"', '\\', 1, 0xff, 0xc3, 0xa9, 0};
  char text[33], hex[65];
  encodeSsid(input, sizeof(input), text, hex);
  TEST_ASSERT_EQUAL_INT(0, strcmp("a\"\\??\xc3\xa9", text));
  TEST_ASSERT_EQUAL_INT(0, strcmp("61225c01ffc3a9", hex));
  uint8_t full[33];
  memset(full, 'x', sizeof(full));
  encodeSsid(full, sizeof(full), text, hex);
  TEST_ASSERT_EQUAL_INT(32, strlen(text));
  TEST_ASSERT_EQUAL_INT(64, strlen(hex));
}

static void test_completion_is_one_shot_and_teardown_rejects_late_events(void)
{
  wifi_scan::CompletionGate gate;
  TEST_ASSERT_EQUAL_INT(false, gate.accept());
  gate.arm();
  TEST_ASSERT_EQUAL_INT(true, gate.accept());
  TEST_ASSERT_EQUAL_INT(false, gate.accept());
  gate.arm();
  gate.close();
  TEST_ASSERT_EQUAL_INT(false, gate.accept());
}

static void test_ap_restoration_never_starts_sta_reconnect(void)
{
  TEST_ASSERT_EQUAL_INT(false, wifi_scan::mayReconnect(true, false, false));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan::mayReconnect(false, true, false));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan::mayReconnect(false, false, true));
  TEST_ASSERT_EQUAL_INT(true, wifi_scan::mayReconnect(false, false, false));
}

static void test_sta_without_ssid_falls_back_to_ap(void)
{
  TEST_ASSERT_EQUAL_INT(true, wifi_scan::staNeedsApFallback(true, true));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan::staNeedsApFallback(true, false));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan::staNeedsApFallback(false, true));
}

static void test_scan_route_accepts_only_bodyless_queryless_requests(void)
{
  TEST_ASSERT_EQUAL_INT(true, wifi_scan_http::accepts("/api/v1/wifi/scan", 0, false));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan_http::accepts("/api/v1/wifi/scan?limit=999", 0, false));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan_http::accepts("/api/v1/wifi/scan?", 0, false));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan_http::accepts("/api/v1/wifi/scan", 1, false));
  TEST_ASSERT_EQUAL_INT(false, wifi_scan_http::accepts("/api/v1/wifi/scan", 0, true));
}

extern "C" void run_wifi_scan_tests(void)
{
  RUN_TEST(test_scan_admission);
  RUN_TEST(test_scan_expiry_and_hostile_ssid);
  RUN_TEST(test_completion_is_one_shot_and_teardown_rejects_late_events);
  RUN_TEST(test_ap_restoration_never_starts_sta_reconnect);
  RUN_TEST(test_sta_without_ssid_falls_back_to_ap);
  RUN_TEST(test_scan_route_accepts_only_bodyless_queryless_requests);
}

#ifdef WIFI_SCAN_STANDALONE
int main() { run_wifi_scan_tests(); }
#endif
