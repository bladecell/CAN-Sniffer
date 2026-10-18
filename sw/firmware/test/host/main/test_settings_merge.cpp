// SET-3: one missing key must not reset the whole settings section.
#include <map>
#include <string>

#include "../../../main/include/settings_merge.hpp"
#include "unity.h"

namespace
{
constexpr esp_err_t kNotFound = 0x1102;

struct Cfg
{
  std::string ssid = "unset";
  std::string pass = "unset";
  int         chan = 0;
};

// Fake NVS: a key is either present (keeps its value) or missing (cfg field left alone, NOT_FOUND returned).
struct Store
{
  std::map<std::string, std::string> kv;
  int                                defaultsWritten = 0;
  int                                saves           = 0;
  bool                               saveFails       = false;

  esp_err_t get(Cfg& c)
  {
    bool missing = false;
    auto read    = [&](const char* k, std::string& dst) {
      auto it = kv.find(k);
      if (it == kv.end()) { missing = true; } else { dst = it->second; }
    };
    read("ssid", c.ssid);
    read("pass", c.pass);
    auto it = kv.find("chan");
    if (it == kv.end()) { missing = true; } else { c.chan = std::stoi(it->second); }
    return missing ? kNotFound : ESP_OK;
  }
  esp_err_t save(const Cfg& c)
  {
    ++saves;
    if (saveFails) { return ESP_FAIL; }
    kv["ssid"] = c.ssid;
    kv["pass"] = c.pass;
    kv["chan"] = std::to_string(c.chan);
    return ESP_OK;
  }
  void writeDefaults(const Cfg& d)
  {
    ++defaultsWritten;
    kv.clear();
    save(d);
  }
};

const Cfg kDefaults{"CAN-Sniffer", "default-pass", 6};

esp_err_t run(Store& s, Cfg& out)
{
  return settings_merge::recoverMissing(
      out, kDefaults, kNotFound, [&](Cfg& c) { return s.get(c); }, [&](const Cfg& c) { return s.save(c); },
      [&] { s.writeDefaults(kDefaults); });
}

void test_one_missing_key_keeps_the_other_keys(void)
{
  Store s;
  s.kv = {{"ssid", "MyCar"}, {"pass", "secret123"}};  // chan missing
  Cfg out;
  TEST_ASSERT_EQUAL(ESP_OK, run(s, out));
  TEST_ASSERT_EQUAL_STRING("MyCar", out.ssid.c_str());
  TEST_ASSERT_EQUAL_STRING("secret123", out.pass.c_str());
  TEST_ASSERT_EQUAL(6, out.chan);
  TEST_ASSERT_EQUAL(0, s.defaultsWritten);
  TEST_ASSERT_EQUAL_STRING("MyCar", s.kv["ssid"].c_str());
  TEST_ASSERT_EQUAL_STRING("6", s.kv["chan"].c_str());
}

void test_first_boot_saves_the_defaults(void)
{
  Store s;
  Cfg   out;
  TEST_ASSERT_EQUAL(ESP_OK, run(s, out));
  TEST_ASSERT_EQUAL_STRING("CAN-Sniffer", out.ssid.c_str());
  TEST_ASSERT_EQUAL(6, out.chan);
  TEST_ASSERT_EQUAL_STRING("default-pass", s.kv["pass"].c_str());
}

void test_a_merge_that_cannot_be_saved_falls_back_to_defaults(void)
{
  Store s;
  s.kv        = {{"ssid", "MyCar"}};
  s.saveFails = true;
  Cfg             out;
  const esp_err_t err = settings_merge::recoverMissing(
      out, kDefaults, kNotFound, [&](Cfg& c) { return s.get(c); }, [&](const Cfg& c) { return s.save(c); },
      [&] {
        s.saveFails = false;  // the fallback path writes the defaults and must leave a readable config
        s.writeDefaults(kDefaults);
      });
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_EQUAL(1, s.defaultsWritten);
  TEST_ASSERT_EQUAL_STRING("CAN-Sniffer", out.ssid.c_str());
}
}  // namespace

extern "C" void run_settings_merge_tests(void)
{
  RUN_TEST(test_one_missing_key_keeps_the_other_keys);
  RUN_TEST(test_first_boot_saves_the_defaults);
  RUN_TEST(test_a_merge_that_cannot_be_saved_falls_back_to_defaults);
}
