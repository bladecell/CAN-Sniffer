#include "../../../main/include/power_settings_storage.hpp"
#include "../../../main/include/power_settings_json.hpp"
#include "unity.h"
#include <cstring>
#include <initializer_list>
#include <cmath>

namespace
{
struct Stored
{
  uint32_t awake = 14000, low = 10000;
  uint8_t enabled = 0;
  esp_err_t awakeError = ESP_OK, lowError = ESP_OK, enabledError = ESP_OK;
  esp_err_t load(power_settings::Config& c)
  {
    return power_settings::read(c,
      [&](const char* key, uint32_t& value)
      {
        const bool a = strcmp(key, "p_awake_mv") == 0;
        if (!a) TEST_ASSERT_EQUAL_STRING("p_low_mv", key);
        value = a ? awake : low;
        return a ? awakeError : lowError;
      },
      [&](const char* key, uint8_t& value)
      {
        TEST_ASSERT_EQUAL_STRING("p_sleep", key);
        value = enabled;
        return enabledError;
      });
  }
};

void test_stored_power_fields_are_loaded_without_losing_integer_precision()
{
  power_settings::Config c;
  const esp_err_t error = power_settings::read(c,
    [](const char* key, uint32_t& value)
    {
      value = strcmp(key, "p_awake_mv") == 0 ? 14999 : 9001;
      return ESP_OK;
    },
    [](const char*, uint8_t& value) { value = 0; return ESP_OK; });
  TEST_ASSERT_EQUAL_INT(ESP_OK, error);
  TEST_ASSERT_EQUAL_UINT32(14999, c.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(9001, c.low_battery_voltage_mv);
  TEST_ASSERT_FALSE(c.sleep_enabled);
}

void test_legacy_missing_keys_use_defaults_independently()
{
  Stored s;
  power_settings::Config c;
  s.awakeError = s.lowError = s.enabledError = ESP_ERR_NVS_NOT_FOUND;
  TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
  TEST_ASSERT_EQUAL_UINT32(13200, c.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(11400, c.low_battery_voltage_mv);
  TEST_ASSERT_TRUE(c.sleep_enabled);
  s.awakeError = ESP_OK;
  TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
  TEST_ASSERT_EQUAL_UINT32(14000, c.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(11400, c.low_battery_voltage_mv);
}

void test_unusable_keys_default_individually_and_inconsistent_pair_defaults_together()
{
  Stored s;
  power_settings::Config c;
  for (esp_err_t error : {ESP_ERR_NVS_TYPE_MISMATCH, ESP_ERR_NVS_INVALID_LENGTH})
  {
    s.awakeError = error;
    TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
    TEST_ASSERT_EQUAL_UINT32(13200, c.stay_awake_voltage_mv);
    TEST_ASSERT_EQUAL_UINT32(10000, c.low_battery_voltage_mv);
    TEST_ASSERT_FALSE(c.sleep_enabled);
    s.awakeError = ESP_OK;
    s.lowError = error;
    TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
    TEST_ASSERT_EQUAL_UINT32(14000, c.stay_awake_voltage_mv);
    TEST_ASSERT_EQUAL_UINT32(11400, c.low_battery_voltage_mv);
    s.lowError = ESP_OK;
    s.enabledError = error;
    TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
    TEST_ASSERT_TRUE(c.sleep_enabled);
    s.enabledError = ESP_OK;
  }
  s.awakeError = ESP_OK;
  for (uint32_t bad : {0U, 8999U, 16001U, UINT32_MAX})
  {
    s.awake = bad;
    TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
    TEST_ASSERT_EQUAL_UINT32(13200, c.stay_awake_voltage_mv);
    TEST_ASSERT_EQUAL_UINT32(10000, c.low_battery_voltage_mv);
  }
  s.awake = 9000;
  s.low = 16000;
  TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
  TEST_ASSERT_EQUAL_UINT32(13200, c.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(11400, c.low_battery_voltage_mv);
  TEST_ASSERT_FALSE(c.sleep_enabled);
  s.enabled = 2;
  TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(c));
  TEST_ASSERT_TRUE(c.sleep_enabled);
}

void test_operational_power_read_errors_degrade_to_defaults_but_partition_corruption_propagates()
{
  for (esp_err_t error : {ESP_ERR_NO_MEM, ESP_ERR_NVS_INVALID_HANDLE, ESP_ERR_NVS_CORRUPT_KEY_PART})
  {
    for (unsigned field = 0; field < 3; ++field)
    {
      Stored s;
      if (field == 0) s.awakeError = error;
      if (field == 1) s.lowError = error;
      if (field == 2) s.enabledError = error;
      power_settings::Config c;
      c.stay_awake_voltage_mv = 1;
      const bool corrupt = error == ESP_ERR_NVS_CORRUPT_KEY_PART;
      TEST_ASSERT_EQUAL_INT(corrupt ? error : ESP_OK, s.load(c));
      if (corrupt) continue;
      TEST_ASSERT_EQUAL_UINT32(13200, c.stay_awake_voltage_mv);
      TEST_ASSERT_EQUAL_UINT32(11400, c.low_battery_voltage_mv);
      TEST_ASSERT_TRUE(c.sleep_enabled);
    }
  }
}

void test_json_patch_updates_exact_integer_millivolts_and_boolean()
{
  cJSON* json = cJSON_Parse("{\"stay_awake_voltage_mv\":16000,\"low_battery_voltage_mv\":9000,\"sleep_enabled\":false}");
  TEST_ASSERT_NOT_NULL(json);
  power_settings::Config c;
  const auto result = power_settings::patch(json, c);
  cJSON_Delete(json);
  TEST_ASSERT_NULL(result.error);
  TEST_ASSERT_EQUAL_UINT32(3, result.count);
  TEST_ASSERT_EQUAL_UINT32(16000, c.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(9000, c.low_battery_voltage_mv);
  TEST_ASSERT_FALSE(c.sleep_enabled);
}

void test_json_partial_patch_validates_the_final_pair_without_mutating_on_error()
{
  power_settings::Config c{14000, 10000, true};
  cJSON* json = cJSON_Parse("{\"stay_awake_voltage_mv\":9500}");
  auto result = power_settings::patch(json, c);
  cJSON_Delete(json);
  TEST_ASSERT_NOT_NULL(result.error);
  TEST_ASSERT_EQUAL_UINT32(14000, c.stay_awake_voltage_mv);
  json = cJSON_Parse("{\"stay_awake_voltage_mv\":9500,\"low_battery_voltage_mv\":9000}");
  result = power_settings::patch(json, c);
  cJSON_Delete(json);
  TEST_ASSERT_NULL(result.error);
  TEST_ASSERT_EQUAL_UINT32(2, result.count);
  TEST_ASSERT_EQUAL_UINT32(9500, c.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(9000, c.low_battery_voltage_mv);
  json = cJSON_Parse("{\"sleep_enabled\":false}");
  result = power_settings::patch(json, c);
  cJSON_Delete(json);
  TEST_ASSERT_NULL(result.error);
  TEST_ASSERT_EQUAL_UINT32(1, result.count);
  TEST_ASSERT_EQUAL_UINT32(9500, c.stay_awake_voltage_mv);
  TEST_ASSERT_FALSE(c.sleep_enabled);
}

void test_json_rejects_wrong_types_fractional_out_of_range_and_duplicate_fields()
{
  for (const char* text : {
    "{\"sleep_enabled\":0}", "{\"sleep_enabled\":1}", "{\"sleep_enabled\":\"false\"}",
    "{\"sleep_enabled\":null}", "{\"sleep_enabled\":[]}", "{\"sleep_enabled\":{}}",
    "{\"stay_awake_voltage_mv\":8999}", "{\"stay_awake_voltage_mv\":16001}",
    "{\"stay_awake_voltage_mv\":13200.1}", "{\"stay_awake_voltage_mv\":true}",
    "{\"stay_awake_voltage_mv\":\"13200\"}", "{\"stay_awake_voltage_mv\":null}",
    "{\"low_battery_voltage_mv\":8999}", "{\"low_battery_voltage_mv\":16001}",
    "{\"low_battery_voltage_mv\":11400.5}", "{\"low_battery_voltage_mv\":false}",
    "{\"low_battery_voltage_mv\":\"11400\"}", "{\"low_battery_voltage_mv\":null}",
    "{\"low_battery_voltage_mv\":13200}", "{\"low_battery_voltage_mv\":14000}",
    "{\"sleep_enabled\":true,\"sleep_enabled\":false}",
    "{\"stay_awake_voltage_mv\":14000,\"stay_awake_voltage_mv\":15000}",
    "{\"low_battery_voltage_mv\":10000,\"low_battery_voltage_mv\":11000}",
    "{\"pid_def_path\":\"/sdcard/good.json\",\"sleep_enabled\":0}", "[]", "null"})
  {
    power_settings::Config c;
    cJSON* json = cJSON_Parse(text);
    const auto result = power_settings::patch(json, c);
    cJSON_Delete(json);
    TEST_ASSERT_NOT_NULL_MESSAGE(result.error, text);
    TEST_ASSERT_EQUAL_UINT32(0, result.count);
    TEST_ASSERT_EQUAL_UINT32(13200, c.stay_awake_voltage_mv);
    TEST_ASSERT_EQUAL_UINT32(11400, c.low_battery_voltage_mv);
    TEST_ASSERT_TRUE(c.sleep_enabled);
  }
  const double nonfinite[] = {NAN, INFINITY, -INFINITY, 1e100};
  for (double number : nonfinite)
  {
    cJSON* json = cJSON_CreateObject();
    TEST_ASSERT_NOT_NULL(json);
    TEST_ASSERT_NOT_NULL(cJSON_AddNumberToObject(json, "stay_awake_voltage_mv", number));
    power_settings::Config c;
    const auto result = power_settings::patch(json, c);
    cJSON_Delete(json);
    TEST_ASSERT_NOT_NULL(result.error);
  }
}

void test_get_json_contract_round_trips_and_unknown_old_fields_are_not_consumed()
{
  power_settings::Config c{15999, 9001, false};
  cJSON* json = cJSON_CreateObject();
  TEST_ASSERT_NOT_NULL(json);
  TEST_ASSERT_TRUE(power_settings::append(json, c));
  TEST_ASSERT_TRUE(cJSON_IsNumber(cJSON_GetObjectItemCaseSensitive(json, "stay_awake_voltage_mv")));
  TEST_ASSERT_TRUE(cJSON_IsBool(cJSON_GetObjectItemCaseSensitive(json, "sleep_enabled")));
  power_settings::Config copy;
  const auto result = power_settings::patch(json, copy);
  cJSON_Delete(json);
  TEST_ASSERT_NULL(result.error);
  TEST_ASSERT_EQUAL_UINT32(15999, copy.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(9001, copy.low_battery_voltage_mv);
  TEST_ASSERT_FALSE(copy.sleep_enabled);
  json = cJSON_Parse("{\"pid_def_path\":\"/sdcard/a.json\",\"unknown\":123}");
  const auto untouched = power_settings::patch(json, copy);
  cJSON_Delete(json);
  TEST_ASSERT_NULL(untouched.error);
  TEST_ASSERT_EQUAL_UINT32(0, untouched.count);
  TEST_ASSERT_EQUAL_UINT32(15999, copy.stay_awake_voltage_mv);
}

void test_power_storage_writes_typed_keys_and_validates_before_any_write()
{
  Stored s;
  unsigned writes = 0;
  auto set32 = [&](const char* key, uint32_t value)
  {
    ++writes;
    if (strcmp(key, "p_awake_mv") == 0) s.awake = value;
    else
    {
      TEST_ASSERT_EQUAL_STRING("p_low_mv", key);
      s.low = value;
    }
    return ESP_OK;
  };
  auto set8 = [&](const char* key, uint8_t value)
  {
    ++writes;
    TEST_ASSERT_EQUAL_STRING("p_sleep", key);
    s.enabled = value;
    return ESP_OK;
  };
  const power_settings::Config c{16000, 9000, false};
  TEST_ASSERT_EQUAL_INT(ESP_OK, power_settings::write(c, set32, set8));
  TEST_ASSERT_EQUAL_UINT32(3, writes);
  power_settings::Config loaded;
  TEST_ASSERT_EQUAL_INT(ESP_OK, s.load(loaded));
  TEST_ASSERT_EQUAL_UINT32(16000, loaded.stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(9000, loaded.low_battery_voltage_mv);
  TEST_ASSERT_FALSE(loaded.sleep_enabled);
  writes = 0;
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, power_settings::write({16000, 16000, true}, set32, set8));
  TEST_ASSERT_EQUAL_UINT32(0, writes);
}

void test_power_storage_failure_stops_further_writes()
{
  for (unsigned fail = 1; fail <= 3; ++fail)
  {
    unsigned writes = 0;
    auto error = [&]() { return ++writes == fail ? ESP_FAIL : ESP_OK; };
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, power_settings::write({},
      [&](const char*, uint32_t) { return error(); }, [&](const char*, uint8_t) { return error(); }));
    TEST_ASSERT_EQUAL_UINT32(fail, writes);
  }
}
}

extern "C" void run_power_settings_tests()
{
  RUN_TEST(test_stored_power_fields_are_loaded_without_losing_integer_precision);
  RUN_TEST(test_legacy_missing_keys_use_defaults_independently);
  RUN_TEST(test_unusable_keys_default_individually_and_inconsistent_pair_defaults_together);
  RUN_TEST(test_operational_power_read_errors_degrade_to_defaults_but_partition_corruption_propagates);
  RUN_TEST(test_json_patch_updates_exact_integer_millivolts_and_boolean);
  RUN_TEST(test_json_partial_patch_validates_the_final_pair_without_mutating_on_error);
  RUN_TEST(test_json_rejects_wrong_types_fractional_out_of_range_and_duplicate_fields);
  RUN_TEST(test_get_json_contract_round_trips_and_unknown_old_fields_are_not_consumed);
  RUN_TEST(test_power_storage_writes_typed_keys_and_validates_before_any_write);
  RUN_TEST(test_power_storage_failure_stops_further_writes);
}
