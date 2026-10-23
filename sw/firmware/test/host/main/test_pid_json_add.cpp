// POST /pid_def item validation (pidjson::parseAddItem): same limits as PUT and the SD load, looser only where documented.
#include <cmath>
#include <string>

#include "obd2_data_model.hpp"
#include "pid_json.hpp"
#include "unity.h"

namespace
{
class Model : public OBD2DataModel
{
public:
    Model()
    {
        pidMapMtx        = xSemaphoreCreateMutex();
        subscribers_mtx_ = xSemaphoreCreateMutex();
        vinData.mtx_     = xSemaphoreCreateMutex();
        dtcData.mtx_     = xSemaphoreCreateMutex();
        TEST_ASSERT_EQUAL_INT(ESP_OK, initDef());
    }
};

bool addItem(const std::string& text, PIDDefinitionData* keep = nullptr)
{
    cJSON* item = cJSON_Parse(text.c_str());
    TEST_ASSERT_NOT_NULL(item);
    PIDDefinitionData def = {};
    const bool        ok  = pidjson::parseAddItem(item, def);
    cJSON_Delete(item);
    if (keep != nullptr)
    {
        *keep = def;
    }
    return ok;
}

// A valid minimal POST item, with optional extra members appended and `name` / `formula` overridable.
std::string item(const std::string& extra = "", const std::string& name = "n", const std::string& formula = "A")
{
    return "{\"id\":2015,\"mode\":1,\"pid\":12,\"interval\":100,\"name\":\"" + name + "\",\"formula\":\"" + formula +
           "\"" + (extra.empty() ? "" : "," + extra) + "}";
}
}  // namespace

static void test_minimal_item_gets_documented_defaults(void)
{
    PIDDefinitionData d = {};
    TEST_ASSERT_TRUE(addItem(item(), &d));
    TEST_ASSERT_EQUAL_UINT8(2, d.len);
    TEST_ASSERT_EQUAL_UINT8(0, d.priority);
    TEST_ASSERT_EQUAL_UINT32(0x4EB31B, d.color);
    TEST_ASSERT_TRUE(std::isnan(d.minValue));
    TEST_ASSERT_TRUE(std::isnan(d.maxValue));
    TEST_ASSERT_EQUAL_STRING("", d.unit.c_str());
    TEST_ASSERT_TRUE(addItem("{\"id\":1,\"mode\":34,\"pid\":61840,\"interval\":0,\"name\":\"c\",\"formula\":\"A\"}", &d));
    TEST_ASSERT_EQUAL_UINT8(3, d.len);
    TEST_ASSERT_TRUE(addItem("{\"id\":1,\"mode\":69,\"pid\":4660,\"interval\":0,\"name\":\"c\",\"formula\":\"1\"}", &d));
    TEST_ASSERT_EQUAL_UINT8(0, d.len);
}

static void test_string_lengths_are_capped_and_boundaries_pass(void)
{
    TEST_ASSERT_TRUE(addItem(item("", std::string(64, 'n'))));
    TEST_ASSERT_FALSE(addItem(item("", std::string(65, 'n'))));
    TEST_ASSERT_TRUE(addItem(item("", "n", std::string(1024, 'A'))));
    TEST_ASSERT_FALSE(addItem(item("", "n", std::string(1025, 'A'))));
    TEST_ASSERT_TRUE(addItem(item("\"unit\":\"" + std::string(32, 'u') + "\"")));
    TEST_ASSERT_FALSE(addItem(item("\"unit\":\"" + std::string(33, 'u') + "\"")));
    TEST_ASSERT_TRUE(addItem(item("\"desc\":\"" + std::string(256, 'd') + "\"")));
    TEST_ASSERT_FALSE(addItem(item("\"desc\":\"" + std::string(257, 'd') + "\"")));
    TEST_ASSERT_TRUE(addItem(item("\"icon\":\"" + std::string(64, 'i') + "\"")));
    TEST_ASSERT_FALSE(addItem(item("\"icon\":\"" + std::string(65, 'i') + "\"")));
}

static void test_empty_formula_and_missing_required_are_rejected(void)
{
    TEST_ASSERT_FALSE(addItem(item("", "n", "")));
    TEST_ASSERT_TRUE(addItem(item("", "")));  // empty name stays allowed, as in PUT
    TEST_ASSERT_FALSE(addItem("{\"id\":1,\"mode\":1,\"pid\":12,\"interval\":100,\"name\":\"n\"}"));
    TEST_ASSERT_FALSE(addItem("{\"id\":1,\"mode\":1,\"pid\":12,\"name\":\"n\",\"formula\":\"A\"}"));
    TEST_ASSERT_FALSE(addItem("[]"));
}

static void test_invalid_spec_and_length_are_rejected(void)
{
    TEST_ASSERT_FALSE(addItem(item("\"length\":3")));  // mode 1 needs 2
    TEST_ASSERT_FALSE(addItem(item("\"length\":2,\"len\":3")));
    TEST_ASSERT_TRUE(addItem(item("\"len\":2")));
    TEST_ASSERT_FALSE(addItem("{\"id\":1,\"mode\":9,\"pid\":12,\"interval\":100,\"name\":\"n\",\"formula\":\"A\"}"));
    TEST_ASSERT_FALSE(addItem("{\"id\":1,\"mode\":1,\"pid\":0,\"interval\":100,\"name\":\"n\",\"formula\":\"A\"}"));
    TEST_ASSERT_FALSE(addItem("{\"id\":1,\"mode\":1,\"pid\":256,\"interval\":100,\"name\":\"n\",\"formula\":\"A\"}"));
}

static void test_numbers_and_range_are_checked(void)
{
    TEST_ASSERT_FALSE(addItem(item("\"minV\":5,\"maxV\":1")));
    TEST_ASSERT_TRUE(addItem(item("\"minV\":null,\"maxV\":1")));
    TEST_ASSERT_FALSE(addItem(item("\"color\":16777216")));
    TEST_ASSERT_FALSE(addItem(item("\"priority\":256")));
    TEST_ASSERT_FALSE(addItem(item("\"priority\":1.5")));
    TEST_ASSERT_FALSE(addItem("{\"id\":2048,\"mode\":1,\"pid\":12,\"interval\":100,\"name\":\"n\",\"formula\":\"A\"}"));
}

static void test_short_interval_is_still_accepted_here_and_clamped_by_the_queue(void)
{
    PIDDefinitionData d = {};
    TEST_ASSERT_TRUE(addItem("{\"id\":1,\"mode\":1,\"pid\":12,\"interval\":1,\"name\":\"n\",\"formula\":\"A\"}", &d));
    TEST_ASSERT_EQUAL_UINT16(1, d.updateInterval_ms);
}

static void test_duplicate_pid_in_one_batch_fails_the_second_item(void)
{
    Model             m;
    PIDDefinitionData a = {}, b = {};
    TEST_ASSERT_TRUE(addItem(item(), &a));
    TEST_ASSERT_TRUE(addItem(item("", "other"), &b));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.addPID(a.id, a.mode, a.pid, a.len, a.name, a.unit, a.description, a.formula,
                                           a.minValue, a.maxValue, a.priority, a.updateInterval_ms, a.color, a.icon));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          m.addPID(b.id, b.mode, b.pid, b.len, b.name, b.unit, b.description, b.formula, b.minValue,
                                   b.maxValue, b.priority, b.updateInterval_ms, b.color, b.icon));
    TEST_ASSERT_EQUAL_UINT32(1, m.getPIDDEFSize());
}

static void test_wrong_type_strings_are_rejected_and_absent_or_null_default(void)
{
    for (const char* key : {"unit", "desc", "icon"})
    {
        TEST_ASSERT_FALSE(addItem(item(std::string("\"") + key + "\":5")));
        TEST_ASSERT_FALSE(addItem(item(std::string("\"") + key + "\":[]")));
        TEST_ASSERT_FALSE(addItem(item(std::string("\"") + key + "\":false")));
        PIDDefinitionData d = {};
        d.unit              = "x";
        TEST_ASSERT_TRUE(addItem(item(std::string("\"") + key + "\":null"), &d));
        TEST_ASSERT_EQUAL_STRING("", d.unit.c_str());
    }
    TEST_ASSERT_FALSE(addItem("{\"id\":1,\"mode\":1,\"pid\":12,\"interval\":100,\"name\":5,\"formula\":\"A\"}"));
    TEST_ASSERT_FALSE(addItem("{\"id\":1,\"mode\":1,\"pid\":12,\"interval\":100,\"name\":\"n\",\"formula\":null}"));
}

extern "C" void run_pid_json_add_tests(void)
{
    RUN_TEST(test_minimal_item_gets_documented_defaults);
    RUN_TEST(test_string_lengths_are_capped_and_boundaries_pass);
    RUN_TEST(test_empty_formula_and_missing_required_are_rejected);
    RUN_TEST(test_invalid_spec_and_length_are_rejected);
    RUN_TEST(test_numbers_and_range_are_checked);
    RUN_TEST(test_short_interval_is_still_accepted_here_and_clamped_by_the_queue);
    RUN_TEST(test_duplicate_pid_in_one_batch_fails_the_second_item);
    RUN_TEST(test_wrong_type_strings_are_rejected_and_absent_or_null_default);
}
