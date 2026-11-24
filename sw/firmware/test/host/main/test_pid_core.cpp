// Built-in core PID set: present with an empty DB, overridden by a file, restored after a PUT that omits it.
#include <cmath>
#include <string>

#include "obd2_data_model.hpp"
#include "pid_core.hpp"
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
    // OBD2::replacePIDDefinitions = pidcore::merged + the model swap (OBD2 itself needs the RTOS).
    esp_err_t replaceWithCore(const std::vector<PIDDefinitionData>& d)
    {
        std::vector<PIDDefinitionData> full;
        const esp_err_t                err = pidcore::merged(d, full, 256);
        return err != ESP_OK ? err : replacePIDDefinitions(full, {}, {});
    }
};

bool has(const std::vector<PIDDefinitionData>& snap, uint16_t pid, PIDDefinitionData* out = nullptr)
{
    for (const auto& d : snap)
    {
        if (d.pid == pid)
        {
            if (out != nullptr)
            {
                *out = d;
            }
            return true;
        }
    }
    return false;
}

std::vector<PIDDefinitionData> parseText(const char* text)
{
    cJSON* root = cJSON_Parse(text);
    TEST_ASSERT_NOT_NULL(root);
    std::vector<PIDDefinitionData> defs;
    TEST_ASSERT_TRUE(pidjson::parse(root, defs).ok);
    cJSON_Delete(root);
    return defs;
}

const uint16_t CORE[] = {0x04, 0x05, 0x0C, 0x0D, 0x0F, 0x10};
}  // namespace

static void test_empty_load_yields_core_set(void)
{
    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replaceWithCore({}));
    std::vector<PIDDefinitionData> snap;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT32(6, snap.size());
    for (uint16_t pid : CORE)
    {
        TEST_ASSERT_TRUE(has(snap, pid));
        TEST_ASSERT_TRUE(pidcore::isCore(pid));
    }
}

static void test_core_formulas_compile_and_evaluate(void)
{
    for (const auto& d : pidcore::definitions())
    {
        PIDDefinition def(d.id, d.mode, d.pid, d.len, d.name, d.unit, d.description, d.formula, d.minValue, d.maxValue,
                          d.priority, d.updateInterval_ms, d.color, d.icon);
        TEST_ASSERT_TRUE(def.formulaValid());
        TEST_ASSERT_EQUAL_UINT32(0x7DF, d.id);
        TEST_ASSERT_EQUAL_UINT8(2, d.len);
        TEST_ASSERT_TRUE(d.updateInterval_ms >= 4);
        TEST_ASSERT_TRUE(d.minValue <= d.maxValue);
    }
    auto eval = [](uint16_t pid, uint8_t a, uint8_t b)
    {
        for (const auto& d : pidcore::definitions())
        {
            if (d.pid != pid)
            {
                continue;
            }
            PIDDefinition def(d.id, d.mode, d.pid, d.len, d.name, d.unit, d.description, d.formula, d.minValue,
                              d.maxValue, d.priority, d.updateInterval_ms, d.color, d.icon);
            const uint8_t frame[2] = {a, b};
            float         r        = NAN;
            TEST_ASSERT_EQUAL_INT(ESP_OK, def.evaluate(frame, 2, r));
            return r;
        }
        return (float)NAN;
    };
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, eval(0x04, 255, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, eval(0x05, 90, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 800.0f, eval(0x0C, 0x0C, 0x80));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, eval(0x0D, 120, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -15.0f, eval(0x0F, 25, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.12f, eval(0x10, 2, 0));
}

static std::string coreJson(const char* rpmName = nullptr)
{
    cJSON* arr = pidjson::serialize(pidcore::definitions());
    TEST_ASSERT_NOT_NULL(arr);
    if (rpmName != nullptr)
    {
        cJSON* it = nullptr;
        cJSON_ArrayForEach(it, arr)
        {
            if (cJSON_GetObjectItem(it, "pid")->valueint == 0x0C)
            {
                cJSON_ReplaceItemInObject(it, "name", cJSON_CreateString(rpmName));
            }
        }
    }
    char* t = cJSON_PrintUnformatted(arr);
    std::string out(t);
    free(t);
    cJSON_Delete(arr);
    return out;
}

static void test_identical_core_in_a_set_is_accepted_round_trip_and_builtin_key_tolerated(void)
{
    Model m;
    std::string text = coreJson();
    size_t      pos  = 0;
    while ((pos = text.find("{\"id\"", pos)) != std::string::npos)
    {
        text.insert(pos + 1, "\"builtin\":true,");
        pos += 2;
    }
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replaceWithCore(parseText(text.c_str())));
    std::vector<PIDDefinitionData> snap;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT32(6, snap.size());
}

static void test_changed_core_item_rejects_the_whole_set(void)
{
    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replaceWithCore({}));
    auto defs = parseText(coreJson("Mine").c_str());
    defs.push_back(parseText(
        "[{\"id\":2016,\"mode\":34,\"pid\":61840,\"length\":3,\"name\":\"Custom\",\"unit\":\"\",\"desc\":\"\","
        "\"formula\":\"A\",\"minV\":0,\"maxV\":1,\"priority\":1,\"interval\":500,\"color\":0,\"icon\":\"\"}]")[0]);
    TEST_ASSERT_EQUAL_INT(pidcore::kBuiltin, m.replaceWithCore(defs));
    std::vector<PIDDefinitionData> snap;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT32(6, snap.size());
    PIDDefinitionData rpm = {};
    TEST_ASSERT_TRUE(has(snap, 0x0C, &rpm));
    TEST_ASSERT_EQUAL_STRING("Engine RPM", rpm.name.c_str());
}

static void test_core_pid_with_other_mode_is_a_change(void)
{
    auto defs = parseText(
        "[{\"id\":2016,\"mode\":34,\"pid\":12,\"length\":3,\"name\":\"x\",\"unit\":\"\",\"desc\":\"\","
        "\"formula\":\"A\",\"minV\":0,\"maxV\":1,\"priority\":1,\"interval\":500,\"color\":0,\"icon\":\"\"}]");
    TEST_ASSERT_EQUAL_INT(pidcore::kBuiltin, pidcore::fill(defs, 256));
    TEST_ASSERT_EQUAL_UINT32(1, defs.size());
}

static void test_put_omitting_core_keeps_core(void)
{
    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replaceWithCore(parseText("[]")));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replaceWithCore(parseText(
        "[{\"id\":2016,\"mode\":34,\"pid\":61840,\"length\":3,\"name\":\"Custom\",\"unit\":\"\",\"desc\":\"\","
        "\"formula\":\"A\",\"minV\":0,\"maxV\":1,\"priority\":1,\"interval\":500,\"color\":0,\"icon\":\"\"}]")));
    std::vector<PIDDefinitionData> snap;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT32(7, snap.size());
    for (uint16_t pid : CORE)
    {
        TEST_ASSERT_TRUE(has(snap, pid));
    }
}

static void test_boot_override_uses_repo_pid_def_shape(void)
{
    // Like sw/firmware/pid_def.json: "len" alias, spaced formula, different unit / interval / icon on core pids.
    auto defs = parseText(
        "[{\"id\":2015,\"mode\":1,\"pid\":4,\"len\":2,\"name\":\"Engine Load\",\"unit\":\"%\",\"desc\":\"d\","
        "\"formula\":\"A * 100 / 255\",\"minV\":0.0,\"maxV\":100.0,\"priority\":1,\"interval\":256,\"color\":1,"
        "\"icon\":\"x\"},"
        "{\"id\":2015,\"mode\":1,\"pid\":5,\"len\":2,\"name\":\"Coolant\",\"unit\":\"\\u00b0C\",\"desc\":\"\","
        "\"formula\":\"A - 40\",\"minV\":-40,\"maxV\":215,\"priority\":1,\"interval\":1000,\"color\":1,\"icon\":\"\"},"
        "{\"id\":2016,\"mode\":34,\"pid\":61840,\"length\":3,\"name\":\"Custom\",\"unit\":\"\",\"desc\":\"\","
        "\"formula\":\"A\",\"minV\":0,\"maxV\":1,\"priority\":1,\"interval\":500,\"color\":0,\"icon\":\"\"}]");
    // The API path rejects it.
    Model api;
    TEST_ASSERT_EQUAL_INT(pidcore::kBuiltin, api.replaceWithCore(defs));
    // The boot path replaces the two core items and keeps the custom one.
    std::vector<uint16_t> replaced;
    pidcore::overrideChanged(defs, replaced);
    TEST_ASSERT_EQUAL_UINT32(2, replaced.size());
    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replaceWithCore(defs));
    std::vector<PIDDefinitionData> snap;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    TEST_ASSERT_EQUAL_UINT32(7, snap.size());
    PIDDefinitionData load = {};
    TEST_ASSERT_TRUE(has(snap, 4, &load));
    TEST_ASSERT_EQUAL_UINT16(500, load.updateInterval_ms);
    TEST_ASSERT_TRUE(has(snap, 61840));
}

static std::vector<PIDDefinitionData> customSet(size_t n)
{
    std::vector<PIDDefinitionData> defs;
    for (size_t i = 0; i < n; i++)
    {
        PIDDefinitionData d = {};
        d.id                = 0x7DF;
        d.mode              = 0x22;
        d.pid               = (uint16_t)(0x1000 + i);
        d.len               = 3;
        d.formula           = "A";
        d.updateInterval_ms = 500;
        defs.push_back(d);
    }
    return defs;
}

static void test_capacity_250_custom_plus_core_passes_251_fails(void)
{
    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replaceWithCore(customSet(250)));
    TEST_ASSERT_EQUAL_UINT32(256, m.getPIDDEFSize());
    Model n;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_SIZE, n.replaceWithCore(customSet(251)));
}

static void test_guards_status_and_seed_rules(void)
{
    for (uint16_t pid : CORE)
    {
        TEST_ASSERT_EQUAL_INT(pidcore::kBuiltin, pidcore::guardChange(pid));  // removePID / addPID of a core pid
    }
    TEST_ASSERT_EQUAL_INT(ESP_OK, pidcore::guardChange(0x0B));
    TEST_ASSERT_EQUAL_INT(ESP_OK, pidcore::guardChange(61840));
    TEST_ASSERT_EQUAL_INT(403, pidcore::statusFor(pidcore::Op::Write));  // PUT, DELETE, load
    TEST_ASSERT_EQUAL_INT(422, pidcore::statusFor(pidcore::Op::Add));    // POST add
    TEST_ASSERT_EQUAL_STRING("PID_BUILTIN", pidcore::kCode);
    TEST_ASSERT_TRUE(pidcore::shouldSeed(ESP_ERR_NOT_FOUND));
    for (esp_err_t e : {ESP_OK, ESP_FAIL, ESP_ERR_INVALID_ARG, ESP_ERR_TIMEOUT, ESP_ERR_NO_MEM, pidcore::kBuiltin})
    {
        TEST_ASSERT_FALSE(pidcore::shouldSeed(e));
    }
}

static void test_fill_over_capacity_fails_and_leaves_input(void)
{
    std::vector<PIDDefinitionData> defs;
    for (uint16_t i = 0; i < 255; i++)
    {
        PIDDefinitionData d = {};
        d.pid               = 0x1000 + i;
        defs.push_back(d);
    }
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_SIZE, pidcore::fill(defs, 256));
    TEST_ASSERT_EQUAL_UINT32(255, defs.size());
}

extern "C" void run_pid_core_tests(void)
{
    RUN_TEST(test_boot_override_uses_repo_pid_def_shape);
    RUN_TEST(test_capacity_250_custom_plus_core_passes_251_fails);
    RUN_TEST(test_guards_status_and_seed_rules);
    RUN_TEST(test_empty_load_yields_core_set);
    RUN_TEST(test_core_formulas_compile_and_evaluate);
    RUN_TEST(test_identical_core_in_a_set_is_accepted_round_trip_and_builtin_key_tolerated);
    RUN_TEST(test_changed_core_item_rejects_the_whole_set);
    RUN_TEST(test_core_pid_with_other_mode_is_a_change);
    RUN_TEST(test_put_omitting_core_keeps_core);
    RUN_TEST(test_fill_over_capacity_fails_and_leaves_input);
}
