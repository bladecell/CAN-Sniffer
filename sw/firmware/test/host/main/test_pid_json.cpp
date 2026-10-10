// PID definition JSON: validation, add / edit / remove through the data model, and an exact round trip (OBD-5, OBD-6).
#include <cstdlib>
#include <cstring>
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
    esp_err_t replace(const std::vector<PIDDefinitionData>& d)
    {
        return replacePIDDefinitions(d, {}, {});
    }
};

// Canonical text: keys in serializer order, definitions sorted by PID, numbers as cJSON prints them.
const char* CANONICAL =
    "[{\"id\":2015,\"mode\":1,\"pid\":12,\"length\":2,\"name\":\"RPM\",\"unit\":\"rpm\",\"desc\":\"Engine speed\","
    "\"formula\":\"(A*256+B)/4\",\"minV\":0,\"maxV\":8000,\"priority\":1,\"interval\":100,\"color\":5157659,"
    "\"icon\":\"gauge\"},"
    "{\"id\":2015,\"mode\":1,\"pid\":13,\"length\":2,\"name\":\"Speed\",\"unit\":\"km/h\",\"desc\":\"\","
    "\"formula\":\"A\",\"minV\":-0.5,\"maxV\":255.25,\"priority\":2,\"interval\":0,\"color\":0,\"icon\":\"\"},"
    "{\"id\":2016,\"mode\":69,\"pid\":4660,\"length\":0,\"name\":\"Derived\",\"unit\":\"\",\"desc\":\"\","
    "\"formula\":\"getPID(12)/2\",\"minV\":0,\"maxV\":1,\"priority\":0,\"interval\":1000,\"color\":1,\"icon\":\"\"},"
    "{\"id\":2016,\"mode\":34,\"pid\":61840,\"length\":3,\"name\":\"Custom\",\"unit\":\"V\",\"desc\":\"x\","
    "\"formula\":\"(A*256+B)/10\",\"minV\":0.25,\"maxV\":20,\"priority\":3,\"interval\":500,\"color\":16777215,"
    "\"icon\":\"bolt\"}]";

std::string parseAndSerialize(const char* text, std::vector<PIDDefinitionData>* keep = nullptr)
{
    cJSON* root = cJSON_Parse(text);
    TEST_ASSERT_NOT_NULL(root);
    std::vector<PIDDefinitionData> defs;
    pidjson::ParseResult           r = pidjson::parse(root, defs);
    TEST_ASSERT_TRUE_MESSAGE(r.ok, r.reason ? r.reason : "");
    cJSON_Delete(root);
    cJSON* out = pidjson::serialize(defs);
    TEST_ASSERT_NOT_NULL(out);
    char* s = cJSON_PrintUnformatted(out);
    cJSON_Delete(out);
    TEST_ASSERT_NOT_NULL(s);
    std::string result(s);
    free(s);
    if (keep != nullptr)
    {
        *keep = defs;
    }
    return result;
}

pidjson::ParseResult parseText(const char* text)
{
    cJSON* root = cJSON_Parse(text);
    TEST_ASSERT_NOT_NULL(root);
    std::vector<PIDDefinitionData> defs;
    pidjson::ParseResult           r = pidjson::parse(root, defs);
    cJSON_Delete(root);
    return r;
}

// One valid definition with a single field replaced, to probe each validation rule.
std::string withField(const char* key, const char* jsonValue)
{
    std::string fields[][2] = {
        {"id", "2015"},        {"mode", "1"},        {"pid", "12"},        {"length", "2"},
        {"name", "\"n\""},     {"unit", "\"u\""},    {"desc", "\"d\""},    {"formula", "\"A\""},
        {"minV", "0"},         {"maxV", "10"},       {"priority", "1"},    {"interval", "100"},
        {"color", "0"},        {"icon", "\"i\""},
    };
    std::string out = "[{";
    bool        first = true;
    for (auto& f : fields)
    {
        if (!first)
        {
            out += ",";
        }
        first = false;
        out += "\"" + f[0] + "\":" + (f[0] == key ? std::string(jsonValue) : f[1]);
    }
    return out + "}]";
}
}  // namespace

static void test_round_trip_is_byte_identical(void)
{
    std::vector<PIDDefinitionData> defs;
    const std::string              first = parseAndSerialize(CANONICAL, &defs);
    TEST_ASSERT_EQUAL_STRING(CANONICAL, first.c_str());
    TEST_ASSERT_EQUAL_STRING(first.c_str(), parseAndSerialize(first.c_str()).c_str());
}

static void test_float_values_are_stable_after_the_first_save(void)
{
    // 0.1 is not exact as a float, so the first save prints the float's value; every later save is identical.
    const std::string base   = withField("minV", "0.1");
    const std::string first  = parseAndSerialize(base.c_str());
    const std::string second = parseAndSerialize(first.c_str());
    TEST_ASSERT_EQUAL_STRING(first.c_str(), second.c_str());
}

static void test_round_trip_through_the_data_model(void)
{
    std::vector<PIDDefinitionData> defs;
    parseAndSerialize(CANONICAL, &defs);

    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replace(defs));
    TEST_ASSERT_EQUAL_UINT32(4, m.getPIDDEFSize());

    std::vector<PIDDefinitionData> snap;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    cJSON* out = pidjson::serialize(snap);
    char*  s   = cJSON_PrintUnformatted(out);
    TEST_ASSERT_EQUAL_STRING(CANONICAL, s);
    free(s);
    cJSON_Delete(out);
}

static void test_add_edit_remove_persist_as_json(void)
{
    std::vector<PIDDefinitionData> defs;
    parseAndSerialize(CANONICAL, &defs);
    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replace(defs));

    // add (single PID)
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.addPID(2015, 1, 0x05, 2, "Coolant", "C", "", "A-40", -40.f, 215.f, 1, 1000, 0, ""));
    TEST_ASSERT_EQUAL_UINT32(5, m.getPIDDEFSize());

    // edit: replace the definition set with the name and formula of PID 12 changed
    std::vector<PIDDefinitionData> snap;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    for (auto& d : snap)
    {
        if (d.pid == 12)
        {
            d.name    = "Engine RPM";
            d.formula = "(A*256+B)/8";
        }
    }
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replace(snap));
    PIDDefinitionData edited;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDef(12, edited));
    TEST_ASSERT_EQUAL_STRING("Engine RPM", edited.name.c_str());
    TEST_ASSERT_EQUAL_STRING("(A*256+B)/8", edited.formula.c_str());

    // remove
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.removePID(5));
    TEST_ASSERT_EQUAL_UINT32(4, m.getPIDDEFSize());

    // the saved JSON reloads to the same definitions, and a second save is identical
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDefinitionSnapshot(snap));
    cJSON* saved = pidjson::serialize(snap);
    char*  text  = cJSON_PrintUnformatted(saved);
    cJSON_Delete(saved);
    std::string reloaded = parseAndSerialize(text);
    TEST_ASSERT_EQUAL_STRING(text, reloaded.c_str());
    free(text);
}

static void test_replace_rejects_bad_formula_and_keeps_old_set(void)
{
    std::vector<PIDDefinitionData> defs;
    parseAndSerialize(CANONICAL, &defs);
    Model m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.replace(defs));

    std::vector<PIDDefinitionData> bad = defs;
    bad[0].formula                     = "2+*";
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, m.replace(bad));
    TEST_ASSERT_EQUAL_UINT32(4, m.getPIDDEFSize());
    PIDDefinitionData d;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.getDef(12, d));
    TEST_ASSERT_EQUAL_STRING("(A*256+B)/4", d.formula.c_str());
}

static void test_len_alias_accepted_and_conflict_rejected(void)
{
    TEST_ASSERT_TRUE(parseText(withField("length", "2").c_str()).ok);
    std::string alias = withField("length", "2");
    alias.replace(alias.find("\"length\""), 8, "\"len\"");
    TEST_ASSERT_TRUE(parseText(alias.c_str()).ok);
    std::string both = withField("length", "2");
    both.insert(both.size() - 2, ",\"len\":3");
    TEST_ASSERT_FALSE(parseText(both.c_str()).ok);
}

static void test_hostile_fields_are_rejected(void)
{
    const char* bad[][2] = {
        {"id", "2048"},          {"id", "-1"},          {"id", "1.5"},          {"id", "\"7DF\""},
        {"mode", "256"},         {"mode", "2"},         {"pid", "65536"},       {"pid", "0"},
        {"pid", "256"},          {"length", "3"},       {"length", "9"},        {"length", "-1"},
        {"priority", "256"},     {"interval", "1"},     {"interval", "65536"},  {"color", "16777216"},
        {"name", "5"},           {"formula", "\"\""},   {"formula", "null"},    {"minV", "1e999"},
        {"maxV", "-1e999"},      {"minV", "20"},        {"maxV", "\"x\""},      {"icon", "[]"},
    };
    for (auto& b : bad)
    {
        const std::string text = withField(b[0], b[1]);
        TEST_ASSERT_FALSE_MESSAGE(parseText(text.c_str()).ok, text.c_str());
    }
    // names above the limit
    const std::string longName = withField("name", ("\"" + std::string(65, 'x') + "\"").c_str());
    TEST_ASSERT_FALSE(parseText(longName.c_str()).ok);
    const std::string longFormula = withField("formula", ("\"" + std::string(1025, 'A') + "\"").c_str());
    TEST_ASSERT_FALSE(parseText(longFormula.c_str()).ok);
}

static void test_hostile_structure_is_rejected(void)
{
    TEST_ASSERT_EQUAL_INT(400, parseText("{}").httpStatus);
    TEST_ASSERT_EQUAL_INT(400, parseText("[1,2]").httpStatus);
    TEST_ASSERT_EQUAL_INT(400, parseText("\"s\"").httpStatus);
    TEST_ASSERT_EQUAL_INT(422, parseText("[{}]").httpStatus);
    std::vector<PIDDefinitionData> sink;
    TEST_ASSERT_FALSE(pidjson::parse(nullptr, sink).ok);

    std::string two = withField("pid", "12");
    two.pop_back();
    two += "," + withField("pid", "12").substr(1);  // duplicate PID
    TEST_ASSERT_FALSE(parseText(two.c_str()).ok);

    std::string many = "[";
    for (int i = 0; i < 257; i++)
    {
        many += std::string(i ? "," : "") + "{\"id\":1,\"mode\":34,\"pid\":" + std::to_string(1000 + i) +
                ",\"length\":3,\"name\":\"n\",\"unit\":\"\",\"desc\":\"\",\"formula\":\"A\",\"minV\":0,"
                "\"maxV\":1,\"priority\":0,\"interval\":0,\"color\":0,\"icon\":\"\"}";
    }
    many += "]";
    TEST_ASSERT_EQUAL_INT(422, parseText(many.c_str()).httpStatus);  // above NUMBER_OF_ITEMS
}

static void test_deeply_nested_and_truncated_text_is_a_parse_error_not_a_crash(void)
{
    // A hostile body of nested brackets must fail at CONFIG_CJSON_NESTING_LIMIT (32), long before it can use the
    // stack of the HTTP or main task. With the IDF default of 1000 this test crashed with SIGSEGV.
    TEST_ASSERT_NULL(cJSON_Parse(std::string(5000, '[').c_str()));
    TEST_ASSERT_NULL(cJSON_Parse((std::string(40, '[') + std::string(40, ']')).c_str()));
    cJSON* ok = cJSON_Parse((std::string(8, '[') + std::string(8, ']')).c_str());
    TEST_ASSERT_NOT_NULL(ok);
    cJSON_Delete(ok);
    TEST_ASSERT_NULL(cJSON_Parse("[{\"id\":"));
    TEST_ASSERT_NULL(cJSON_Parse(""));
}

extern "C" void run_pid_json_tests(void)
{
    RUN_TEST(test_round_trip_is_byte_identical);
    RUN_TEST(test_float_values_are_stable_after_the_first_save);
    RUN_TEST(test_round_trip_through_the_data_model);
    RUN_TEST(test_add_edit_remove_persist_as_json);
    RUN_TEST(test_replace_rejects_bad_formula_and_keeps_old_set);
    RUN_TEST(test_len_alias_accepted_and_conflict_rejected);
    RUN_TEST(test_hostile_fields_are_rejected);
    RUN_TEST(test_hostile_structure_is_rejected);
    RUN_TEST(test_deeply_nested_and_truncated_text_is_a_parse_error_not_a_crash);
}
