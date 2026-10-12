// PID database and frame-to-value path through OBD2DataModel (OBD-1, OBD-2, OBD-5/6 add and remove).
#include <cmath>
#include <cstring>

#include "obd2_data_model.hpp"
#include "unity.h"

namespace
{
class TestModel : public OBD2DataModel
{
public:
    TestModel()
    {
        pidMapMtx        = xSemaphoreCreateMutex();
        subscribers_mtx_ = xSemaphoreCreateMutex();
        vinData.mtx_     = xSemaphoreCreateMutex();
        dtcData.mtx_     = xSemaphoreCreateMutex();
        TEST_ASSERT_EQUAL_INT(ESP_OK, initDef());
    }
    esp_err_t add(uint8_t mode, uint16_t pid, uint8_t len, const char* formula)
    {
        return addPID(1, mode, pid, len, "n", "u", "d", formula, 0.f, 100.f, 1, 100, 0, "i");
    }
};

CanDriver::CanFrame canFrame(std::initializer_list<uint8_t> bytes, size_t length = 8)
{
    CanDriver::CanFrame f = {};
    size_t              i = 0;
    for (uint8_t b : bytes)
    {
        f.data[i++] = b;
    }
    f.length = length;
    return f;
}
}  // namespace

static void test_mode01_frame_updates_value(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x0C, 2, "(A*256+B)/4"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.updateData(canFrame({0x04, 0x41, 0x0C, 0x1A, 0xF8, 0x55, 0x55, 0x55})));
    TEST_ASSERT_EQUAL_FLOAT(1726.f, m.getValue(0x0C));
}

static void test_mode22_frame_updates_value(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x22, 0xF190, 3, "A*256+B"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.updateData(canFrame({0x05, 0x62, 0xF1, 0x90, 0x01, 0x02, 0x55, 0x55})));
    TEST_ASSERT_EQUAL_FLOAT(258.f, m.getValue(0xF190));
}

static void test_padding_bytes_do_not_leak_into_formula(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x0D, 2, "C+D"));
    // One data byte (0x40) then 0x55 padding: C and D are not in the frame, so it is rejected, not 0xAA.
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x03, 0x41, 0x0D, 0x40, 0x55, 0x55, 0x55, 0x55})));
    TEST_ASSERT_EQUAL_FLOAT(0.f, m.getValue(0x0D));
    // With all four bytes present the formula runs.
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.updateData(canFrame({0x06, 0x41, 0x0D, 1, 2, 3, 4, 0x55})));
    TEST_ASSERT_EQUAL_FLOAT(7.f, m.getValue(0x0D));
}

static void test_malformed_frames_are_rejected(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x0C, 2, "A"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x22, 0xF190, 3, "A"));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x04, 0x41, 0x0C, 0x10}, 2)));      // DLC 2
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x04, 0x41, 0x0C, 0x10}, 200)));    // DLC above 8
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x02, 0x62, 0xF1}, 3)));            // 0x62 without full PID
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x00, 0x41, 0x0C, 0x10})));         // PCI length 0
    TEST_ASSERT_TRUE(std::isnan(m.getValue(0x0C)) || m.getValue(0x0C) == 0.f);
    TEST_ASSERT_EQUAL_FLOAT(0.f, m.getValue(0xF190));
}

static void test_frame_without_the_bytes_the_formula_reads_is_rejected(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x0C, 2, "(A*256+B)/4"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x0D, 2, "getBit(A,3)"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x0E, 2, "getPID(12)+1"));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x02, 0x41, 0x0C})));              // no data bytes
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x03, 0x41, 0x0C, 0x1A})));        // B missing
    TEST_ASSERT_EQUAL_FLOAT(0.f, m.getValue(0x0C));                                         // nothing stored
    TEST_ASSERT_FALSE(m.isValid(0x0C));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.updateData(canFrame({0x04, 0x41, 0x0C, 0x1A, 0xF8})));  // both present
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x02, 0x41, 0x0D})));              // A missing
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.updateData(canFrame({0x03, 0x41, 0x0D, 0x08})));        // A only, enough
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.updateData(canFrame({0x02, 0x41, 0x0E})));              // reads no frame bytes
}

static void test_unknown_pid_is_not_found(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_FOUND, m.updateData(canFrame({0x04, 0x41, 0x99, 1, 2})));
}

static void test_bad_formula_or_nan_never_stores_a_value(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x10, 2, "2+*"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x11, 2, "A/(B-B)"));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x04, 0x41, 0x10, 5, 6})));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, m.updateData(canFrame({0x04, 0x41, 0x11, 5, 6})));
    TEST_ASSERT_EQUAL_FLOAT(0.f, m.getValue(0x10));
    TEST_ASSERT_EQUAL_FLOAT(0.f, m.getValue(0x11));
}

static void test_add_duplicate_and_remove(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.add(0x01, 0x0C, 2, "A"));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, m.add(0x01, 0x0C, 2, "B"));
    TEST_ASSERT_TRUE(m.pidExists(0x0C));
    TEST_ASSERT_EQUAL_INT(ESP_OK, m.removePID(0x0C));
    TEST_ASSERT_FALSE(m.pidExists(0x0C));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_FOUND, m.removePID(0x0C));
}

static void test_decode_dtc(void)
{
    TestModel m;
    TEST_ASSERT_EQUAL_STRING("P0133", m.decodeDTC(0x0133).c_str());
    TEST_ASSERT_EQUAL_STRING("P2100", m.decodeDTC(0x2100).c_str());
    TEST_ASSERT_EQUAL_STRING("P3001", m.decodeDTC(0x3001).c_str());
    TEST_ASSERT_EQUAL_STRING("C0123", m.decodeDTC(0x4123).c_str());
    TEST_ASSERT_EQUAL_STRING("B0001", m.decodeDTC(0x8001).c_str());
    TEST_ASSERT_EQUAL_STRING("U0100", m.decodeDTC(0xC100).c_str());
    TEST_ASSERT_EQUAL_STRING("U3FFF", m.decodeDTC(0xFFFF).c_str());
    TEST_ASSERT_EQUAL_STRING("No DTC", m.decodeDTC(0).c_str());
}

static PollRequest recurringReq(uint32_t id, uint8_t mode, uint16_t pid)
{
    PollRequest r = {};
    r.id          = id;
    r.isRecurring = true;
    r.isRaw       = false;
    r.interval    = 100;
    r.payload.obd.mode = mode;
    r.payload.obd.pid  = pid;
    return r;
}

static void test_push_recurring_unique_skips_a_queued_duplicate(void)
{
    PIDPriorityQueue q;
    TEST_ASSERT_TRUE(q.pushRecurringUnique(recurringReq(0x7E0, 1, 0x0C)));
    TEST_ASSERT_TRUE(q.pushRecurringUnique(recurringReq(0x7E0, 1, 0x0C)));  // in-flight copy re-added after a refill
    TEST_ASSERT_TRUE(q.pushRecurringUnique(recurringReq(0x7E0, 1, 0x0D)));
    TEST_ASSERT_TRUE(q.pushRecurringUnique(recurringReq(0x7E1, 1, 0x0C)));  // other ECU is a different key
    int n = 0;
    PollRequest out;
    while (q.tryPop(out)) { n++; }
    TEST_ASSERT_EQUAL_INT(3, n);
}

extern "C" void run_obd2_model_tests(void)
{
    RUN_TEST(test_mode01_frame_updates_value);
    RUN_TEST(test_mode22_frame_updates_value);
    RUN_TEST(test_padding_bytes_do_not_leak_into_formula);
    RUN_TEST(test_malformed_frames_are_rejected);
    RUN_TEST(test_frame_without_the_bytes_the_formula_reads_is_rejected);
    RUN_TEST(test_unknown_pid_is_not_found);
    RUN_TEST(test_bad_formula_or_nan_never_stores_a_value);
    RUN_TEST(test_add_duplicate_and_remove);
    RUN_TEST(test_decode_dtc);
    RUN_TEST(test_push_recurring_unique_skips_a_queued_duplicate);
}
