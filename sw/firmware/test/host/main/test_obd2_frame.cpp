// OBD-II / ISO-TP frame parsing (OBD-1, OBD-2) and DTC / VIN extraction.
#include <cstring>

#include "obd2_frame.hpp"
#include "unity.h"

using namespace obd2frame;

static RawFrame frame(std::initializer_list<uint8_t> bytes)
{
    RawFrame f = {};
    size_t   i = 0;
    for (uint8_t b : bytes)
    {
        f.data[i++] = b;
    }
    f.length = 8;  // CAN frames from the bus are padded to 8
    return f;
}

static void test_single_frame_mode01(void)
{
    RawFrame        f = frame({0x04, 0x41, 0x0C, 0x1A, 0xF8, 0x55, 0x55, 0x55});
    SingleFrameInfo i;
    TEST_ASSERT_EQUAL_INT(ESP_OK, parseSingleFrame(f.data, f.length, i));
    TEST_ASSERT_EQUAL_HEX8(0x41, i.mode);
    TEST_ASSERT_EQUAL_HEX16(0x0C, i.pid);
    TEST_ASSERT_EQUAL_UINT8(3, i.payloadOffset);
    TEST_ASSERT_EQUAL_UINT8(2, i.payloadLength);  // padding is not data
}

static void test_single_frame_mode22(void)
{
    RawFrame        f = frame({0x05, 0x62, 0xF1, 0x90, 0x12, 0x34, 0, 0});
    SingleFrameInfo i;
    TEST_ASSERT_EQUAL_INT(ESP_OK, parseSingleFrame(f.data, f.length, i));
    TEST_ASSERT_EQUAL_HEX8(0x62, i.mode);
    TEST_ASSERT_EQUAL_HEX16(0xF190, i.pid);
    TEST_ASSERT_EQUAL_UINT8(4, i.payloadOffset);
    TEST_ASSERT_EQUAL_UINT8(2, i.payloadLength);
}

static void test_single_frame_without_pid_modes(void)
{
    RawFrame        f = frame({0x03, 0x7F, 0x01, 0x12, 0, 0, 0, 0});
    SingleFrameInfo i;
    TEST_ASSERT_EQUAL_INT(ESP_OK, parseSingleFrame(f.data, f.length, i));
    TEST_ASSERT_EQUAL_HEX8(0x7F, i.mode);
}

static void test_clear_dtc_ack_is_two_bytes(void)
{
    RawFrame        f = frame({0x01, 0x44, 0, 0, 0, 0, 0, 0});
    SingleFrameInfo i;
    TEST_ASSERT_EQUAL_INT(ESP_OK, parseSingleFrame(f.data, f.length, i));
    TEST_ASSERT_EQUAL_HEX8(0x44, i.mode);
    TEST_ASSERT_EQUAL_UINT8(0, i.payloadLength);
}

static void test_single_frame_rejects_malformed(void)
{
    SingleFrameInfo i;
    uint8_t         ok[8] = {0x04, 0x41, 0x0C, 0x1A, 0xF8, 0, 0, 0};
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(nullptr, 8, i));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(ok, 0, i));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(ok, 2, i));    // too short
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(ok, 9, i));    // DLC above 8
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(ok, 255, i));
    uint8_t zeroLen[8] = {0x00, 0x41, 0x0C, 0, 0, 0, 0, 0};
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(zeroLen, 8, i));
    uint8_t bigLen[8] = {0x08, 0x41, 0x0C, 0, 0, 0, 0, 0};  // PCI length 8 cannot fit a single frame
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(bigLen, 8, i));
    uint8_t beyond[8] = {0x07, 0x41, 0x0C, 0, 0, 0, 0, 0};
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(beyond, 5, i));  // PCI says 7 bytes, frame has 4
    uint8_t rdbiShort[8] = {0x02, 0x62, 0xF1, 0, 0, 0, 0, 0};       // 0x62 needs a 2-byte PID
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(rdbiShort, 8, i));
    uint8_t notSingle[8] = {0x10, 0x14, 0x49, 0x02, 0x01, 0, 0, 0};
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseSingleFrame(notSingle, 8, i));
}

static void test_internal_derived_frame_is_accepted(void)
{
    // OBD2::receiveTask synthesizes {0x03, 0x85, pidHi, pidLo}, length 4.
    uint8_t         d[8] = {0x03, 0x85, 0x12, 0x34, 0, 0, 0, 0};
    SingleFrameInfo i;
    TEST_ASSERT_EQUAL_INT(ESP_OK, parseSingleFrame(d, 4, i));
    TEST_ASSERT_EQUAL_HEX16(0x1234, i.pid);
    TEST_ASSERT_EQUAL_UINT8(0, i.payloadLength);
}

static void test_first_frame_lengths(void)
{
    uint16_t total = 0, needed = 0;
    uint8_t  ff20[8] = {0x10, 0x14, 0x49, 0x02, 0x01, 0x57, 0x56, 0x57};
    TEST_ASSERT_EQUAL_INT(ESP_OK, parseFirstFrame(ff20, 8, total, needed));
    TEST_ASSERT_EQUAL_UINT16(20, total);
    TEST_ASSERT_EQUAL_UINT16(2, needed);  // 6 in the FF, then 7 + 7

    uint8_t ff8[8] = {0x10, 0x08, 0x43, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_INT(ESP_OK, parseFirstFrame(ff8, 8, total, needed));
    TEST_ASSERT_EQUAL_UINT16(1, needed);
}

static void test_first_frame_rejects_bad_lengths(void)
{
    uint16_t total = 0, needed = 0;
    uint8_t  ff7[8]    = {0x10, 0x07, 0x43, 0, 0, 0, 0, 0};  // would have been a single frame
    uint8_t  ff0[8]    = {0x10, 0x00, 0x43, 0, 0, 0, 0, 0};
    uint8_t  ff4095[8] = {0x1F, 0xFF, 0x43, 0, 0, 0, 0, 0};  // far above anything OBD returns
    uint8_t  ff513[8]  = {0x12, 0x01, 0x43, 0, 0, 0, 0, 0};
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseFirstFrame(ff7, 8, total, needed));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseFirstFrame(ff0, 8, total, needed));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseFirstFrame(ff4095, 8, total, needed));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseFirstFrame(ff513, 8, total, needed));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseFirstFrame(ff7, 1, total, needed));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, parseFirstFrame(nullptr, 8, total, needed));
}

static void test_consecutive_sequence_wraps_after_15(void)
{
    uint8_t cf1[8]  = {0x21, 0, 0, 0, 0, 0, 0, 0};
    uint8_t cf15[8] = {0x2F, 0, 0, 0, 0, 0, 0, 0};
    uint8_t cf16[8] = {0x20, 0, 0, 0, 0, 0, 0, 0};  // index 16 is sequence 0
    uint8_t cf17[8] = {0x21, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_TRUE(consecutiveSequenceOk(cf1, 8, 1));
    TEST_ASSERT_TRUE(consecutiveSequenceOk(cf15, 8, 15));
    TEST_ASSERT_TRUE(consecutiveSequenceOk(cf16, 8, 16));
    TEST_ASSERT_TRUE(consecutiveSequenceOk(cf17, 8, 17));
    TEST_ASSERT_FALSE(consecutiveSequenceOk(cf1, 8, 2));
    uint8_t notCf[8] = {0x11, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_FALSE(consecutiveSequenceOk(notCf, 8, 1));
    TEST_ASSERT_FALSE(consecutiveSequenceOk(cf1, 0, 1));
}

static void test_extract_vin(void)
{
    const char           src[] = "WVWZZZ1KZ6W000001";
    std::vector<uint8_t> p     = {0x49, 0x02, 0x01};
    p.insert(p.end(), src, src + 17);
    char vin[17];
    TEST_ASSERT_EQUAL_INT(ESP_OK, extractVin(p, vin));
    TEST_ASSERT_EQUAL_MEMORY(src, vin, 17);
}

static void test_extract_vin_rejects_short_payload(void)
{
    std::vector<uint8_t> p(10, 0x41);
    char                 vin[17];
    TEST_ASSERT_NOT_EQUAL(ESP_OK, extractVin(p, vin));
}

static void test_dtcs_single_frame(void)
{
    // 06 43 01 33 02 44 00 -> payload after the SID is 4 bytes: P0133, P0244 (PCI length 5)
    std::vector<uint8_t>  p = {0x43, 0x01, 0x33, 0x02, 0x44};
    std::vector<uint16_t> dtcs;
    TEST_ASSERT_EQUAL_INT(ESP_OK, extractDtcs(p, dtcs));
    TEST_ASSERT_EQUAL_UINT32(2, dtcs.size());
    TEST_ASSERT_EQUAL_HEX16(0x0133, dtcs[0]);
    TEST_ASSERT_EQUAL_HEX16(0x0244, dtcs[1]);
}

static void test_dtcs_multi_frame_payload(void)
{
    // 15 bytes: 43 + 7 DTCs (the reassembly itself is covered by the DiagCollector tests).
    std::vector<uint8_t>  p = {0x43, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x04, 0x04, 0x05, 0x05, 0x06, 0x06, 0x07, 0x07};
    std::vector<uint16_t> dtcs;
    TEST_ASSERT_EQUAL_INT(ESP_OK, extractDtcs(p, dtcs));
    TEST_ASSERT_EQUAL_UINT32(7, dtcs.size());
    TEST_ASSERT_EQUAL_HEX16(0x0303, dtcs[2]);
    TEST_ASSERT_EQUAL_HEX16(0x0707, dtcs[6]);
}

static void test_dtcs_odd_payload_and_empty_response(void)
{
    std::vector<uint8_t>  p = {0x43, 0x01, 0x33, 0x02};  // trailing odd byte dropped
    std::vector<uint16_t> dtcs;
    TEST_ASSERT_EQUAL_INT(ESP_OK, extractDtcs(p, dtcs));
    TEST_ASSERT_EQUAL_UINT32(1, dtcs.size());
    p = {0x43};
    TEST_ASSERT_EQUAL_INT(ESP_OK, extractDtcs(p, dtcs));
    TEST_ASSERT_EQUAL_UINT32(0, dtcs.size());
    p.clear();
    TEST_ASSERT_NOT_EQUAL(ESP_OK, extractDtcs(p, dtcs));
}


// ROB-1: exhaustive and seeded-random malformed input. Invariants: no crash, outputs stay in bounds.
static void test_exhaustive_first_bytes_and_lengths(void)
{
    for (size_t len = 0; len <= CAN_DATA_MAX + 2; len++)
    {
        for (int b0 = 0; b0 < 256; b0++)
        {
            for (int b1 : {0x00, 0x41, 0x62, 0xFF})
            {
                uint8_t d[CAN_DATA_MAX + 2];
                memset(d, 0xA5, sizeof(d));
                d[0] = static_cast<uint8_t>(b0);
                d[1] = static_cast<uint8_t>(b1);
                SingleFrameInfo i;
                if (parseSingleFrame(d, len, i) == ESP_OK)
                {
                    TEST_ASSERT_TRUE(len >= 2 && len <= CAN_DATA_MAX);
                    TEST_ASSERT_TRUE(static_cast<size_t>(i.payloadOffset) + i.payloadLength <= len);
                }
                uint16_t total = 0, needed = 0;
                if (parseFirstFrame(d, len, total, needed) == ESP_OK)
                {
                    TEST_ASSERT_TRUE(len >= 3 && len <= CAN_DATA_MAX);
                    TEST_ASSERT_TRUE(total >= 8 && total <= MAX_ISOTP_LENGTH);
                }
                IsoTp t;
                frameType(d, len, t);
                consecutiveSequenceOk(d, len, 0xFFFF);
            }
        }
    }
}

static void test_random_payloads_never_overrun(void)
{
    uint32_t rng = 12345;
    auto     next = [&rng]() { return rng = rng * 1664525u + 1013904223u; };
    for (int iter = 0; iter < 20000; iter++)
    {
        std::vector<uint8_t> payload((next() >> 8) % (MAX_ISOTP_LENGTH + 1));
        for (auto& byte : payload) byte = static_cast<uint8_t>(next() >> 16);
        std::vector<uint16_t> dtcs;
        extractDtcs(payload, dtcs);
        TEST_ASSERT_TRUE(dtcs.size() <= MAX_ISOTP_LENGTH / 2);
        char vin[17];
        extractVin(payload, vin);
    }
}

static void test_connect_plan_skips_discovery_only_on_vin_match(void)
{
    TEST_ASSERT_EQUAL_INT((int)ConnectPlan::RestoreKnown, (int)planOnConnect("1HGCM82633A004352", "1HGCM82633A004352"));
    TEST_ASSERT_EQUAL_INT((int)ConnectPlan::Discover, (int)planOnConnect("1HGCM82633A004352", "WVWZZZ1JZXW000001"));
    TEST_ASSERT_EQUAL_INT((int)ConnectPlan::Discover, (int)planOnConnect("1HGCM82633A004352", ""));    // nothing known yet
    TEST_ASSERT_EQUAL_INT((int)ConnectPlan::Discover, (int)planOnConnect("", "1HGCM82633A004352"));    // VIN request timed out
    TEST_ASSERT_EQUAL_INT((int)ConnectPlan::Discover, (int)planOnConnect(nullptr, "1HGCM82633A004352"));
    TEST_ASSERT_EQUAL_INT((int)ConnectPlan::Discover, (int)planOnConnect("", ""));
    TEST_ASSERT_EQUAL_INT((int)ConnectPlan::Discover, (int)planOnConnect(nullptr, nullptr));
}

static void test_restore_queue_after_health_clear(void)
{
    TEST_ASSERT_TRUE(shouldRestoreQueue(true, true, true, false));    // lost queue, ECU answered again
    TEST_ASSERT_FALSE(shouldRestoreQueue(true, true, true, true));    // queue intact
    TEST_ASSERT_FALSE(shouldRestoreQueue(false, true, true, false));  // user stopped auto poll
    TEST_ASSERT_FALSE(shouldRestoreQueue(true, false, true, false));  // bus down
    TEST_ASSERT_FALSE(shouldRestoreQueue(true, true, false, false));  // discovery not finished
}

static void test_discovery_backoff_doubles_and_caps(void)
{
    uint32_t b = nextDiscoveryBackoffMs(0);
    const uint32_t expect[] = {2000, 4000, 8000, 16000, 30000, 30000, 30000};
    for (uint32_t e : expect)
    {
        TEST_ASSERT_EQUAL_UINT32(e, b);
        b = nextDiscoveryBackoffMs(b);
    }
    TEST_ASSERT_EQUAL_UINT32(30000, nextDiscoveryBackoffMs(0xFFFFFFFFu));
}

static void test_vin_confirmed_by_api_decides_recovery(void)
{
    const char* a = "1HGCM82633A004352";
    const char* b = "WVWZZZ1JZXW000001";
    // OBD not initialised: same as a connect
    TEST_ASSERT_EQUAL_INT((int)VinAction::Restore, (int)planOnVinConfirmed(a, a, false));
    TEST_ASSERT_EQUAL_INT((int)VinAction::Discover, (int)planOnVinConfirmed(a, b, false));
    TEST_ASSERT_EQUAL_INT((int)VinAction::Discover, (int)planOnVinConfirmed(a, "", false));
    // initialised: same VIN changes nothing, other VIN is another car, no known VIN is adopted
    TEST_ASSERT_EQUAL_INT((int)VinAction::None, (int)planOnVinConfirmed(a, a, true));
    TEST_ASSERT_EQUAL_INT((int)VinAction::Discover, (int)planOnVinConfirmed(a, b, true));
    TEST_ASSERT_EQUAL_INT((int)VinAction::Adopt, (int)planOnVinConfirmed(a, "", true));
    TEST_ASSERT_EQUAL_INT((int)VinAction::Discover, (int)planOnVinConfirmed(a, "WVWZZZ1JZXW000001", true));
    TEST_ASSERT_EQUAL_INT((int)VinAction::Adopt, (int)planOnVinConfirmed(a, nullptr, true));
    // differing only at char 17
    TEST_ASSERT_EQUAL_INT((int)VinAction::Discover, (int)planOnVinConfirmed("1HGCM82633A004352", "1HGCM82633A004353", true));
    TEST_ASSERT_EQUAL_INT((int)VinAction::Discover, (int)planOnVinConfirmed("1HGCM82633A004352", "1HGCM82633A004353", false));
    // an invalid VIN is a failed read
    TEST_ASSERT_EQUAL_INT((int)VinAction::None, (int)planOnVinConfirmed("1hgcm82633a004352", a, false));
    TEST_ASSERT_EQUAL_INT((int)VinAction::None, (int)planOnVinConfirmed("1HGCM82633A00435", a, true));
    // a failed read changes nothing
    TEST_ASSERT_EQUAL_INT((int)VinAction::None, (int)planOnVinConfirmed("", a, false));
    TEST_ASSERT_EQUAL_INT((int)VinAction::None, (int)planOnVinConfirmed(nullptr, a, true));
}

static void test_vin_validation(void)
{
    TEST_ASSERT_TRUE(isValidVin("1HGBH41JXMN109186"));  // the simulator VIN
    TEST_ASSERT_FALSE(isValidVin(nullptr));
    TEST_ASSERT_FALSE(isValidVin(""));
    TEST_ASSERT_FALSE(isValidVin("1HGBH41JXMN10918"));    // 16
    TEST_ASSERT_FALSE(isValidVin("1HGBH41JXMN1091866"));  // 18
    TEST_ASSERT_FALSE(isValidVin("1hgbh41jxmn109186"));   // lowercase
    TEST_ASSERT_FALSE(isValidVin("1HGBH41JXMN10918-"));   // punctuation
    TEST_ASSERT_FALSE(isValidVin("1HGBH41J MN109186"));   // space
    const char embedded[18] = {'1', 'H', 'G', 'B', 'H', '4', '1', 'J', '\0', 'M', 'N', '1', '0', '9', '1', '8', '6', '\0'};
    TEST_ASSERT_FALSE(isValidVin(embedded));
}

static void test_simulator_vin_frames_decode_to_a_valid_vin(void)
{
    // Same bytes obd2_simulator.cpp sends: first frame 10 14 49 02 01 v0..v2, then 21 v3..v9, 22 v10..v16.
    const char vinSrc[] = "1HGBH41JXMN109186";
    RawFrame f[3] = {};
    const uint8_t first[8] = {0x10, 0x14, 0x49, 0x02, 0x01, (uint8_t)vinSrc[0], (uint8_t)vinSrc[1], (uint8_t)vinSrc[2]};
    memcpy(f[0].data, first, 8);
    f[1].data[0] = 0x21;
    for (int i = 0; i < 7; i++) f[1].data[1 + i] = (uint8_t)vinSrc[3 + i];
    f[2].data[0] = 0x22;
    for (int i = 0; i < 7; i++) f[2].data[1 + i] = (uint8_t)vinSrc[10 + i];
    for (auto& x : f) x.length = 8;
    DiagCollector c;
    c.begin(DiagKind::Vin, 0x09, 0);
    for (const RawFrame& x : f) c.onFrame(0x7E8, x.data, x.length, 0);
    char vin[17];
    TEST_ASSERT_TRUE(c.vin(vin));
    char z[18] = {};
    memcpy(z, vin, 17);
    TEST_ASSERT_TRUE(isValidVin(z));
    TEST_ASSERT_EQUAL_STRING(vinSrc, z);
}

extern "C" void run_obd2_frame_tests(void)
{
    RUN_TEST(test_single_frame_mode01);
    RUN_TEST(test_single_frame_mode22);
    RUN_TEST(test_single_frame_without_pid_modes);
    RUN_TEST(test_clear_dtc_ack_is_two_bytes);
    RUN_TEST(test_single_frame_rejects_malformed);
    RUN_TEST(test_internal_derived_frame_is_accepted);
    RUN_TEST(test_first_frame_lengths);
    RUN_TEST(test_first_frame_rejects_bad_lengths);
    RUN_TEST(test_consecutive_sequence_wraps_after_15);
    RUN_TEST(test_extract_vin);
    RUN_TEST(test_extract_vin_rejects_short_payload);
    RUN_TEST(test_dtcs_single_frame);
    RUN_TEST(test_dtcs_multi_frame_payload);
    RUN_TEST(test_dtcs_odd_payload_and_empty_response);
    RUN_TEST(test_connect_plan_skips_discovery_only_on_vin_match);
    RUN_TEST(test_vin_validation);
    RUN_TEST(test_simulator_vin_frames_decode_to_a_valid_vin);
    RUN_TEST(test_restore_queue_after_health_clear);
    RUN_TEST(test_vin_confirmed_by_api_decides_recovery);
    RUN_TEST(test_discovery_backoff_doubles_and_caps);
    RUN_TEST(test_exhaustive_first_bytes_and_lengths);
    RUN_TEST(test_random_payloads_never_overrun);
}
