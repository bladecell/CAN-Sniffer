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

static void vinFrames(RawFrame (&f)[3])
{
    // 49 02 01 + "WVWZZZ1KZ6W000001"
    f[0] = frame({0x10, 0x14, 0x49, 0x02, 0x01, 'W', 'V', 'W'});
    f[1] = frame({0x21, 'Z', 'Z', 'Z', '1', 'K', 'Z', '6'});
    f[2] = frame({0x22, 'W', '0', '0', '0', '0', '0', '1'});
}

static void test_reassemble_vin(void)
{
    RawFrame             f[3];
    std::vector<uint8_t> p;
    vinFrames(f);
    TEST_ASSERT_EQUAL_INT(ESP_OK, reassemble(f, 3, p));
    TEST_ASSERT_EQUAL_UINT32(20, p.size());
    char vin[17];
    TEST_ASSERT_EQUAL_INT(ESP_OK, extractVin(p, vin));
    TEST_ASSERT_EQUAL_MEMORY("WVWZZZ1KZ6W000001", vin, 17);
}

static void test_reassemble_rejects_broken_sequences(void)
{
    RawFrame             f[3];
    std::vector<uint8_t> p;
    vinFrames(f);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, reassemble(f, 2, p));  // a consecutive frame is missing
    TEST_ASSERT_TRUE(p.empty());
    f[1].data[0] = 0x22;                                 // wrong sequence number
    TEST_ASSERT_NOT_EQUAL(ESP_OK, reassemble(f, 3, p));
    vinFrames(f);
    f[2].data[0] = 0x32;                                 // flow control instead of consecutive
    TEST_ASSERT_NOT_EQUAL(ESP_OK, reassemble(f, 3, p));
    vinFrames(f);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, reassemble(f + 1, 2, p));  // starts without a first frame
    TEST_ASSERT_NOT_EQUAL(ESP_OK, reassemble(f, 0, p));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, reassemble(nullptr, 3, p));
    vinFrames(f);
    f[1].length = 200;                                   // DLC above 8
    TEST_ASSERT_NOT_EQUAL(ESP_OK, reassemble(f, 3, p));
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
    RawFrame             f = frame({0x05, 0x43, 0x01, 0x33, 0x02, 0x44, 0x00, 0x00});
    std::vector<uint8_t> p;
    std::vector<uint16_t> dtcs;
    TEST_ASSERT_EQUAL_INT(ESP_OK, reassemble(&f, 1, p));
    TEST_ASSERT_EQUAL_INT(ESP_OK, extractDtcs(p, dtcs));
    TEST_ASSERT_EQUAL_UINT32(2, dtcs.size());
    TEST_ASSERT_EQUAL_HEX16(0x0133, dtcs[0]);
    TEST_ASSERT_EQUAL_HEX16(0x0244, dtcs[1]);
}

static void test_dtcs_multi_frame_pair_split_across_frames(void)
{
    // 15 bytes: 43 + 7 DTCs. FF holds 43 + 2.5 DTCs, so the third DTC straddles the FF/CF boundary.
    RawFrame f[3];
    f[0] = frame({0x10, 0x0F, 0x43, 0x01, 0x01, 0x02, 0x02, 0x03});
    f[1] = frame({0x21, 0x03, 0x04, 0x04, 0x05, 0x05, 0x06, 0x06});
    f[2] = frame({0x22, 0x07, 0x07, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA});
    std::vector<uint8_t>  p;
    std::vector<uint16_t> dtcs;
    TEST_ASSERT_EQUAL_INT(ESP_OK, reassemble(f, 3, p));
    TEST_ASSERT_EQUAL_UINT32(15, p.size());
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


static void test_decode_dtc_response_checks_the_sid(void)
{
    RawFrame              f = frame({0x05, 0x43, 0x01, 0x33, 0x02, 0x44, 0x00, 0x00});
    std::vector<uint16_t> dtcs;
    TEST_ASSERT_EQUAL_INT(ESP_OK, decodeDtcResponse(&f, 1, 0x43, dtcs));
    TEST_ASSERT_EQUAL_UINT32(2, dtcs.size());
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_RESPONSE, decodeDtcResponse(&f, 1, 0x47, dtcs));  // pending SID expected
    TEST_ASSERT_TRUE(dtcs.empty());
    TEST_ASSERT_NOT_EQUAL(ESP_OK, decodeDtcResponse(&f, 0, 0x43, dtcs));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, decodeDtcResponse(nullptr, 1, 0x43, dtcs));
}

static void test_decode_vin_response_checks_the_header(void)
{
    RawFrame f[3];
    char     vin[17];
    vinFrames(f);
    TEST_ASSERT_EQUAL_INT(ESP_OK, decodeVinResponse(f, 3, vin));
    TEST_ASSERT_EQUAL_MEMORY("WVWZZZ1KZ6W000001", vin, 17);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_SIZE, decodeVinResponse(f, 2, vin));  // frame missing
    vinFrames(f);
    f[0].data[3] = 0x03;  // PID 0x03 instead of 0x02
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_RESPONSE, decodeVinResponse(f, 3, vin));
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

static void test_random_frame_sequences_never_overrun(void)
{
    uint32_t rng = 12345;
    auto     next = [&rng]() { return rng = rng * 1664525u + 1013904223u; };
    for (int iter = 0; iter < 20000; iter++)
    {
        RawFrame frames[80] = {};
        size_t   count      = (next() >> 8) % 81;
        for (size_t k = 0; k < count; k++)
        {
            for (auto& byte : frames[k].data) byte = static_cast<uint8_t>(next() >> 16);
            frames[k].length = (next() >> 8) % (CAN_DATA_MAX + 2);
            if (k == 0 && (next() & 1)) frames[k].data[0] = 0x10 | (frames[k].data[0] & 0x0F);
            if (k > 0 && (next() & 3)) frames[k].data[0] = 0x20 | (k & 0x0F);
        }
        std::vector<uint8_t> payload;
        if (reassemble(frames, count, payload) == ESP_OK)
        {
            TEST_ASSERT_TRUE(payload.size() <= MAX_ISOTP_LENGTH);
        }
        else
        {
            TEST_ASSERT_TRUE(payload.empty());
        }
        std::vector<uint16_t> dtcs;
        extractDtcs(payload, dtcs);
        char vin[17];
        extractVin(payload, vin);
    }
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
    RUN_TEST(test_reassemble_vin);
    RUN_TEST(test_reassemble_rejects_broken_sequences);
    RUN_TEST(test_extract_vin_rejects_short_payload);
    RUN_TEST(test_dtcs_single_frame);
    RUN_TEST(test_dtcs_multi_frame_pair_split_across_frames);
    RUN_TEST(test_dtcs_odd_payload_and_empty_response);
    RUN_TEST(test_decode_dtc_response_checks_the_sid);
    RUN_TEST(test_decode_vin_response_checks_the_header);
    RUN_TEST(test_exhaustive_first_bytes_and_lengths);
    RUN_TEST(test_random_frame_sequences_never_overrun);
}
