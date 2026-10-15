// Multi-ECU DTC / VIN collection (OBD-11): per-ECU ISO-TP sessions, merged result, quiet-gap completion.
#include <cstring>
#include <initializer_list>
#include <vector>

#include "obd2_frame.hpp"
#include "unity.h"

using namespace obd2frame;

namespace
{
const DiagTimings T = defaultDiagTimings();  // 150 ms N_Cr, 50 ms quiet gap, 100 ms first wait, 5000 ms P2*, 3 pending

RawFrame frame8(std::initializer_list<uint8_t> bytes)
{
    RawFrame f = {};
    memset(f.data, 0xAA, sizeof(f.data));  // padding
    size_t i = 0;
    for (uint8_t b : bytes)
    {
        f.data[i++] = b;
    }
    f.length = 8;
    return f;
}

// ISO-TP segmentation of a payload (SID first) into FF + CFs, or one SF.
std::vector<RawFrame> segment(const std::vector<uint8_t>& payload)
{
    std::vector<RawFrame> out;
    RawFrame              f = {};
    memset(f.data, 0xAA, sizeof(f.data));
    f.length = 8;
    if (payload.size() <= 7)
    {
        f.data[0] = static_cast<uint8_t>(payload.size());
        memcpy(f.data + 1, payload.data(), payload.size());
        out.push_back(f);
        return out;
    }
    f.data[0] = static_cast<uint8_t>(0x10 | (payload.size() >> 8));
    f.data[1] = static_cast<uint8_t>(payload.size() & 0xFF);
    memcpy(f.data + 2, payload.data(), 6);
    out.push_back(f);
    uint8_t seq = 1;
    for (size_t pos = 6; pos < payload.size(); pos += 7)
    {
        RawFrame c = {};
        memset(c.data, 0xAA, sizeof(c.data));
        c.length  = 8;
        c.data[0] = static_cast<uint8_t>(0x20 | (seq++ & 0x0F));
        const size_t n = payload.size() - pos < 7 ? payload.size() - pos : 7;
        memcpy(c.data + 1, payload.data() + pos, n);
        out.push_back(c);
    }
    return out;
}

std::vector<uint8_t> dtcPayload(uint8_t sid, std::initializer_list<uint16_t> dtcs)
{
    // Assumed CAN layout (user-supplied, secondary sources pending): SID, number of DTCs, then 2 bytes per DTC.
    std::vector<uint8_t> p = {sid, static_cast<uint8_t>(dtcs.size())};
    for (uint16_t d : dtcs)
    {
        p.push_back(static_cast<uint8_t>(d >> 8));
        p.push_back(static_cast<uint8_t>(d & 0xFF));
    }
    return p;
}

std::vector<uint8_t> vinPayload(const char* vin)
{
    std::vector<uint8_t> p = {0x49, 0x02, 0x01};
    p.insert(p.end(), vin, vin + 17);
    return p;
}

DiagFrameResult feed(DiagCollector& c, uint32_t id, const RawFrame& f, uint32_t t)
{
    return c.onFrame(id, f.data, f.length, t);
}

// Feeds a whole segmented message from one ECU, spacing the frames `stepMs` apart. Returns the time of the last frame.
uint32_t feedAll(DiagCollector& c, uint32_t id, const std::vector<RawFrame>& frames, uint32_t t, uint32_t stepMs = 1)
{
    for (const RawFrame& f : frames)
    {
        feed(c, id, f, t);
        t += stepMs;
    }
    return t - stepMs;
}

void startDtc(DiagCollector& c, uint8_t mode = 0x03, uint32_t t = 0)
{
    c.begin(DiagKind::Dtc, mode, t);
    c.markSent(t);
}

void startVin(DiagCollector& c, uint32_t t = 0)
{
    c.begin(DiagKind::Vin, 0x09, t);
    c.markSent(t);
}

bool hasDtc(const DiagCollector& c, uint16_t raw)
{
    for (size_t i = 0; i < c.dtcCount(); i++)
    {
        if (c.dtcs()[i] == raw)
        {
            return true;
        }
    }
    return false;
}
}  // namespace

static void test_diag_single_ecu_multiframe_dtc_completes_after_quiet_gap(void)
{
    DiagCollector c;
    startDtc(c);
    const auto frames = segment(dtcPayload(0x43, {0x0101, 0x0202, 0x0303, 0x0404, 0x0505, 0x0606, 0x0707}));
    DiagFrameResult r = feed(c, 0x7E8, frames[0], 10);
    TEST_ASSERT_TRUE(r.consumed);
    TEST_ASSERT_TRUE(r.sendFlowControl);
    TEST_ASSERT_EQUAL_HEX32(0x7E0, r.flowControlId);
    TEST_ASSERT_EQUAL_UINT32(1, c.activeSessions());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(60)));  // reassembly active
    feed(c, 0x7E8, frames[1], 20);
    r = feed(c, 0x7E8, frames[2], 30);
    TEST_ASSERT_FALSE(r.sendFlowControl);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(1, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(30 + T.quietGapMs - 1)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(30 + T.quietGapMs)));
    TEST_ASSERT_EQUAL_UINT32(7, c.dtcCount());
    TEST_ASSERT_TRUE(hasDtc(c, 0x0707));
}

static void test_diag_interleaved_two_ecu_multiframes_do_not_corrupt_each_other(void)
{
    DiagCollector c;
    startDtc(c);
    const auto a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104, 0x0105}));  // 11 bytes: FF + 1 CF
    const auto b = segment(dtcPayload(0x43, {0x0201, 0x0202, 0x0203, 0x0204, 0x0205, 0x0206, 0x0207}));  // 15: FF + 2 CF
    DiagFrameResult ra = feed(c, 0x7E8, a[0], 5);
    DiagFrameResult rb = feed(c, 0x7E9, b[0], 6);
    TEST_ASSERT_EQUAL_HEX32(0x7E0, ra.flowControlId);
    TEST_ASSERT_EQUAL_HEX32(0x7E1, rb.flowControlId);
    TEST_ASSERT_TRUE(ra.sendFlowControl && rb.sendFlowControl);
    TEST_ASSERT_EQUAL_UINT32(2, c.activeSessions());
    feed(c, 0x7E9, b[1], 7);
    feed(c, 0x7E8, a[1], 8);
    feed(c, 0x7E9, b[2], 9);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(2, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(9 + T.quietGapMs)));
    TEST_ASSERT_EQUAL_UINT32(12, c.dtcCount());
    TEST_ASSERT_TRUE(hasDtc(c, 0x0105));
    TEST_ASSERT_TRUE(hasDtc(c, 0x0207));
}

static void test_diag_sequence_number_wraps_15_to_0(void)
{
    DiagCollector       c;
    startDtc(c);
    std::vector<uint8_t> p = {0x43, 60};
    for (uint16_t i = 1; i <= 60; i++)  // 121 bytes: 17 CFs, sequence 1..15, 0, 1
    {
        p.push_back(0x01);
        p.push_back(static_cast<uint8_t>(i));
    }
    const auto frames = segment(p);
    TEST_ASSERT_EQUAL_UINT32(18, frames.size());  // 122 bytes: FF + 17 CFs
    TEST_ASSERT_EQUAL_HEX8(0x20, frames[16].data[0]);  // wrapped
    feedAll(c, 0x7EA, frames, 0);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(1, c.responders());
    TEST_ASSERT_EQUAL_UINT32(60, c.dtcCount());
}

static void test_diag_stale_session_times_out_and_late_frames_are_ignored(void)
{
    DiagCollector c;
    startDtc(c);
    const auto a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104}));
    feed(c, 0x7E8, a[0], 0);
    TEST_ASSERT_EQUAL_UINT32(1, c.activeSessions());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(T.nCrMs - 1)));
    TEST_ASSERT_EQUAL_UINT32(1, c.activeSessions());
    c.poll(T.nCrMs);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    DiagFrameResult r = feed(c, 0x7E8, a[1], T.nCrMs + 1);  // late CF must not resurrect it
    TEST_ASSERT_FALSE(r.consumed);
    TEST_ASSERT_EQUAL_UINT32(0, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Failed), static_cast<int>(c.poll(T.nCrMs + T.firstResponseWaitMs)));
}

static void test_diag_ncr_restarts_on_every_valid_consecutive_frame(void)
{
    DiagCollector c;
    startDtc(c);
    std::vector<uint8_t> p = {0x43, 17};
    for (uint16_t i = 1; i <= 17; i++)  // 36 bytes: FF + 5 CFs
    {
        p.push_back(0x02);
        p.push_back(static_cast<uint8_t>(i));
    }
    const auto frames = segment(p);
    uint32_t   t      = 0;
    for (const RawFrame& f : frames)
    {
        feed(c, 0x7E8, f, t);
        c.poll(t + T.nCrMs - 1);  // would drop the session if N_Cr had not restarted
        TEST_ASSERT_EQUAL_UINT32(&f == &frames.back() ? 0 : 1, c.activeSessions());
        t += T.nCrMs - 10;  // every gap is shorter than N_Cr, the whole message is much longer
    }
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(17, c.dtcCount());
}

static void test_diag_quiet_gap_never_ends_an_active_reassembly(void)
{
    DiagCollector c;
    startDtc(c);
    feed(c, 0x7E8, frame8({0x06, 0x43, 0x02, 0x01, 0x11, 0x01, 0x12}), 0);                  // ECU A done at t=0
    const auto b = segment(dtcPayload(0x43, {0x0201, 0x0202, 0x0203, 0x0204}));
    feed(c, 0x7E9, b[0], 10);                                                           // ECU B starts a multi-frame
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(10 + T.quietGapMs + 20)));
    feed(c, 0x7E9, b[1], 10 + T.quietGapMs + 25);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(10 + 2 * T.quietGapMs + 25)));
}

static void test_diag_session_cap_drops_extra_ecus(void)
{
    DiagCollector c(T, 2);
    startDtc(c);
    const auto a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104}));
    TEST_ASSERT_TRUE(feed(c, 0x7E8, a[0], 0).sendFlowControl);
    TEST_ASSERT_TRUE(feed(c, 0x7E9, a[0], 0).sendFlowControl);
    DiagFrameResult r = feed(c, 0x7EA, a[0], 0);
    TEST_ASSERT_FALSE(r.sendFlowControl);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, r.err);
    TEST_ASSERT_EQUAL_UINT32(2, c.activeSessions());
    feed(c, 0x7E8, a[1], 1);  // the sessions that fit still finish
    feed(c, 0x7E9, a[1], 1);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(2, c.responders());
    // The dropped ECU may retry once a slot is free.
    TEST_ASSERT_TRUE(feed(c, 0x7EA, a[0], 2).sendFlowControl);
}

static void test_diag_dtc_union_is_deduplicated_and_empty_replies_count(void)
{
    DiagCollector c;
    startDtc(c, 0x07);
    feed(c, 0x7E8, frame8({0x06, 0x47, 0x02, 0x01, 0x33, 0x02, 0x44}), 1);
    feed(c, 0x7E9, frame8({0x06, 0x47, 0x02, 0x02, 0x44, 0x03, 0x55}), 2);  // 0x0244 duplicated across ECUs
    feed(c, 0x7EA, frame8({0x02, 0x47, 0x00}), 3);                     // count 0: still a responder
    TEST_ASSERT_EQUAL_UINT32(3, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(3 + T.quietGapMs)));
    TEST_ASSERT_EQUAL_UINT32(3, c.dtcCount());
    TEST_ASSERT_TRUE(hasDtc(c, 0x0133));
    TEST_ASSERT_TRUE(hasDtc(c, 0x0244));
    TEST_ASSERT_TRUE(hasDtc(c, 0x0355));
}

static void test_diag_only_empty_replies_complete_with_no_dtcs(void)
{
    DiagCollector c;
    startDtc(c, 0x0A);
    feed(c, 0x7E8, frame8({0x02, 0x4A, 0x00}), 1);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(1 + T.quietGapMs)));
    TEST_ASSERT_EQUAL_UINT32(1, c.responders());
    TEST_ASSERT_EQUAL_UINT32(0, c.dtcCount());
}

static void test_diag_wrong_sid_is_not_consumed_and_does_not_count(void)
{
    DiagCollector c;
    startDtc(c, 0x03);
    DiagFrameResult r = feed(c, 0x7E8, frame8({0x04, 0x41, 0x0C, 0x1A, 0xF8}), 1);  // mode 01 poll answer
    TEST_ASSERT_FALSE(r.consumed);
    r = feed(c, 0x7E8, frame8({0x03, 0x47, 0x01, 0x33}), 1);  // pending-DTC answer to a confirmed-DTC request
    TEST_ASSERT_FALSE(r.consumed);
    r = feed(c, 0x7E8, frame8({0x10, 0x14, 0x49, 0x02, 0x01, 'W', 'V', 'W'}), 1);  // VIN first frame
    TEST_ASSERT_FALSE(r.consumed);
    TEST_ASSERT_FALSE(r.sendFlowControl);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(0, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Failed), static_cast<int>(c.poll(T.firstResponseWaitMs)));
}

static void test_diag_vin_first_valid_wins_and_differing_vin_is_counted(void)
{
    DiagCollector c;
    startVin(c);
    feedAll(c, 0x7E8, segment(vinPayload("WVWZZZ1JZXW000001")), 0);
    feedAll(c, 0x7E9, segment(vinPayload("WAUZZZ8K9BA000002")), 5);
    TEST_ASSERT_EQUAL_UINT32(2, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(5 + T.quietGapMs + 5)));
    char vin[17];
    TEST_ASSERT_TRUE(c.vin(vin));
    TEST_ASSERT_EQUAL_MEMORY("WVWZZZ1JZXW000001", vin, 17);
    TEST_ASSERT_EQUAL_UINT32(1, c.conflictingVins());
}

static void test_diag_vin_same_vin_from_two_ecus_is_not_a_conflict(void)
{
    DiagCollector c;
    startVin(c);
    feedAll(c, 0x7E8, segment(vinPayload("WVWZZZ1JZXW000001")), 0);
    feedAll(c, 0x7EA, segment(vinPayload("WVWZZZ1JZXW000001")), 3);
    TEST_ASSERT_EQUAL_UINT32(0, c.conflictingVins());
}

static void test_diag_vin_skips_an_invalid_reply(void)
{
    DiagCollector c;
    startVin(c);
    feedAll(c, 0x7E8, segment(vinPayload("WVWZZZ1JZ?W000001")), 0);  // '?' is not alphanumeric
    feedAll(c, 0x7E9, segment(vinPayload("WAUZZZ8K9BA000002")), 3);
    char vin[17];
    TEST_ASSERT_TRUE(c.vin(vin));
    TEST_ASSERT_EQUAL_MEMORY("WAUZZZ8K9BA000002", vin, 17);
    TEST_ASSERT_EQUAL_UINT32(0, c.conflictingVins());
}

static void test_diag_vin_without_valid_reply_has_no_vin(void)
{
    DiagCollector c;
    startVin(c);
    feedAll(c, 0x7E8, segment(vinPayload("wvwzzz1jzxw000001")), 0);  // lower case is rejected like isValidVin()
    char vin[17];
    TEST_ASSERT_FALSE(c.vin(vin));
}

static void test_diag_short_payloads_count_as_responders_without_data(void)
{
    DiagCollector c;
    startDtc(c);
    feed(c, 0x7E8, frame8({0x02, 0x43, 0x01}), 1);  // count 1 but no DTC bytes: nothing to read, still a responder
    feed(c, 0x7E9, frame8({0x01, 0x43}), 2);        // no count byte at all
    feed(c, 0x7EA, frame8({0x05, 0x43, 0x01, 0x33, 0x02}), 3);  // count 1, pair 0x3302
    TEST_ASSERT_EQUAL_UINT32(3, c.responders());
    TEST_ASSERT_EQUAL_UINT32(1, c.dtcCount());
    TEST_ASSERT_TRUE(hasDtc(c, 0x3302));
    // A zero-length or too-long PCI is not a frame at all.
    TEST_ASSERT_FALSE(feed(c, 0x7EB, frame8({0x00, 0x43}), 4).consumed);
    TEST_ASSERT_FALSE(feed(c, 0x7EB, frame8({0x08, 0x43}), 4).consumed);
    RawFrame shortDlc = frame8({0x06, 0x43, 0x02, 0x01, 0x33, 0x02, 0x44});
    shortDlc.length   = 3;  // PCI claims 6 bytes but the frame carries 2
    TEST_ASSERT_FALSE(feed(c, 0x7EB, shortDlc, 4).consumed);
    TEST_ASSERT_EQUAL_UINT32(3, c.responders());
}

static void test_diag_dtc_count_byte_layout(void)
{
    // 43 NN then NN pairs. Assumed CAN layout, see dtcPayload().
    {
        DiagCollector c;
        startDtc(c);
        feed(c, 0x7E8, frame8({0x02, 0x43, 0x00}), 1);
        TEST_ASSERT_EQUAL_UINT32(1, c.responders());
        TEST_ASSERT_EQUAL_UINT32(0, c.dtcCount());
    }
    {
        DiagCollector c;  // one DTC, the rest of the 8-byte frame is padding (non-zero too)
        startDtc(c);
        feed(c, 0x7E8, frame8({0x07, 0x43, 0x01, 0x01, 0x33, 0xAA, 0xAA, 0xAA}), 1);
        TEST_ASSERT_EQUAL_UINT32(1, c.dtcCount());
        TEST_ASSERT_TRUE(hasDtc(c, 0x0133));
    }
    {
        DiagCollector c;  // padding with zeros after the count
        startDtc(c);
        feed(c, 0x7E8, frame8({0x07, 0x43, 0x01, 0x01, 0x33, 0x00, 0x00, 0x00}), 1);
        TEST_ASSERT_EQUAL_UINT32(1, c.dtcCount());
    }
    {
        DiagCollector c;  // two DTCs
        startDtc(c);
        feed(c, 0x7E8, frame8({0x06, 0x43, 0x02, 0x01, 0x33, 0x02, 0x44}), 1);
        TEST_ASSERT_EQUAL_UINT32(2, c.dtcCount());
        TEST_ASSERT_TRUE(hasDtc(c, 0x0133) && hasDtc(c, 0x0244));
    }
    {
        DiagCollector c;  // count larger than the bytes present: only whole pairs that are there
        startDtc(c);
        feed(c, 0x7E8, frame8({0x05, 0x43, 0xFF, 0x01, 0x33, 0x02}), 1);
        TEST_ASSERT_EQUAL_UINT32(1, c.dtcCount());
        TEST_ASSERT_TRUE(hasDtc(c, 0x0133));
    }
    {
        DiagCollector c;  // count smaller than the pairs present: the count wins
        startDtc(c);
        feed(c, 0x7E8, frame8({0x07, 0x43, 0x01, 0x01, 0x33, 0x02, 0x44, 0x03}), 1);
        TEST_ASSERT_EQUAL_UINT32(1, c.dtcCount());
        TEST_ASSERT_TRUE(hasDtc(c, 0x0133));
    }
    {
        DiagCollector c;  // multi-frame, count 255 but only 3 pairs sent
        startDtc(c);
        std::vector<uint8_t> p = {0x43, 0xFF, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03};
        feedAll(c, 0x7E8, segment(p), 0);
        TEST_ASSERT_EQUAL_UINT32(3, c.dtcCount());
    }
    {
        DiagCollector c;  // multi-frame with the count honoured: 4 DTCs listed, count says 3
        startDtc(c);
        std::vector<uint8_t> p = {0x43, 0x03, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x04, 0x04};
        feedAll(c, 0x7E8, segment(p), 0);
        TEST_ASSERT_EQUAL_UINT32(3, c.dtcCount());
        TEST_ASSERT_FALSE(hasDtc(c, 0x0404));
    }
    {
        DiagCollector c;  // a count byte of 0x00 DTC value 0x0000 pairs are never stored
        startDtc(c);
        feed(c, 0x7E8, frame8({0x06, 0x43, 0x02, 0x00, 0x00, 0x01, 0x33}), 1);
        TEST_ASSERT_EQUAL_UINT32(1, c.dtcCount());
    }
    {
        DiagCollector c;  // the first payload byte after the SID is a count, never a DTC half
        startDtc(c, 0x0A);
        feed(c, 0x7E8, frame8({0x04, 0x4A, 0x01, 0x03, 0x01}), 1);
        TEST_ASSERT_TRUE(hasDtc(c, 0x0301));
        TEST_ASSERT_FALSE(hasDtc(c, 0x0103));
    }
}

static void test_diag_short_vin_payloads_give_no_vin(void)
{
    DiagCollector c;
    startVin(c);
    feed(c, 0x7E8, frame8({0x03, 0x49, 0x02, 0x01}), 1);  // header only
    // 12-byte payload: first frame plus one consecutive frame, far short of 17 characters.
    feed(c, 0x7E9, frame8({0x10, 0x0C, 0x49, 0x02, 0x01, 'A', 'B', 'C'}), 2);
    feed(c, 0x7E9, frame8({0x21, 'D', 'E', 'F', 'G', 'H', 'I', 0xAA}), 3);
    TEST_ASSERT_EQUAL_UINT32(2, c.responders());
    char vin[17];
    TEST_ASSERT_FALSE(c.vin(vin));
}

static void test_diag_vin_result_is_an_error_without_a_valid_vin(void)
{
    const char* const bad[] = {
        "WVWZZZ1JZ?W000001",   // punctuation
        "wvwzzz1jzxw000001",   // lower case
        "WVWZZZ1JZXW00000 ",   // space
        "WVWZZZ1JZXW0000\x01\x7F",  // control and DEL bytes
    };
    for (const char* v : bad)
    {
        DiagCollector c;
        startVin(c);
        feedAll(c, 0x7E8, segment(vinPayload(v)), 0);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(60)));
        TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_RESPONSE, c.result());
    }
    {
        DiagCollector c;  // non-ASCII bytes
        startVin(c);
        std::vector<uint8_t> p = {0x49, 0x02, 0x01};
        for (int i = 0; i < 17; i++)
        {
            p.push_back(static_cast<uint8_t>(0x80 + i));
        }
        feedAll(c, 0x7E8, segment(p), 0);
        TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_RESPONSE, c.result());
    }
    {
        DiagCollector c;  // short payload (header only, then 12 bytes)
        startVin(c);
        feed(c, 0x7E8, frame8({0x03, 0x49, 0x02, 0x01}), 1);
        TEST_ASSERT_EQUAL_UINT32(1, c.responders());
        TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_RESPONSE, c.result());
    }
    {
        DiagCollector c;  // two ECUs, both garbage
        startVin(c);
        feedAll(c, 0x7E8, segment(vinPayload("WVWZZZ1JZ?W000001")), 0);
        feedAll(c, 0x7E9, segment(vinPayload("???????????????????")), 3);
        TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_RESPONSE, c.result());
    }
}

static void test_diag_vin_result_is_ok_with_one_valid_vin_or_for_dtcs(void)
{
    {
        DiagCollector c;  // one bad ECU does not spoil a good one
        startVin(c);
        feedAll(c, 0x7E8, segment(vinPayload("WVWZZZ1JZ?W000001")), 0);
        feedAll(c, 0x7E9, segment(vinPayload("WAUZZZ8K9BA000002")), 3);
        TEST_ASSERT_EQUAL_INT(ESP_OK, c.result());
    }
    {
        DiagCollector c;
        startVin(c);
        feedAll(c, 0x7E8, segment(vinPayload("1HGBH41JXMN109186")), 0);
        TEST_ASSERT_EQUAL_INT(ESP_OK, c.result());
    }
    {
        DiagCollector c;  // a positive DTC reply with zero DTCs is a success
        startDtc(c);
        feed(c, 0x7E8, frame8({0x02, 0x43, 0x00}), 1);
        TEST_ASSERT_EQUAL_INT(ESP_OK, c.result());
    }
}

static void test_diag_hostile_first_frame_lengths(void)
{
    DiagCollector c;
    startDtc(c);
    DiagFrameResult r = feed(c, 0x7E8, frame8({0x1F, 0xFF, 0x43, 0x01, 0x01, 0x01, 0x01, 0x01}), 0);  // 4095 > 512
    TEST_ASSERT_FALSE(r.sendFlowControl);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, r.err);
    r = feed(c, 0x7E8, frame8({0x12, 0x01, 0x43, 0x01, 0x01, 0x01, 0x01, 0x01}), 0);  // 513
    TEST_ASSERT_FALSE(r.sendFlowControl);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, r.err);
    r = feed(c, 0x7E8, frame8({0x10, 0x07, 0x43, 0x01, 0x01, 0x01, 0x01, 0x01}), 0);  // 7 fits a single frame
    TEST_ASSERT_FALSE(r.sendFlowControl);
    r = feed(c, 0x7E8, frame8({0x10, 0x00, 0x43, 0x01, 0x01, 0x01, 0x01, 0x01}), 0);
    TEST_ASSERT_FALSE(r.sendFlowControl);
    RawFrame shortFf = frame8({0x10, 0x20, 0x43});
    shortFf.length   = 3;
    r                = feed(c, 0x7E8, shortFf, 0);
    TEST_ASSERT_FALSE(r.sendFlowControl);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    // The largest legal length still works.
    r = feed(c, 0x7E8, frame8({0x12, 0x00, 0x43, 0x01, 0x01, 0x01, 0x01, 0x01}), 0);
    TEST_ASSERT_TRUE(r.sendFlowControl);
}

static void test_diag_bad_sequence_drops_only_that_ecus_session(void)
{
    DiagCollector c;
    startDtc(c);
    const auto a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104, 0x0105, 0x0106, 0x0107}));  // FF + 2 CF
    const auto b = segment(dtcPayload(0x43, {0x0201, 0x0202, 0x0203, 0x0204}));
    feed(c, 0x7E8, a[0], 0);
    feed(c, 0x7E9, b[0], 0);
    DiagFrameResult r = feed(c, 0x7E8, a[2], 1);  // skipped CF 1
    TEST_ASSERT_NOT_EQUAL(ESP_OK, r.err);
    TEST_ASSERT_EQUAL_UINT32(1, c.activeSessions());
    feed(c, 0x7E8, a[1], 2);  // the aborted session is gone: this CF is not consumed
    feed(c, 0x7E9, b[1], 3);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(1, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(3 + T.quietGapMs)));
    TEST_ASSERT_EQUAL_UINT32(4, c.dtcCount());
    TEST_ASSERT_TRUE(hasDtc(c, 0x0204));
    TEST_ASSERT_FALSE(hasDtc(c, 0x0101));
}

static void test_diag_duplicate_and_wrong_sequence_variants(void)
{
    DiagCollector c;
    startDtc(c);
    const auto a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104, 0x0105, 0x0106, 0x0107}));
    feed(c, 0x7E8, a[0], 0);
    feed(c, 0x7E8, a[1], 1);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, feed(c, 0x7E8, a[1], 2).err);  // repeated CF
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    feed(c, 0x7E8, a[0], 3);
    RawFrame bad = a[1];
    bad.data[0]  = 0x2F;  // wrong number
    TEST_ASSERT_NOT_EQUAL(ESP_OK, feed(c, 0x7E8, bad, 4).err);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    feed(c, 0x7E8, a[0], 5);
    RawFrame shortCf = a[1];
    shortCf.length   = 4;  // a CF that is not the last must carry 7 data bytes
    TEST_ASSERT_NOT_EQUAL(ESP_OK, feed(c, 0x7E8, shortCf, 6).err);
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());
    TEST_ASSERT_EQUAL_UINT32(0, c.responders());
}

static void test_diag_foreign_ids_and_degenerate_input_are_ignored(void)
{
    DiagCollector c;
    startDtc(c);
    const RawFrame sf = frame8({0x06, 0x43, 0x02, 0x01, 0x33, 0x02, 0x44});
    TEST_ASSERT_FALSE(feed(c, 0x7DF, sf, 0).consumed);
    TEST_ASSERT_FALSE(feed(c, 0x7E7, sf, 0).consumed);
    TEST_ASSERT_FALSE(feed(c, 0x7F0, sf, 0).consumed);
    TEST_ASSERT_FALSE(feed(c, 0x18DAF110, sf, 0).consumed);  // 29-bit is out of scope
    TEST_ASSERT_FALSE(c.onFrame(0x7E8, nullptr, 8, 0).consumed);
    TEST_ASSERT_FALSE(c.onFrame(0x7E8, sf.data, 0, 0).consumed);
    TEST_ASSERT_FALSE(c.onFrame(0x7E8, sf.data, 9, 0).consumed);
    TEST_ASSERT_EQUAL_UINT32(0, c.responders());
    // Frames while no collection runs are not consumed and send no flow control.
    DiagCollector idle;
    const auto    a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104}));
    DiagFrameResult r = feed(idle, 0x7E8, a[0], 0);
    TEST_ASSERT_FALSE(r.consumed);
    TEST_ASSERT_FALSE(r.sendFlowControl);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Idle), static_cast<int>(idle.poll(1000)));
}

static void test_diag_flow_control_targets_the_physical_id_never_functional(void)
{
    for (uint32_t id = DIAG_RESPONSE_ID_FIRST; id <= DIAG_RESPONSE_ID_LAST; id++)
    {
        DiagCollector c;
        startDtc(c);
        const auto a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104}));
        DiagFrameResult r = feed(c, id, a[0], 0);
        TEST_ASSERT_TRUE(r.sendFlowControl);
        TEST_ASSERT_EQUAL_HEX32(id - 8, r.flowControlId);
        TEST_ASSERT_NOT_EQUAL(0x7DF, r.flowControlId);
    }
}

static void test_diag_first_response_wait_runs_from_the_request(void)
{
    DiagCollector c;
    c.begin(DiagKind::Dtc, 0x03, 0);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(100000)));  // not sent yet
    c.markSent(1000);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(1000 + T.firstResponseWaitMs - 1)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Failed), static_cast<int>(c.poll(1000 + T.firstResponseWaitMs)));
}

static void test_diag_pending_response_extends_the_wait_but_is_bounded(void)
{
    DiagCollector c;
    startDtc(c);
    feed(c, 0x7E8, frame8({0x02, 0x43, 0x00}), 0);  // ECU A answered, nothing to report
    const RawFrame pending = frame8({0x03, 0x7F, 0x03, 0x78});
    TEST_ASSERT_FALSE(c.anyPending());
    feed(c, 0x7E9, pending, 10);
    TEST_ASSERT_TRUE(c.anyPending());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(10 + T.quietGapMs + 100)));
    feed(c, 0x7E9, pending, 4000);  // P2* restarts on each valid 0x78
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(4000 + T.pendingMs - 1)));
    feed(c, 0x7E9, pending, 8000);  // third consecutive 0x78
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(8000 + T.pendingMs - 1)));
    feed(c, 0x7E9, pending, 12000);  // fourth: over the limit, the ECU is given up and not extended further
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(12000 + T.quietGapMs)));
    TEST_ASSERT_EQUAL_UINT32(1, c.responders());
}

static void test_diag_pending_ecu_finally_answers(void)
{
    DiagCollector c;
    startDtc(c);
    feed(c, 0x7E9, frame8({0x03, 0x7F, 0x03, 0x78}), 5);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(2000)));  // beyond the first wait, still pending
    feed(c, 0x7E9, frame8({0x04, 0x43, 0x01, 0x09, 0x99}), 2500);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(2500 + T.quietGapMs)));
    TEST_ASSERT_TRUE(hasDtc(c, 0x0999));
}

static void test_diag_pending_counter_resets_after_a_real_answer(void)
{
    DiagCollector c;
    startDtc(c);
    const RawFrame pending = frame8({0x03, 0x7F, 0x03, 0x78});
    for (uint32_t i = 0; i < T.maxPending; i++)
    {
        feed(c, 0x7E9, pending, i);
    }
    feed(c, 0x7E9, frame8({0x02, 0x43, 0x00}), 10);  // answered: a later pending cycle starts counting again
    feed(c, 0x7E9, pending, 20);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(20 + T.quietGapMs + 10)));
}

static void test_diag_other_negative_response_is_not_a_responder(void)
{
    DiagCollector c;
    startDtc(c);
    TEST_ASSERT_TRUE(feed(c, 0x7E8, frame8({0x03, 0x7F, 0x03, 0x12}), 1).consumed);  // sub-function not supported
    TEST_ASSERT_EQUAL_UINT32(0, c.responders());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Failed), static_cast<int>(c.poll(T.firstResponseWaitMs)));
    // A negative response for another service is none of our business.
    TEST_ASSERT_FALSE(feed(c, 0x7E8, frame8({0x03, 0x7F, 0x09, 0x78}), 1).consumed);
}

static void test_diag_clock_wrap_and_restart(void)
{
    DiagCollector c;
    const uint32_t t0 = 0xFFFFFFF0u;
    startDtc(c, 0x03, t0);
    feed(c, 0x7E8, frame8({0x06, 0x43, 0x02, 0x01, 0x33, 0x02, 0x44}), t0 + 5);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Waiting), static_cast<int>(c.poll(t0 + 5 + T.quietGapMs - 1)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(t0 + 5 + T.quietGapMs)));
    // begin() clears the previous result.
    c.begin(DiagKind::Dtc, 0x03, 1000);
    TEST_ASSERT_EQUAL_UINT32(0, c.dtcCount());
    TEST_ASSERT_EQUAL_UINT32(0, c.responders());
    c.cancel();
    TEST_ASSERT_FALSE(c.active());
}

static void test_diag_random_frames_never_break_the_collector(void)
{
    uint32_t seed = 12345;
    auto     rnd  = [&seed]() { seed = seed * 1664525u + 1013904223u; return seed >> 8; };
    for (int round = 0; round < 200; round++)
    {
        DiagCollector c;
        if (round % 2)
        {
            startVin(c);
        }
        else
        {
            startDtc(c, 0x03);
        }
        uint32_t t = 0;
        for (int i = 0; i < 200; i++)
        {
            RawFrame f = {};
            for (auto& b : f.data)
            {
                b = static_cast<uint8_t>(rnd());
            }
            if (rnd() % 3 == 0)
            {
                f.data[0] = static_cast<uint8_t>((rnd() % 4 == 0 ? 0x10 : 0x20) | (rnd() % 16));
                if (f.data[0] >= 0x10 && f.data[0] < 0x20)
                {
                    f.data[2] = round % 2 ? 0x49 : 0x43;
                    f.data[3] = 0x02;
                }
            }
            f.length = 1 + rnd() % 8;
            t += rnd() % 40;
            c.onFrame(0x7E7 + rnd() % 10, f.data, f.length, t);
            c.poll(t);
            TEST_ASSERT_TRUE(c.activeSessions() <= ISOTP_MAX_SESSIONS);
            TEST_ASSERT_TRUE(c.dtcCount() <= MAX_MERGED_DTCS);
        }
    }
}

namespace
{
DiagWaitState waitState(uint32_t start, uint32_t now, bool reassembling, bool pending)
{
    return {now, start, OBD2_DIAG_WAIT_MS, OBD2_DIAG_REASSEMBLY_CAP_MS, OBD2_DIAG_PENDING_CAP_MS, reassembling, pending};
}
}  // namespace

static void test_diag_wait_slow_multiframe_is_not_abandoned_and_completes(void)
{
    // 14 consecutive frames 50 ms apart: about 700 ms, every gap well inside N_Cr, longer than the 500 ms base wait.
    DiagCollector c;
    startDtc(c);
    std::vector<uint8_t> p = {0x43, 50};
    for (uint16_t i = 1; i <= 50; i++)  // 102 bytes: FF + 14 CFs
    {
        p.push_back(0x01);
        p.push_back(static_cast<uint8_t>(i));
    }
    const auto frames = segment(p);
    TEST_ASSERT_EQUAL_UINT32(15, frames.size());
    uint32_t t = 10;
    for (size_t i = 0; i < frames.size(); i++, t += 50)
    {
        feed(c, 0x7E8, frames[i], t);
        c.poll(t);
        TEST_ASSERT_FALSE(diagWaitExpired(waitState(0, t, c.progressing(), c.anyPending())));
        // the requester also wakes between frames
        c.poll(t + 25);
        TEST_ASSERT_FALSE(diagWaitExpired(waitState(0, t + 25, c.progressing(), c.anyPending())));
    }
    t -= 50;
    TEST_ASSERT_TRUE(t > OBD2_DIAG_WAIT_MS);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DiagStatus::Complete), static_cast<int>(c.poll(t + T.quietGapMs)));
    TEST_ASSERT_EQUAL_UINT32(50, c.dtcCount());
}

static void test_diag_wait_stalled_session_does_not_keep_the_wait_alive(void)
{
    DiagCollector c;
    startDtc(c);
    const auto a = segment(dtcPayload(0x43, {0x0101, 0x0102, 0x0103, 0x0104}));
    feed(c, 0x7E8, a[0], 10);  // first frame, then the ECU goes silent
    uint32_t expiredAt = 0;
    for (uint32_t t = 10; t < 20000 && expiredAt == 0; t += 50)
    {
        c.poll(t);
        if (diagWaitExpired(waitState(0, t, c.progressing(), c.anyPending())))
        {
            expiredAt = t;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0, c.activeSessions());  // N_Cr dropped it
    TEST_ASSERT_TRUE(expiredAt >= OBD2_DIAG_WAIT_MS);
    TEST_ASSERT_TRUE(expiredAt < OBD2_DIAG_WAIT_MS + 100);
}

static void test_diag_wait_reassembly_cap_is_a_hard_limit(void)
{
    TEST_ASSERT_EQUAL_UINT32(11600, OBD2_DIAG_REASSEMBLY_CAP_MS);  // 500 + 74 x 150
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(1000, 1000 + OBD2_DIAG_REASSEMBLY_CAP_MS - 1, true, false)));
    TEST_ASSERT_TRUE(diagWaitExpired(waitState(1000, 1000 + OBD2_DIAG_REASSEMBLY_CAP_MS, true, false)));
}

static void test_diag_wait_pending_behaviour_is_unchanged(void)
{
    TEST_ASSERT_TRUE(diagWaitExpired(waitState(0, OBD2_DIAG_WAIT_MS, false, false)));
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(0, OBD2_DIAG_WAIT_MS - 1, false, false)));
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(0, OBD2_DIAG_WAIT_MS + 4000, false, true)));
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(0, OBD2_DIAG_PENDING_CAP_MS - 1, false, true)));
    TEST_ASSERT_TRUE(diagWaitExpired(waitState(0, OBD2_DIAG_PENDING_CAP_MS, false, true)));
    // a pending ECU does not lift the reassembly cap, and the larger cap wins when both hold
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(0, OBD2_DIAG_REASSEMBLY_CAP_MS, true, true)));
    TEST_ASSERT_TRUE(diagWaitExpired(waitState(0, OBD2_DIAG_PENDING_CAP_MS, true, true)));
}

static void test_diag_wait_survives_tick_wraparound(void)
{
    const uint32_t start = 0xFFFFFF00u;
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(start, start + OBD2_DIAG_WAIT_MS - 1, false, false)));
    TEST_ASSERT_TRUE(diagWaitExpired(waitState(start, start + OBD2_DIAG_WAIT_MS, false, false)));
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(start, start + 700, true, false)));  // wrapped past 0
    TEST_ASSERT_TRUE(diagWaitExpired(waitState(start, start + OBD2_DIAG_REASSEMBLY_CAP_MS, true, false)));
    TEST_ASSERT_FALSE(diagWaitExpired(waitState(start, start, false, false)));
}

extern "C" void run_obd2_diag_collector_tests(void)
{
    RUN_TEST(test_diag_single_ecu_multiframe_dtc_completes_after_quiet_gap);
    RUN_TEST(test_diag_interleaved_two_ecu_multiframes_do_not_corrupt_each_other);
    RUN_TEST(test_diag_sequence_number_wraps_15_to_0);
    RUN_TEST(test_diag_stale_session_times_out_and_late_frames_are_ignored);
    RUN_TEST(test_diag_ncr_restarts_on_every_valid_consecutive_frame);
    RUN_TEST(test_diag_quiet_gap_never_ends_an_active_reassembly);
    RUN_TEST(test_diag_session_cap_drops_extra_ecus);
    RUN_TEST(test_diag_dtc_union_is_deduplicated_and_empty_replies_count);
    RUN_TEST(test_diag_only_empty_replies_complete_with_no_dtcs);
    RUN_TEST(test_diag_wrong_sid_is_not_consumed_and_does_not_count);
    RUN_TEST(test_diag_vin_first_valid_wins_and_differing_vin_is_counted);
    RUN_TEST(test_diag_vin_same_vin_from_two_ecus_is_not_a_conflict);
    RUN_TEST(test_diag_vin_skips_an_invalid_reply);
    RUN_TEST(test_diag_vin_without_valid_reply_has_no_vin);
    RUN_TEST(test_diag_short_payloads_count_as_responders_without_data);
    RUN_TEST(test_diag_dtc_count_byte_layout);
    RUN_TEST(test_diag_short_vin_payloads_give_no_vin);
    RUN_TEST(test_diag_vin_result_is_an_error_without_a_valid_vin);
    RUN_TEST(test_diag_vin_result_is_ok_with_one_valid_vin_or_for_dtcs);
    RUN_TEST(test_diag_hostile_first_frame_lengths);
    RUN_TEST(test_diag_bad_sequence_drops_only_that_ecus_session);
    RUN_TEST(test_diag_duplicate_and_wrong_sequence_variants);
    RUN_TEST(test_diag_foreign_ids_and_degenerate_input_are_ignored);
    RUN_TEST(test_diag_flow_control_targets_the_physical_id_never_functional);
    RUN_TEST(test_diag_first_response_wait_runs_from_the_request);
    RUN_TEST(test_diag_pending_response_extends_the_wait_but_is_bounded);
    RUN_TEST(test_diag_pending_ecu_finally_answers);
    RUN_TEST(test_diag_pending_counter_resets_after_a_real_answer);
    RUN_TEST(test_diag_other_negative_response_is_not_a_responder);
    RUN_TEST(test_diag_clock_wrap_and_restart);
    RUN_TEST(test_diag_wait_slow_multiframe_is_not_abandoned_and_completes);
    RUN_TEST(test_diag_wait_stalled_session_does_not_keep_the_wait_alive);
    RUN_TEST(test_diag_wait_reassembly_cap_is_a_hard_limit);
    RUN_TEST(test_diag_wait_pending_behaviour_is_unchanged);
    RUN_TEST(test_diag_wait_survives_tick_wraparound);
    RUN_TEST(test_diag_random_frames_never_break_the_collector);
}
