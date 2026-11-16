// RawFlowControl: the bounded flow-control responder armed by a raw send with fc=true.
#include <initializer_list>

#include "raw_flow_control.hpp"
#include "unity.h"

using obd2frame::RawFlowControl;

namespace
{
const uint8_t kFirstFrame[8] = {0x10, 0x14, 0x62, 0xF1, 0x90, 0x57, 0x30, 0x4C};

bool feed(RawFlowControl& fc, uint32_t now, uint32_t id, const uint8_t* d, size_t len, uint32_t& fcId, bool ext = false)
{
  fcId = 0;
  return fc.onFrame(now, id, ext, d, len, fcId);
}
}  // namespace

static void test_unarmed_never_answers(void)
{
  RawFlowControl fc;
  uint32_t       id;
  TEST_ASSERT_FALSE(feed(fc, 0, 0x7E8, kFirstFrame, 8, id));
}

static void test_first_frame_from_an_ecu_gets_flow_control_to_the_physical_id(void)
{
  RawFlowControl fc;
  fc.arm(1000, 1500, 0x7DF);
  uint32_t id;
  TEST_ASSERT_TRUE(feed(fc, 1010, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_EQUAL_UINT32(0x7E0, id);
  TEST_ASSERT_TRUE(feed(fc, 1011, 0x7EF, kFirstFrame, 8, id));
  TEST_ASSERT_EQUAL_UINT32(0x7E7, id);
}

static void test_each_ecu_is_answered_once_per_arming(void)
{
  RawFlowControl fc;
  fc.arm(0, 1500, 0x7DF);
  uint32_t id;
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E9, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 2, 0x7E9, kFirstFrame, 8, id));
  TEST_ASSERT_TRUE(feed(fc, 3, 0x7EA, kFirstFrame, 8, id));
  fc.arm(100, 1500, 0x7DF);  // the next send re-arms
  TEST_ASSERT_TRUE(feed(fc, 101, 0x7E9, kFirstFrame, 8, id));
}

static void test_only_first_frames_of_real_multiframe_messages_count(void)
{
  uint32_t      id;
  const uint8_t sf[8]  = {0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0x00};
  const uint8_t cf[8]  = {0x21, 1, 2, 3, 4, 5, 6, 7};
  const uint8_t fcf[8] = {0x30, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t tiny[8] = {0x10, 0x05, 1, 2, 3, 4, 5, 6};  // a "first frame" claiming 5 bytes: not multi-frame
  const uint8_t zero[8] = {0x10, 0x00, 1, 2, 3, 4, 5, 6};
  const uint8_t min8[8] = {0x10, 0x08, 1, 2, 3, 4, 5, 6};
  const uint8_t* never[] = {sf, cf, fcf, tiny, zero};
  for (const uint8_t* d : never)
  {
    RawFlowControl fc;
    fc.arm(0, 1500, 0x7DF);
    TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, d, 8, id));
  }
  RawFlowControl fc;
  fc.arm(0, 1500, 0x7DF);
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, nullptr, 8, id));  // no data: not even counted
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E8, min8, 8, id));
  RawFlowControl cut;
  cut.arm(0, 1500, 0x7DF);
  TEST_ASSERT_FALSE(feed(cut, 1, 0x7E8, kFirstFrame, 1, id));  // truncated frame
}

static void test_a_single_frame_reply_consumes_that_ecus_arming(void)
{
  // Bench finding: 7DF 02 01 00 with fc armed, both ECUs answer with single frames; a VIN request < 1.5 s later must not
  // get flow control from the leftover arming.
  RawFlowControl fc;
  uint32_t       id;
  const uint8_t  sf[8] = {0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0x00};
  fc.arm(0, 1500, 0x7DF);
  TEST_ASSERT_FALSE(feed(fc, 10, 0x7E8, sf, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 11, 0x7E9, sf, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 200, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 201, 0x7E9, kFirstFrame, 8, id));
  TEST_ASSERT_TRUE(feed(fc, 202, 0x7EA, kFirstFrame, 8, id));  // an ECU that has not answered yet is still covered
  TEST_ASSERT_EQUAL_UINT32(0x7E2, id);
}

static void test_the_responder_stands_down_once_every_addressed_ecu_has_answered(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(0, 1500, 0x7E1);
  TEST_ASSERT_TRUE(fc.armed());
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E9, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(fc.armed());
}

static void test_an_extended_id_frame_does_not_consume_the_arming(void)
{
  RawFlowControl fc;
  uint32_t       id;
  const uint8_t  sf[8] = {0x06, 0x41, 0x00, 0, 0, 0, 0, 0};
  fc.arm(0, 1500, 0x7DF);
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, sf, 8, id, true));
  TEST_ASSERT_TRUE(feed(fc, 2, 0x7E8, kFirstFrame, 8, id));
}

static void test_a_send_without_fc_never_leaves_an_older_arming_active(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.apply(true, 0, 1500, 0x7DF);
  TEST_ASSERT_TRUE(fc.armed());
  fc.apply(false, 100, 1500, 0x7E0);  // the next raw send has fc off
  TEST_ASSERT_FALSE(fc.armed());
  TEST_ASSERT_FALSE(feed(fc, 101, 0x7E8, kFirstFrame, 8, id));
  fc.apply(true, 200, 1500, 0x7E0);
  TEST_ASSERT_TRUE(feed(fc, 201, 0x7E8, kFirstFrame, 8, id));
}

static void test_only_ecu_response_ids_are_answered(void)
{
  RawFlowControl fc;
  fc.arm(0, 1500, 0x7DF);
  uint32_t id;
  for (uint32_t i : {0x7DFu, 0x7E0u, 0x7E7u, 0x7F0u, 0x100u, 0x7E8u - 1})
  {
    TEST_ASSERT_FALSE(feed(fc, 1, i, kFirstFrame, 8, id));
  }
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, kFirstFrame, 8, id, true));  // 29-bit id with the same number
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E8, kFirstFrame, 8, id));
}

static void test_window_expires_and_disarm_stops_it(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(1000, 1500, 0x7DF);
  TEST_ASSERT_TRUE(feed(fc, 2499, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 2500, 0x7EA, kFirstFrame, 8, id));
  fc.arm(5000, 1500, 0x7DF);
  fc.disarm();
  TEST_ASSERT_FALSE(feed(fc, 5001, 0x7E8, kFirstFrame, 8, id));
}

static void test_window_is_clock_wrap_safe(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(0xFFFFFF00u, 1500, 0x7DF);
  TEST_ASSERT_TRUE(feed(fc, 0x100u, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 0x700u, 0x7E9, kFirstFrame, 8, id));  // 1.8 s later
}

static void test_window_length_is_bounded(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(0, 0xFFFFFFFFu, 0x7DF);  // a huge window is capped
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, RawFlowControl::kMaxWindowMs + 1, 0x7E9, kFirstFrame, 8, id));
}

static void test_physical_send_arms_only_its_own_ecu(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(0, 1500, 0x7E3);  // request to ECU 3: only 0x7EB may be answered
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7EA, kFirstFrame, 8, id));
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7EB, kFirstFrame, 8, id));
  TEST_ASSERT_EQUAL_UINT32(0x7E3, id);
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7EC, kFirstFrame, 8, id));
  for (uint32_t k = 0; k < 8; k++)
  {
    RawFlowControl one;
    one.arm(0, 1500, 0x7E0 + k);
    for (uint32_t r = 0x7E8; r <= 0x7EF; r++)
    {
      TEST_ASSERT_EQUAL(r == 0x7E8 + k, feed(one, 1, r, kFirstFrame, 8, id));
    }
  }
}

static void test_functional_send_arms_every_ecu(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(0, 1500, 0x7DF);
  for (uint32_t r = 0x7E8; r <= 0x7EF; r++)
  {
    TEST_ASSERT_TRUE(feed(fc, 1, r, kFirstFrame, 8, id));
    TEST_ASSERT_EQUAL_UINT32(r - 8, id);
  }
}

static void test_other_request_ids_arm_nothing(void)
{
  RawFlowControl fc;
  uint32_t       id;
  for (uint32_t req : {0u, 0x100u, 0x7DEu, 0x7E8u, 0x7F0u, 0xFFFFFFFFu})
  {
    fc.arm(0, 1500, req);
    for (uint32_t r = 0x7E8; r <= 0x7EF; r++)
    {
      TEST_ASSERT_FALSE(feed(fc, 1, r, kFirstFrame, 8, id));
    }
  }
}

extern "C" void run_raw_flow_control_tests(void)
{
  RUN_TEST(test_unarmed_never_answers);
  RUN_TEST(test_first_frame_from_an_ecu_gets_flow_control_to_the_physical_id);
  RUN_TEST(test_each_ecu_is_answered_once_per_arming);
  RUN_TEST(test_only_first_frames_of_real_multiframe_messages_count);
  RUN_TEST(test_a_single_frame_reply_consumes_that_ecus_arming);
  RUN_TEST(test_the_responder_stands_down_once_every_addressed_ecu_has_answered);
  RUN_TEST(test_an_extended_id_frame_does_not_consume_the_arming);
  RUN_TEST(test_a_send_without_fc_never_leaves_an_older_arming_active);
  RUN_TEST(test_only_ecu_response_ids_are_answered);
  RUN_TEST(test_window_expires_and_disarm_stops_it);
  RUN_TEST(test_window_is_clock_wrap_safe);
  RUN_TEST(test_window_length_is_bounded);
  RUN_TEST(test_physical_send_arms_only_its_own_ecu);
  RUN_TEST(test_functional_send_arms_every_ecu);
  RUN_TEST(test_other_request_ids_arm_nothing);
}
