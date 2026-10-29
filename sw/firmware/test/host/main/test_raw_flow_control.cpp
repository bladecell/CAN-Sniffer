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
  fc.arm(1000, 1500);
  uint32_t id;
  TEST_ASSERT_TRUE(feed(fc, 1010, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_EQUAL_UINT32(0x7E0, id);
  TEST_ASSERT_TRUE(feed(fc, 1011, 0x7EF, kFirstFrame, 8, id));
  TEST_ASSERT_EQUAL_UINT32(0x7E7, id);
}

static void test_each_ecu_is_answered_once_per_arming(void)
{
  RawFlowControl fc;
  fc.arm(0, 1500);
  uint32_t id;
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E9, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 2, 0x7E9, kFirstFrame, 8, id));
  TEST_ASSERT_TRUE(feed(fc, 3, 0x7EA, kFirstFrame, 8, id));
  fc.arm(100, 1500);  // the next send re-arms
  TEST_ASSERT_TRUE(feed(fc, 101, 0x7E9, kFirstFrame, 8, id));
}

static void test_only_first_frames_of_real_multiframe_messages_count(void)
{
  RawFlowControl fc;
  fc.arm(0, 1500);
  uint32_t      id;
  const uint8_t sf[8]  = {0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0x00};
  const uint8_t cf[8]  = {0x21, 1, 2, 3, 4, 5, 6, 7};
  const uint8_t fcf[8] = {0x30, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t tiny[8] = {0x10, 0x05, 1, 2, 3, 4, 5, 6};  // a "first frame" claiming 5 bytes: not multi-frame
  const uint8_t zero[8] = {0x10, 0x00, 1, 2, 3, 4, 5, 6};
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, sf, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, cf, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, fcf, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, tiny, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, zero, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, kFirstFrame, 1, id));  // truncated frame
  TEST_ASSERT_FALSE(feed(fc, 1, 0x7E8, nullptr, 8, id));
  const uint8_t min8[8] = {0x10, 0x08, 1, 2, 3, 4, 5, 6};
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E8, min8, 8, id));
}

static void test_only_ecu_response_ids_are_answered(void)
{
  RawFlowControl fc;
  fc.arm(0, 1500);
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
  fc.arm(1000, 1500);
  TEST_ASSERT_TRUE(feed(fc, 2499, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 2500, 0x7EA, kFirstFrame, 8, id));
  fc.arm(5000, 1500);
  fc.disarm();
  TEST_ASSERT_FALSE(feed(fc, 5001, 0x7E8, kFirstFrame, 8, id));
}

static void test_window_is_clock_wrap_safe(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(0xFFFFFF00u, 1500);
  TEST_ASSERT_TRUE(feed(fc, 0x100u, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, 0x700u, 0x7E9, kFirstFrame, 8, id));  // 1.8 s later
}

static void test_window_length_is_bounded(void)
{
  RawFlowControl fc;
  uint32_t       id;
  fc.arm(0, 0xFFFFFFFFu);  // a huge window is capped
  TEST_ASSERT_TRUE(feed(fc, 1, 0x7E8, kFirstFrame, 8, id));
  TEST_ASSERT_FALSE(feed(fc, RawFlowControl::kMaxWindowMs + 1, 0x7E9, kFirstFrame, 8, id));
}

extern "C" void run_raw_flow_control_tests(void)
{
  RUN_TEST(test_unarmed_never_answers);
  RUN_TEST(test_first_frame_from_an_ecu_gets_flow_control_to_the_physical_id);
  RUN_TEST(test_each_ecu_is_answered_once_per_arming);
  RUN_TEST(test_only_first_frames_of_real_multiframe_messages_count);
  RUN_TEST(test_only_ecu_response_ids_are_answered);
  RUN_TEST(test_window_expires_and_disarm_stops_it);
  RUN_TEST(test_window_is_clock_wrap_safe);
  RUN_TEST(test_window_length_is_bounded);
}
