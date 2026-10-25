// can_raw: ring, batch/stats encoders, command parser and the capture/pump core (no ESP-IDF, fake lock and sink).
#include <cstring>
#include <vector>

#include "can_raw_core.hpp"
#include "unity.h"

using namespace can_raw;

static Frame mk(uint32_t id, uint32_t ts, std::initializer_list<uint8_t> d, bool tx = false, bool ext = false)
{
  uint8_t buf[8] = {};
  size_t  n      = 0;
  for (uint8_t b : d)
  {
    buf[n++] = b;
  }
  return makeFrame(id, ext, false, n, buf, ts, tx);
}

// ---- makeFrame ----

static void test_make_frame_clamps_and_masks(void)
{
  const uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  Frame         f    = makeFrame(0xFFFFFFFFu, true, false, 200, d, 7, false);
  TEST_ASSERT_EQUAL_UINT32(kIdExt | kIdMask, f.id);
  TEST_ASSERT_EQUAL_UINT8(8, f.ctl & kCtlLenMask);
  TEST_ASSERT_EQUAL_UINT8(8, f.data[7]);
  TEST_ASSERT_EQUAL_UINT32(7, f.ts_us);
}

static void test_make_frame_standard_id_has_no_flags(void)
{
  Frame f = makeFrame(0x7E8, false, false, 3, nullptr, 0, false);
  TEST_ASSERT_EQUAL_UINT32(0x7E8, f.id);
  TEST_ASSERT_EQUAL_UINT8(0, f.ctl & kCtlLenMask);  // null data: no bytes claimed
}

static void test_make_frame_remote_frame_has_no_data_and_tx_flag(void)
{
  const uint8_t d[8] = {9, 9, 9, 9, 9, 9, 9, 9};
  Frame         f    = makeFrame(0x123, false, true, 8, d, 0, true);
  TEST_ASSERT_EQUAL_UINT32(0x123u | kIdRtr, f.id);
  TEST_ASSERT_EQUAL_UINT8(0, f.ctl & kCtlLenMask);
  TEST_ASSERT_EQUAL_UINT8(kCtlTx, f.ctl & kCtlTx);
}

// ---- Ring ----

static void test_ring_unattached_refuses(void)
{
  Ring r;
  TEST_ASSERT_FALSE(r.attached());
  TEST_ASSERT_FALSE(r.push(mk(1, 1, {})));
  Frame out[2];
  TEST_ASSERT_EQUAL(0, r.peek(out, 2));
  r.discard(5);  // must not crash
  TEST_ASSERT_NULL(r.detach());
}

static void test_ring_full_refuses_and_keeps_order_across_wrap(void)
{
  Frame storage[4];
  Ring  r;
  r.attach(storage, 4);
  for (uint32_t i = 0; i < 4; i++)
  {
    TEST_ASSERT_TRUE(r.push(mk(i, i, {})));
  }
  TEST_ASSERT_FALSE(r.push(mk(99, 99, {})));
  TEST_ASSERT_EQUAL(4, r.size());
  r.discard(2);
  TEST_ASSERT_TRUE(r.push(mk(4, 4, {})));
  TEST_ASSERT_TRUE(r.push(mk(5, 5, {})));
  TEST_ASSERT_FALSE(r.push(mk(6, 6, {})));
  Frame out[8];
  TEST_ASSERT_EQUAL(4, r.peek(out, 8));
  for (uint32_t i = 0; i < 4; i++)
  {
    TEST_ASSERT_EQUAL_UINT32(i + 2, out[i].id);
  }
  TEST_ASSERT_EQUAL(4, r.peak());
}

static void test_ring_peek_is_bounded_and_discard_clamps(void)
{
  Frame storage[4];
  Ring  r;
  r.attach(storage, 4);
  r.push(mk(1, 1, {}));
  r.push(mk(2, 2, {}));
  Frame out[1];
  TEST_ASSERT_EQUAL(1, r.peek(out, 1));
  TEST_ASSERT_EQUAL(0, r.peek(out, 0));
  TEST_ASSERT_EQUAL(0, r.peek(nullptr, 3));
  r.discard(100);
  TEST_ASSERT_EQUAL(0, r.size());
  TEST_ASSERT_TRUE(r.push(mk(3, 3, {})));
}

static void test_ring_attach_rejects_empty_storage(void)
{
  Ring r;
  r.attach(nullptr, 4);
  TEST_ASSERT_FALSE(r.attached());
  Frame s[1];
  r.attach(s, 0);
  TEST_ASSERT_FALSE(r.attached());
}

static void test_ring_detach_returns_storage_once(void)
{
  Frame storage[2];
  Ring  r;
  r.attach(storage, 2);
  r.push(mk(1, 1, {}));
  TEST_ASSERT_EQUAL_PTR(storage, r.detach());
  TEST_ASSERT_NULL(r.detach());
  TEST_ASSERT_FALSE(r.push(mk(1, 1, {})));
  TEST_ASSERT_EQUAL(0, r.size());
}

// ---- encodeBatch ----

static void test_batch_single_frame_exact_bytes(void)
{
  Frame   f = mk(0x7E8, 0x01020304, {0x41, 0x0C});
  uint8_t out[kMaxBatchBytes] = {};
  size_t  used = 99;
  size_t  n    = encodeBatch(&f, 1, 0x0A0B, out, sizeof(out), used);
  const uint8_t expect[] = {0x10, 15,         // type, payload length (checked against n - 2 below)
                            0x0B, 0x0A,         // dropped
                            0x04, 0x03, 0x02, 0x01,  // base ts
                            0x00, 0x00,              // dt
                            0xE8, 0x07, 0x00, 0x00,  // id
                            0x02,                    // ctl: len 2, RX
                            0x41, 0x0C};
  TEST_ASSERT_EQUAL(sizeof(expect), n);
  TEST_ASSERT_EQUAL(1, used);
  TEST_ASSERT_EQUAL_UINT8(0x10, out[0]);
  TEST_ASSERT_EQUAL_UINT8(n - 2, out[1]);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect + 2, out + 2, sizeof(expect) - 2);
}

static void test_batch_tx_ext_rtr_encoding(void)
{
  Frame   f[3] = {mk(0x7E0, 100, {0x02, 0x01, 0x00}, true), mk(0x18DB33F1, 110, {1}, false, true),
                  makeFrame(0x55, false, true, 0, nullptr, 120, false)};
  uint8_t out[kMaxBatchBytes] = {};
  size_t  used = 0;
  size_t  n    = encodeBatch(f, 3, 0, out, sizeof(out), used);
  TEST_ASSERT_EQUAL(3, used);
  TEST_ASSERT_EQUAL(kBatchHeaderBytes + (7 + 3) + (7 + 1) + 7, n);
  TEST_ASSERT_EQUAL_UINT8(0x83, out[8 + 6]);   // TX bit + len 3
  const uint8_t* r2 = out + 8 + 10;
  TEST_ASSERT_EQUAL_INT16(10, static_cast<int16_t>(r2[0] | (r2[1] << 8)));
  uint32_t id2 = r2[2] | (r2[3] << 8) | (r2[4] << 16) | (static_cast<uint32_t>(r2[5]) << 24);
  TEST_ASSERT_EQUAL_UINT32(kIdExt | 0x18DB33F1u, id2);
  const uint8_t* r3 = r2 + 8;
  uint32_t id3 = r3[2] | (r3[3] << 8) | (r3[4] << 16) | (static_cast<uint32_t>(r3[5]) << 24);
  TEST_ASSERT_EQUAL_UINT32(kIdRtr | 0x55u, id3);
  TEST_ASSERT_EQUAL_UINT8(0, r3[6]);
}

static void test_batch_negative_dt_for_out_of_order_tx(void)
{
  Frame   f[2] = {mk(0x7E8, 5000, {1}), mk(0x7E0, 4200, {2}, true)};
  uint8_t out[kMaxBatchBytes] = {};
  size_t  used = 0;
  encodeBatch(f, 2, 0, out, sizeof(out), used);
  TEST_ASSERT_EQUAL(2, used);
  const uint8_t* r2 = out + 8 + 8;
  TEST_ASSERT_EQUAL_INT16(-800, static_cast<int16_t>(r2[0] | (r2[1] << 8)));
}

static void test_batch_dt_wraps_with_the_32_bit_clock(void)
{
  Frame   f[2] = {mk(1, 0xFFFFFFF0u, {}), mk(2, 0x10u, {})};
  uint8_t out[kMaxBatchBytes] = {};
  size_t  used = 0;
  encodeBatch(f, 2, 0, out, sizeof(out), used);
  TEST_ASSERT_EQUAL(2, used);
  const uint8_t* r2 = out + 8 + 7;
  TEST_ASSERT_EQUAL_INT16(32, static_cast<int16_t>(r2[0] | (r2[1] << 8)));
}

static void test_batch_splits_when_dt_does_not_fit_int16(void)
{
  Frame   f[3] = {mk(1, 0, {}), mk(2, 32767, {}), mk(3, 32767 + 32768, {})};
  uint8_t out[kMaxBatchBytes] = {};
  size_t  used = 0;
  encodeBatch(f, 3, 0, out, sizeof(out), used);
  TEST_ASSERT_EQUAL(2, used);  // third is 32768 after the second: starts the next batch
  Frame g[2] = {mk(1, 100000, {}), mk(2, 0, {})};
  encodeBatch(g, 2, 0, out, sizeof(out), used);
  TEST_ASSERT_EQUAL(1, used);
}

static void test_batch_never_exceeds_limit_and_length_byte_fits(void)
{
  Frame f[40];
  for (int i = 0; i < 40; i++)
  {
    f[i] = mk(0x7E8, 1000 + i, {1, 2, 3, 4, 5, 6, 7, 8});
  }
  uint8_t out[400] = {};
  size_t  used = 0;
  size_t  n    = encodeBatch(f, 40, 0, out, sizeof(out), used);
  TEST_ASSERT_TRUE(n <= kMaxBatchBytes);
  TEST_ASSERT_EQUAL(kBatchHeaderBytes + used * 15, n);
  TEST_ASSERT_EQUAL(16, used);
  TEST_ASSERT_EQUAL_UINT8(n - 2, out[1]);
}

static void test_batch_respects_small_output_buffer(void)
{
  Frame   f[2] = {mk(1, 0, {1, 2, 3, 4, 5, 6, 7, 8}), mk(2, 1, {1})};
  uint8_t out[64] = {};
  size_t  used = 7;
  TEST_ASSERT_EQUAL(0, encodeBatch(f, 2, 0, out, kBatchHeaderBytes + 6, used));
  TEST_ASSERT_EQUAL(0, used);
  size_t n = encodeBatch(f, 2, 0, out, kBatchHeaderBytes + 15, used);  // exactly the first record
  TEST_ASSERT_EQUAL(kBatchHeaderBytes + 15, n);
  TEST_ASSERT_EQUAL(1, used);
  n = encodeBatch(f, 2, 0, out, kBatchHeaderBytes + 15 + 7, used);  // second record (8 bytes) does not fit
  TEST_ASSERT_EQUAL(1, used);
}

static void test_batch_hostile_arguments(void)
{
  Frame   f = mk(1, 0, {});
  uint8_t out[32] = {};
  size_t  used = 5;
  TEST_ASSERT_EQUAL(0, encodeBatch(nullptr, 1, 0, out, sizeof(out), used));
  TEST_ASSERT_EQUAL(0, used);
  TEST_ASSERT_EQUAL(0, encodeBatch(&f, 0, 0, out, sizeof(out), used));
  TEST_ASSERT_EQUAL(0, encodeBatch(&f, 1, 0, nullptr, 32, used));
  TEST_ASSERT_EQUAL(0, encodeBatch(&f, 1, 0, out, 0, used));
  Frame bad = f;
  bad.ctl   = 0x0F;  // length nibble above 8 from a corrupt ring slot
  size_t n  = encodeBatch(&bad, 1, 0, out, sizeof(out), used);
  TEST_ASSERT_EQUAL(kBatchHeaderBytes + 7 + 8, n);
  TEST_ASSERT_EQUAL_UINT8(0x08, out[8 + 6]);
}

// ---- encodeStats / parseCommand / stopOnClose ----

static void test_stats_exact_bytes(void)
{
  Stats s{};
  s.flags       = 0x03;
  s.streamed    = 0x01020304;
  s.isrDropped  = 5;
  s.ringDropped = 6;
  s.wsDropped   = 7;
  s.capacity    = 256;
  s.peak        = 17;
  uint8_t out[kStatsBytes] = {};
  TEST_ASSERT_EQUAL(kStatsBytes, encodeStats(s, out, sizeof(out)));
  const uint8_t expect[] = {0x11, 21, 0x03, 4, 3, 2, 1, 5, 0, 0, 0, 6, 0, 0, 0, 7, 0, 0, 0, 0x00, 0x01, 17, 0};
  TEST_ASSERT_EQUAL(sizeof(expect), kStatsBytes);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, out, sizeof(expect));
  TEST_ASSERT_EQUAL(0, encodeStats(s, out, kStatsBytes - 1));
  TEST_ASSERT_EQUAL(0, encodeStats(s, nullptr, 64));
}

static void test_parse_command(void)
{
  const uint8_t a[] = {0xB0};
  const uint8_t b[] = {0xB1, 0xFF, 0xFF};
  const uint8_t c[] = {0xA0};
  TEST_ASSERT_EQUAL(static_cast<int>(Command::start), static_cast<int>(parseCommand(a, 1)));
  TEST_ASSERT_EQUAL(static_cast<int>(Command::stop), static_cast<int>(parseCommand(b, 3)));
  TEST_ASSERT_EQUAL(static_cast<int>(Command::none), static_cast<int>(parseCommand(c, 1)));
  TEST_ASSERT_EQUAL(static_cast<int>(Command::none), static_cast<int>(parseCommand(a, 0)));
  TEST_ASSERT_EQUAL(static_cast<int>(Command::none), static_cast<int>(parseCommand(nullptr, 1)));
}

static void test_stop_on_close_only_when_no_client_remains(void)
{
  TEST_ASSERT_TRUE(stopOnClose(0));
  TEST_ASSERT_FALSE(stopOnClose(1));
  TEST_ASSERT_FALSE(stopOnClose(20));
}

// ---- Core ----

struct FakeLock
{
  int  locks = 0;
  bool held  = false;
  struct Guard
  {
    FakeLock& l;
    explicit Guard(FakeLock& lk) : l(lk) { l.locks++; l.held = true; }
    ~Guard() { l.held = false; }
  };
};

struct Rig
{
  Core<FakeLock> core;
  Frame          storage[8];
  std::vector<std::vector<uint8_t>> sent;
  std::vector<size_t>               sentFrames;
  SinkResult                        result = SinkResult::accepted;
  bool                              lockHeldInSink = false;
  void (*duringSink)(Rig&)                     = nullptr;

  Rig() { TEST_ASSERT_TRUE(core.start(storage, 8)); }
  SinkResult send(const uint8_t* d, size_t n, size_t frames)
  {
    lockHeldInSink |= core.lockForTest().held;
    sent.emplace_back(d, d + n);
    sentFrames.push_back(frames);
    if (duringSink != nullptr)
    {
      duringSink(*this);
    }
    return result;
  }
  size_t pump(uint32_t isr = 0, size_t maxBatches = 4)
  {
    return core.pump([this](const uint8_t* d, size_t n, size_t f) { return send(d, n, f); }, isr, maxBatches);
  }
  static uint16_t droppedOf(const std::vector<uint8_t>& b) { return static_cast<uint16_t>(b[2] | (b[3] << 8)); }
};

static void test_core_tap_is_a_noop_while_inactive(void)
{
  Core<FakeLock> c;
  c.tap(mk(1, 1, {}));
  TEST_ASSERT_EQUAL(0, c.lockForTest().locks);  // not even a lock when idle
  TEST_ASSERT_FALSE(c.active());
  Stats s = c.snapshot(0, 0);
  TEST_ASSERT_EQUAL(0, s.ringDropped);
}

static void test_core_start_twice_keeps_first_storage(void)
{
  Rig   r;
  Frame other[2];
  TEST_ASSERT_FALSE(r.core.start(other, 2));
  r.core.tap(mk(1, 1, {}));
  Frame* back = r.core.stop();
  TEST_ASSERT_EQUAL_PTR(r.storage, back);
  TEST_ASSERT_NULL(r.core.stop());
  TEST_ASSERT_FALSE(r.core.active());
  r.core.tap(mk(2, 2, {}));  // after stop: ignored, storage untouched
  TEST_ASSERT_EQUAL_UINT32(1, r.storage[0].id);
}

static void test_core_start_rejects_bad_storage(void)
{
  Core<FakeLock> c;
  TEST_ASSERT_FALSE(c.start(nullptr, 4));
  Frame f[1];
  TEST_ASSERT_FALSE(c.start(f, 0));
  TEST_ASSERT_FALSE(c.active());
}

static void test_core_overflow_counts_ring_drops(void)
{
  Rig r;
  for (uint32_t i = 0; i < 11; i++)
  {
    r.core.tap(mk(i, i, {}));
  }
  Stats s = r.core.snapshot(0, 0);
  TEST_ASSERT_EQUAL(3, s.ringDropped);
  TEST_ASSERT_EQUAL(8, s.peak);
  TEST_ASSERT_EQUAL(8, s.capacity);
}

static void test_pump_accepted_discards_and_counts(void)
{
  Rig r;
  for (uint32_t i = 0; i < 3; i++)
  {
    r.core.tap(mk(0x7E8, 10 * i, {1}));
  }
  TEST_ASSERT_EQUAL(3, r.pump());
  TEST_ASSERT_EQUAL(1, r.sent.size());
  TEST_ASSERT_EQUAL(3, r.sentFrames[0]);
  TEST_ASSERT_FALSE(r.lockHeldInSink);
  TEST_ASSERT_EQUAL(0, r.pump());  // ring empty: nothing more is sent
  TEST_ASSERT_EQUAL(1, r.sent.size());
  TEST_ASSERT_EQUAL(3, r.core.snapshot(0, 0).streamed);
}

static void test_pump_reports_drops_once_across_causes(void)
{
  Rig r;
  for (uint32_t i = 0; i < 10; i++)  // 2 ring drops
  {
    r.core.tap(mk(1, i, {}));
  }
  r.core.addWsDropped(4);
  r.pump(5, 1);  // isr drops 5
  TEST_ASSERT_EQUAL(11, Rig::droppedOf(r.sent[0]));
  r.core.tap(mk(1, 100, {}));
  r.pump(5, 1);
  TEST_ASSERT_EQUAL(0, Rig::droppedOf(r.sent[1]));
  r.core.tap(mk(1, 200, {}));
  r.pump(9, 1);  // 4 more isr drops
  TEST_ASSERT_EQUAL(4, Rig::droppedOf(r.sent[2]));
}

static void test_pump_drop_field_saturates_and_carries(void)
{
  Rig r;
  r.core.tap(mk(1, 0, {}));
  r.core.tap(mk(1, 100000, {}));  // more than int16 apart: one frame per batch
  r.core.tap(mk(1, 200000, {}));
  r.pump(70000, 4);
  TEST_ASSERT_EQUAL(3, r.sent.size());
  TEST_ASSERT_EQUAL(65535, Rig::droppedOf(r.sent[0]));
  TEST_ASSERT_EQUAL(70000 - 65535, Rig::droppedOf(r.sent[1]));
  TEST_ASSERT_EQUAL(0, Rig::droppedOf(r.sent[2]));
}

static void test_pump_busy_keeps_frames_and_drop_report(void)
{
  Rig r;
  r.core.tap(mk(0x7E8, 1, {1}));
  r.result = SinkResult::busy;
  TEST_ASSERT_EQUAL(0, r.pump(3, 4));
  TEST_ASSERT_EQUAL(1, r.sent.size());  // stops at the first busy answer, no spinning
  r.result = SinkResult::accepted;
  TEST_ASSERT_EQUAL(1, r.pump(3, 4));
  TEST_ASSERT_EQUAL(2, r.sent.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(r.sent[0].data() + 4, r.sent[1].data() + 4, r.sent[0].size() - 4);
  TEST_ASSERT_EQUAL(3, Rig::droppedOf(r.sent[1]));
  TEST_ASSERT_EQUAL(0, r.core.snapshot(3, 0).wsDropped);
}

static void test_pump_dropped_discards_and_counts_ws(void)
{
  Rig r;
  r.core.tap(mk(1, 1, {}));
  r.core.tap(mk(2, 2, {}));
  r.result = SinkResult::dropped;
  TEST_ASSERT_EQUAL(0, r.pump(0, 1));
  Stats s = r.core.snapshot(0, 0);
  TEST_ASSERT_EQUAL(2, s.wsDropped);
  TEST_ASSERT_EQUAL(0, s.streamed);
  r.result = SinkResult::accepted;
  r.core.tap(mk(3, 3, {}));
  r.pump(0, 1);
  TEST_ASSERT_EQUAL(2, Rig::droppedOf(r.sent.back()));  // the loss is told to the client on the next batch
}

static void test_pump_respects_max_batches(void)
{
  Rig r;
  for (uint32_t i = 0; i < 8; i++)
  {
    r.core.tap(mk(1, i * 40000, {}));  // each frame is more than int16 apart: one frame per batch
  }
  TEST_ASSERT_EQUAL(3, r.pump(0, 3));
  TEST_ASSERT_EQUAL(3, r.sent.size());
  TEST_ASSERT_EQUAL(0, r.pump(0, 0));
}

static void restartDuringSink(Rig& r)
{
  Frame* back = r.core.stop();
  (void)back;
  TEST_ASSERT_TRUE(r.core.start(r.storage, 8));
  r.core.tap(mk(0x99, 5, {}));
  r.duringSink = nullptr;
}

static void test_pump_does_not_discard_frames_of_a_newer_session(void)
{
  Rig r;
  r.core.tap(mk(1, 1, {}));
  r.duringSink = restartDuringSink;
  r.pump(0, 1);
  Stats s = r.core.snapshot(0, 0);
  TEST_ASSERT_EQUAL(0, s.streamed);  // the accepted batch belonged to the old session
  r.pump(0, 1);                      // the new session's frame is still there
  TEST_ASSERT_EQUAL(2, r.sent.size());
  TEST_ASSERT_EQUAL_UINT8(0x99, r.sent[1][8 + 2]);
}

static void test_stop_on_last_client_close_ends_capture(void)
{
  Rig r;
  r.core.tap(mk(1, 1, {}));
  uint32_t remaining = 0;
  if (stopOnClose(remaining))
  {
    TEST_ASSERT_EQUAL_PTR(r.storage, r.core.stop());
  }
  TEST_ASSERT_FALSE(r.core.active());
  TEST_ASSERT_EQUAL(0, r.pump());
  TEST_ASSERT_EQUAL(0, r.sent.size());
}

static void test_snapshot_resets_peak_and_zeroes_on_restart(void)
{
  Rig r;
  for (uint32_t i = 0; i < 5; i++)
  {
    r.core.tap(mk(1, i, {}));
  }
  TEST_ASSERT_EQUAL(5, r.core.snapshot(0, 0x02).peak);
  TEST_ASSERT_EQUAL_UINT8(0x02, r.core.snapshot(0, 0x02).flags & 0x02);
  TEST_ASSERT_EQUAL(5, r.core.snapshot(0, 0).peak);  // current fill is the floor of the next window
  r.core.stop();
  Frame s2[4];
  TEST_ASSERT_TRUE(r.core.start(s2, 4));
  Stats s = r.core.snapshot(0, 0);
  TEST_ASSERT_EQUAL(0, s.peak);
  TEST_ASSERT_EQUAL(0, s.ringDropped);
  TEST_ASSERT_EQUAL(0, s.streamed);
}

extern "C" void run_can_raw_tests(void)
{
  RUN_TEST(test_make_frame_clamps_and_masks);
  RUN_TEST(test_make_frame_standard_id_has_no_flags);
  RUN_TEST(test_make_frame_remote_frame_has_no_data_and_tx_flag);
  RUN_TEST(test_ring_unattached_refuses);
  RUN_TEST(test_ring_full_refuses_and_keeps_order_across_wrap);
  RUN_TEST(test_ring_peek_is_bounded_and_discard_clamps);
  RUN_TEST(test_ring_attach_rejects_empty_storage);
  RUN_TEST(test_ring_detach_returns_storage_once);
  RUN_TEST(test_batch_single_frame_exact_bytes);
  RUN_TEST(test_batch_tx_ext_rtr_encoding);
  RUN_TEST(test_batch_negative_dt_for_out_of_order_tx);
  RUN_TEST(test_batch_dt_wraps_with_the_32_bit_clock);
  RUN_TEST(test_batch_splits_when_dt_does_not_fit_int16);
  RUN_TEST(test_batch_never_exceeds_limit_and_length_byte_fits);
  RUN_TEST(test_batch_respects_small_output_buffer);
  RUN_TEST(test_batch_hostile_arguments);
  RUN_TEST(test_stats_exact_bytes);
  RUN_TEST(test_parse_command);
  RUN_TEST(test_stop_on_close_only_when_no_client_remains);
  RUN_TEST(test_core_tap_is_a_noop_while_inactive);
  RUN_TEST(test_core_start_twice_keeps_first_storage);
  RUN_TEST(test_core_start_rejects_bad_storage);
  RUN_TEST(test_core_overflow_counts_ring_drops);
  RUN_TEST(test_pump_accepted_discards_and_counts);
  RUN_TEST(test_pump_reports_drops_once_across_causes);
  RUN_TEST(test_pump_drop_field_saturates_and_carries);
  RUN_TEST(test_pump_busy_keeps_frames_and_drop_report);
  RUN_TEST(test_pump_dropped_discards_and_counts_ws);
  RUN_TEST(test_pump_respects_max_batches);
  RUN_TEST(test_pump_does_not_discard_frames_of_a_newer_session);
  RUN_TEST(test_stop_on_last_client_close_ends_capture);
  RUN_TEST(test_snapshot_resets_peak_and_zeroes_on_restart);
}
