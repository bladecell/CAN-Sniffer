// P2.10c: access-control rules with a fake clock, RNG, key derivation and storage. Includes the hostile cases:
// replay, expiry, session overflow, brute force, claim outside the window, claim twice, storage failure.
#include <cstring>
#include <initializer_list>

#include "../../../main/include/auth_core.hpp"
#include "../../../main/include/auth_http.hpp"
#include "unity.h"

namespace
{
using namespace auth;

constexpr int64_t kSec = 1000000;

// ---- fakes -----------------------------------------------------------------------------------------------------
int      lockDepth;
int      kdfCalls;
bool     kdfSawLock;
bool     kdfFails;
bool     rngDead;
uint32_t rngCounter;
bool     saveFails;
bool     eraseFails;
int      saves;
int      erases;
uint8_t  stored[kRecordSize];
bool     haveStored;

void fakeLock() { lockDepth++; }
void fakeUnlock() { lockDepth--; }

bool fakeKdf(const uint8_t* pin, size_t pinLen, const uint8_t* salt, size_t saltLen, uint32_t it, uint8_t* out)
{
  kdfCalls++;
  kdfSawLock = kdfSawLock || lockDepth != 0;
  if (kdfFails)
  {
    return false;
  }
  uint32_t h = 2166136261u ^ it;
  for (size_t i = 0; i < pinLen; i++) h = (h ^ pin[i]) * 16777619u;
  for (size_t i = 0; i < saltLen; i++) h = (h ^ salt[i]) * 16777619u;
  for (size_t i = 0; i < kHashLen; i++)
  {
    h      = (h ^ (i + 1)) * 16777619u;
    out[i] = static_cast<uint8_t>(h >> 13);
  }
  return true;
}

void fakeRng(uint8_t* b, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    rngCounter = rngCounter * 1664525u + 1013904223u;
    b[i]       = rngDead ? 0 : static_cast<uint8_t>((rngCounter >> 24) | 1);
  }
}

bool fakeSave(const uint8_t* rec, size_t len)
{
  if (saveFails || len != kRecordSize)
  {
    return false;
  }
  memcpy(stored, rec, len);
  haveStored = true;
  saves++;
  return true;
}

bool fakeErase()
{
  if (eraseFails)
  {
    return false;
  }
  haveStored = false;
  erases++;
  return true;
}

int64_t fakeNow;
int64_t fakeClock() { return fakeNow; }

const Platform kPlat{fakeKdf, fakeRng, fakeSave, fakeErase, fakeLock, fakeUnlock, 10000, fakeClock};

void reset()
{
  lockDepth = kdfCalls = saves = erases = 0;
  kdfSawLock = kdfFails = rngDead = saveFails = eraseFails = haveStored = false;
  rngCounter = 12345;
  fakeNow = 0;
}

struct Fixture
{
  AuthCore core{kPlat};
  uint32_t retry = 0;
  char     tok[SessionTable::kTokenLen + 1] = {};
  Fixture() { reset(); core.load(State::Unclaimed, nullptr); }
  char     id[kClaimIdLen + 1] = {};
  Result post(const char* pin, int64_t now, int64_t press = 0)
  {
    fakeNow = now;
    return core.claimPost(pin, strlen(pin), press, now, id);
  }
  ClaimView poll(int64_t now, int64_t press = 0) { return core.claimStatus(id, press, now); }
  // Whole claim: post just before the press, poll after it. Ok when it committed (token in `tok`).
  Result claim(const char* pin, int64_t press, int64_t now)
  {
    Result r = post(pin, press - 1);
    if (r != Result::Ok) return r;
    const ClaimView v = poll(now, press);
    if (v.status != ClaimStatus::Committed || !v.hasToken) return Result::Internal;
    memcpy(tok, v.token, sizeof(tok));
    return Result::Ok;
  }
  Result login(const char* pin, int64_t now) { return core.login(pin, strlen(pin), now, tok, &retry); }
};

// A claimed fixture: claimed at t=100 s with press at t=90 s, PIN 123456. Token in `tok`.
Fixture claimed()
{
  Fixture f;
  TEST_ASSERT_EQUAL(Result::Ok, f.claim("123456", 90 * kSec, 100 * kSec));
  return f;
}

// ---- PIN record and primitives ---------------------------------------------------------------------------------
void test_pin_policy(void)
{
  TEST_ASSERT_TRUE(isValidPin("000000", 6));
  TEST_ASSERT_TRUE(isValidPin("123456", 6));
  TEST_ASSERT_FALSE(isValidPin("12345", 5));
  TEST_ASSERT_FALSE(isValidPin("1234567", 7));
  TEST_ASSERT_FALSE(isValidPin("12345a", 6));
  TEST_ASSERT_FALSE(isValidPin("12 456", 6));
  TEST_ASSERT_FALSE(isValidPin("-12345", 6));
  TEST_ASSERT_FALSE(isValidPin("12345\n", 6));
  TEST_ASSERT_FALSE(isValidPin("", 0));
  TEST_ASSERT_FALSE(isValidPin(nullptr, 6));
  TEST_ASSERT_FALSE(isValidPin("12345\0" "6", 7));  // embedded NUL
}

void test_constant_time_eq(void)
{
  const uint8_t a[4] = {1, 2, 3, 4};
  uint8_t       b[4] = {1, 2, 3, 4};
  TEST_ASSERT_TRUE(constantTimeEq(a, b, 4));
  b[3] = 5;
  TEST_ASSERT_FALSE(constantTimeEq(a, b, 4));
  b[3] = 4;
  b[0] = 0;
  TEST_ASSERT_FALSE(constantTimeEq(a, b, 4));
  TEST_ASSERT_TRUE(constantTimeEq(a, b, 0));
}

void test_record_round_trip_and_corruption(void)
{
  PinRecord r{};
  r.iterations = 10000;
  for (size_t i = 0; i < kSaltLen; i++) r.salt[i] = static_cast<uint8_t>(i);
  for (size_t i = 0; i < kHashLen; i++) r.hash[i] = static_cast<uint8_t>(0xA0 + i);
  uint8_t blob[kRecordSize];
  encodeRecord(r, blob);
  PinRecord out{};
  TEST_ASSERT_TRUE(decodeRecord(blob, sizeof(blob), &out));
  TEST_ASSERT_EQUAL_UINT32(10000, out.iterations);
  TEST_ASSERT_EQUAL_MEMORY(r.salt, out.salt, kSaltLen);
  TEST_ASSERT_EQUAL_MEMORY(r.hash, out.hash, kHashLen);

  TEST_ASSERT_FALSE(decodeRecord(blob, sizeof(blob) - 1, &out));
  TEST_ASSERT_FALSE(decodeRecord(blob, sizeof(blob) + 1, &out));
  TEST_ASSERT_FALSE(decodeRecord(blob, 0, &out));
  TEST_ASSERT_FALSE(decodeRecord(nullptr, sizeof(blob), &out));
  uint8_t bad[kRecordSize];
  memcpy(bad, blob, sizeof(bad));
  bad[0] = 2;
  TEST_ASSERT_FALSE(decodeRecord(bad, sizeof(bad), &out));
  memcpy(bad, blob, sizeof(bad));
  memset(bad + 1, 0, 4);  // zero iterations
  TEST_ASSERT_FALSE(decodeRecord(bad, sizeof(bad), &out));
  memset(bad + 1, 0xFF, 4);  // 4 billion iterations would hang a worker
  TEST_ASSERT_FALSE(decodeRecord(bad, sizeof(bad), &out));
  bad[1] = 0xE7; bad[2] = 0x03; bad[3] = 0; bad[4] = 0;  // 999
  TEST_ASSERT_FALSE(decodeRecord(bad, sizeof(bad), &out));
}

void test_backoff_schedule(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, LoginBackoff::delayS(0));
  TEST_ASSERT_EQUAL_UINT32(1, LoginBackoff::delayS(1));
  TEST_ASSERT_EQUAL_UINT32(2, LoginBackoff::delayS(2));
  TEST_ASSERT_EQUAL_UINT32(4, LoginBackoff::delayS(3));
  TEST_ASSERT_EQUAL_UINT32(512, LoginBackoff::delayS(10));
  TEST_ASSERT_EQUAL_UINT32(900, LoginBackoff::delayS(11));
  TEST_ASSERT_EQUAL_UINT32(900, LoginBackoff::delayS(32));
  TEST_ASSERT_EQUAL_UINT32(900, LoginBackoff::delayS(0xFFFFFFFFu));
}

void test_session_token_shape_and_rng_failure(void)
{
  reset();
  SessionTable t;
  char a[SessionTable::kTokenLen + 1], b[SessionTable::kTokenLen + 1];
  TEST_ASSERT_TRUE(t.create(1000, fakeRng, a));
  TEST_ASSERT_TRUE(t.create(1000, fakeRng, b));
  TEST_ASSERT_EQUAL_UINT32(SessionTable::kTokenLen, strlen(a));
  TEST_ASSERT_TRUE(strcmp(a, b) != 0);
  for (const char* p = a; *p; p++) TEST_ASSERT_TRUE((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f'));
  rngDead = true;
  char c[SessionTable::kTokenLen + 1] = "unchanged";
  TEST_ASSERT_FALSE(t.create(1000, fakeRng, c));  // an all-zero token is never issued
  TEST_ASSERT_EQUAL_STRING("unchanged", c);
}

void test_session_lookup_rejects_malformed_tokens(void)
{
  reset();
  SessionTable t;
  char a[SessionTable::kTokenLen + 1];
  TEST_ASSERT_TRUE(t.create(0, fakeRng, a));
  TEST_ASSERT_TRUE(t.validate(a, 1));
  TEST_ASSERT_FALSE(t.validate(nullptr, 1));
  TEST_ASSERT_FALSE(t.validate("", 1));
  char shortTok[SessionTable::kTokenLen];
  memcpy(shortTok, a, sizeof(shortTok) - 1);
  shortTok[sizeof(shortTok) - 1] = '\0';
  TEST_ASSERT_FALSE(t.validate(shortTok, 1));
  char longTok[SessionTable::kTokenLen + 3];
  memcpy(longTok, a, SessionTable::kTokenLen);
  longTok[SessionTable::kTokenLen]     = 'a';
  longTok[SessionTable::kTokenLen + 1] = '\0';
  TEST_ASSERT_FALSE(t.validate(longTok, 1));
  char flipped[SessionTable::kTokenLen + 1];
  memcpy(flipped, a, sizeof(flipped));
  flipped[31] = flipped[31] == '0' ? '1' : '0';
  TEST_ASSERT_FALSE(t.validate(flipped, 1));
  TEST_ASSERT_FALSE(t.validate("00000000000000000000000000000000", 1));  // empty slots hold zeros, not a wildcard
}

// ---- claim (two steps) -----------------------------------------------------------------------------------------
void test_unclaimed_device_refuses_everything_but_claim(void)
{
  Fixture f;
  TEST_ASSERT_EQUAL(Result::NotClaimed, f.core.check("anything", 0));
  TEST_ASSERT_EQUAL(Result::NotClaimed, f.login("123456", 0));
  TEST_ASSERT_EQUAL(0, kdfCalls);
  uint32_t r;
  TEST_ASSERT_EQUAL(Result::NotClaimed, f.core.changePin("123456", 6, "654321", 6, 0, &r));
}

void test_post_parks_the_pin_and_changes_nothing_else(void)
{
  Fixture f;
  TEST_ASSERT_EQUAL(Result::Ok, f.post("123456", 10 * kSec));
  TEST_ASSERT_EQUAL_UINT32(kClaimIdLen, strlen(f.id));
  TEST_ASSERT_EQUAL(1, kdfCalls);
  TEST_ASSERT_EQUAL(0, saves);
  TEST_ASSERT_EQUAL(State::Unclaimed, f.core.state());
  TEST_ASSERT_EQUAL(Result::NotClaimed, f.core.check("anything", 11 * kSec));
  const ClaimView v = f.poll(25 * kSec);
  TEST_ASSERT_EQUAL(ClaimStatus::Pending, v.status);
  TEST_ASSERT_EQUAL_UINT32(45, v.expiresInS);
  TEST_ASSERT_FALSE(v.hasToken);
  const AuthCore::Status s = f.core.status(0, 25 * kSec);
  TEST_ASSERT_TRUE(s.pending);
  TEST_ASSERT_EQUAL_UINT32(45, s.pendingExpiresS);
}

void test_press_after_the_post_commits_and_hands_over_the_token_once(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  ClaimView v = f.poll(20 * kSec, 15 * kSec);
  TEST_ASSERT_EQUAL(ClaimStatus::Committed, v.status);
  TEST_ASSERT_TRUE(v.hasToken);
  TEST_ASSERT_EQUAL(State::Claimed, f.core.state());
  TEST_ASSERT_EQUAL(1, saves);
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(v.token, 21 * kSec));
  PinRecord rec;
  TEST_ASSERT_TRUE(decodeRecord(stored, sizeof(stored), &rec));
  TEST_ASSERT_NULL(memmem(stored, sizeof(stored), "123456", 6));
  v = f.poll(22 * kSec, 15 * kSec);  // second poll: committed, no token again
  TEST_ASSERT_EQUAL(ClaimStatus::Committed, v.status);
  TEST_ASSERT_FALSE(v.hasToken);
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", 23 * kSec));
  TEST_ASSERT_FALSE(f.core.status(0, 24 * kSec).pending);
}

void test_a_press_before_the_post_does_not_count(void)
{
  Fixture f;
  f.post("123456", 100 * kSec);
  TEST_ASSERT_EQUAL(ClaimStatus::Pending, f.poll(101 * kSec, 90 * kSec).status);
  TEST_ASSERT_EQUAL(ClaimStatus::Pending, f.poll(102 * kSec, 100 * kSec).status);  // same instant is not "after"
  TEST_ASSERT_EQUAL(State::Unclaimed, f.core.state());
  TEST_ASSERT_EQUAL(0, saves);
}

void test_window_is_60_seconds_from_the_post(void)
{
  Fixture f;
  f.post("123456", 100 * kSec);
  TEST_ASSERT_EQUAL(ClaimStatus::Committed, f.poll(161 * kSec, 160 * kSec).status);  // press at exactly +60 s
  Fixture g;
  g.post("123456", 100 * kSec);
  TEST_ASSERT_EQUAL(ClaimStatus::Expired, g.poll(170 * kSec, 161 * kSec).status);   // press at +61 s
  TEST_ASSERT_EQUAL(State::Unclaimed, g.core.state());
  Fixture h;
  h.post("123456", 100 * kSec);
  TEST_ASSERT_EQUAL(ClaimStatus::Pending, h.poll(160 * kSec).status);
  TEST_ASSERT_EQUAL(ClaimStatus::Expired, h.poll(161 * kSec).status);               // never pressed
  TEST_ASSERT_EQUAL(ClaimStatus::Expired, h.poll(200 * kSec).status);
  TEST_ASSERT_FALSE(h.core.status(0, 162 * kSec).pending);
}

void test_second_post_cancels_both(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  char firstId[kClaimIdLen + 1];
  memcpy(firstId, f.id, sizeof(firstId));
  TEST_ASSERT_EQUAL(Result::ClaimConflict, f.post("654321", 20 * kSec));
  TEST_ASSERT_EQUAL(1, kdfCalls);  // the conflict path derives nothing
  TEST_ASSERT_EQUAL(ClaimStatus::Conflict, f.core.claimStatus(firstId, 0, 21 * kSec).status);
  TEST_ASSERT_FALSE(f.core.status(0, 21 * kSec).pending);
  // the press that follows has nothing to commit
  TEST_ASSERT_EQUAL(ClaimStatus::Conflict, f.core.claimStatus(firstId, 22 * kSec, 23 * kSec).status);
  TEST_ASSERT_EQUAL(State::Unclaimed, f.core.state());
  TEST_ASSERT_EQUAL(0, saves);
  // both can start over
  TEST_ASSERT_EQUAL(Result::Ok, f.post("123456", 30 * kSec));
  TEST_ASSERT_EQUAL(ClaimStatus::Committed, f.poll(40 * kSec, 35 * kSec).status);
}

void test_bad_format_post_leaves_a_pending_claim_alone(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  TEST_ASSERT_EQUAL(Result::BadFormat, f.post("12", 11 * kSec));
  TEST_ASSERT_EQUAL(Result::BadFormat, f.post("abcdef", 12 * kSec));
  Fixture g;
  const char* bad[] = {"", "12345", "1234567", "12345a"};
  for (const char* p : bad) TEST_ASSERT_EQUAL(Result::BadFormat, g.post(p, 5 * kSec));
  TEST_ASSERT_EQUAL(0, kdfCalls);  // a bad PIN never reaches the key derivation
  TEST_ASSERT_FALSE(g.core.status(0, 6 * kSec).pending);
}

void test_post_after_the_press_but_before_anyone_polled_loses_to_the_first(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  char firstId[kClaimIdLen + 1];
  memcpy(firstId, f.id, sizeof(firstId));
  // press at 15 s, nobody polled; a second client posts at 20 s: the first claim commits, the second is refused
  TEST_ASSERT_EQUAL(Result::AlreadyClaimed, f.post("654321", 20 * kSec, 15 * kSec));
  TEST_ASSERT_EQUAL(ClaimStatus::Committed, f.core.claimStatus(firstId, 15 * kSec, 21 * kSec).status);
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", 22 * kSec));
  TEST_ASSERT_EQUAL(Result::BadPin, f.login("654321", 100 * kSec));
}

void test_post_on_claimed_or_unavailable_device(void)
{
  Fixture f = claimed();
  const int before = kdfCalls;
  TEST_ASSERT_EQUAL(Result::AlreadyClaimed, f.post("654321", 200 * kSec));
  TEST_ASSERT_EQUAL(before, kdfCalls);
  reset();
  AuthCore core(kPlat);
  core.load(State::Unavailable, nullptr);
  char id[kClaimIdLen + 1];
  TEST_ASSERT_EQUAL(Result::Unavailable, core.claimPost("123456", 6, 0, 0, id));
  TEST_ASSERT_EQUAL(0, kdfCalls);
}

void test_commit_storage_failure_reports_failed_and_changes_nothing(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  saveFails = true;
  TEST_ASSERT_EQUAL(ClaimStatus::Failed, f.poll(20 * kSec, 15 * kSec).status);
  TEST_ASSERT_EQUAL(State::Unclaimed, f.core.state());
  TEST_ASSERT_FALSE(f.core.status(0, 21 * kSec).pending);
  saveFails = false;
  TEST_ASSERT_EQUAL(Result::Ok, f.claim("123456", 40 * kSec, 41 * kSec));  // can try again
}

void test_commit_with_a_dead_rng_rolls_the_record_back(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  rngDead = true;  // the session token cannot be made
  TEST_ASSERT_EQUAL(ClaimStatus::Failed, f.poll(20 * kSec, 15 * kSec).status);
  TEST_ASSERT_EQUAL(State::Unclaimed, f.core.state());
  TEST_ASSERT_FALSE(haveStored);
}

void test_kdf_or_rng_failure_at_post_parks_nothing(void)
{
  Fixture f;
  kdfFails = true;
  TEST_ASSERT_EQUAL(Result::Internal, f.post("123456", 10 * kSec));
  kdfFails = false;
  rngDead = true;
  TEST_ASSERT_EQUAL(Result::Internal, f.post("123456", 11 * kSec));
  rngDead = false;
  TEST_ASSERT_FALSE(f.core.status(0, 12 * kSec).pending);
  TEST_ASSERT_EQUAL(0, saves);
}

void test_claim_status_ids_are_not_guessable_handles(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.core.claimStatus(nullptr, 0, 11 * kSec).status);
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.core.claimStatus("", 0, 11 * kSec).status);
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.core.claimStatus("abc", 0, 11 * kSec).status);
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.core.claimStatus("00000000000000000000000000000000", 0, 11 * kSec).status);
  char other[kClaimIdLen + 1];
  memcpy(other, f.id, sizeof(other));
  other[0] = other[0] == 'a' ? 'b' : 'a';
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.core.claimStatus(other, 0, 11 * kSec).status);
  char longId[kClaimIdLen + 3];
  memcpy(longId, f.id, kClaimIdLen);
  longId[kClaimIdLen] = 'a';
  longId[kClaimIdLen + 1] = '\0';
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.core.claimStatus(longId, 0, 11 * kSec).status);
  TEST_ASSERT_EQUAL(ClaimStatus::Pending, f.poll(11 * kSec).status);
}

void test_outcomes_are_forgotten_after_five_minutes_and_the_table_is_bounded(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  f.post("654321", 11 * kSec);  // conflict: first id remembered
  char firstId[kClaimIdLen + 1];
  memcpy(firstId, f.id, sizeof(firstId));
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.core.claimStatus(firstId, 0, 11 * kSec + 301 * kSec).status);
  // 6 conflicts in a row never overflow the table
  for (int i = 0; i < 6; i++)
  {
    TEST_ASSERT_EQUAL(Result::Ok, f.post("123456", (400 + 10 * i) * kSec));
    TEST_ASSERT_EQUAL(Result::ClaimConflict, f.post("123456", (405 + 10 * i) * kSec));
  }
}

// ---- login, replay, expiry, session limit ----------------------------------------------------------------------
void test_login_right_and_wrong_pin(void)
{
  Fixture f = claimed();
  TEST_ASSERT_EQUAL(Result::BadPin, f.login("000000", 200 * kSec));
  TEST_ASSERT_EQUAL_UINT32(1, f.retry);
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", 202 * kSec));
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(f.tok, 203 * kSec));
  TEST_ASSERT_EQUAL(Result::BadPin, f.login("12345", 300 * kSec));  // format errors count as failures, no KDF run
}

void test_logout_then_replay_is_refused(void)
{
  Fixture f = claimed();
  char old[SessionTable::kTokenLen + 1];
  memcpy(old, f.tok, sizeof(old));
  TEST_ASSERT_EQUAL(Result::Ok, f.core.logout(old));
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.check(old, 101 * kSec));
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.logout(old));
}

void test_idle_expiry_30_minutes(void)
{
  Fixture f = claimed();  // created at 100 s
  const int64_t t0 = 100 * kSec;
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(f.tok, t0 + 29 * 60 * kSec));          // refreshes idle
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(f.tok, t0 + (29 + 29) * 60 * kSec));
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.check(f.tok, t0 + (29 + 29 + 30) * 60 * kSec));
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.check(f.tok, t0 + (29 + 29 + 31) * 60 * kSec));  // stays dead
}

void test_absolute_expiry_12_hours_despite_activity(void)
{
  Fixture f = claimed();
  const int64_t t0 = 100 * kSec;
  for (int64_t m = 10; m < 12 * 60; m += 10)
  {
    TEST_ASSERT_EQUAL(Result::Ok, f.core.check(f.tok, t0 + m * 60 * kSec));
  }
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.check(f.tok, t0 + 12 * 3600 * kSec));
}

void test_fifth_session_replaces_the_idlest_one(void)
{
  Fixture f = claimed();
  char toks[5][SessionTable::kTokenLen + 1];
  memcpy(toks[0], f.tok, sizeof(toks[0]));
  int64_t t = 200 * kSec;
  for (int i = 1; i < 4; i++)
  {
    TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", t));
    memcpy(toks[i], f.tok, sizeof(toks[i]));
    t += 10 * kSec;
  }
  // use session 0 so session 1 is now the idlest
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(toks[0], t));
  t += kSec;
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", t));  // the 5th
  memcpy(toks[4], f.tok, sizeof(toks[4]));
  t += kSec;
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.check(toks[1], t));
  for (int i : {0, 2, 3, 4}) TEST_ASSERT_EQUAL(Result::Ok, f.core.check(toks[i], t));
}

// ---- brute force -----------------------------------------------------------------------------------------------
void test_hammering_while_blocked_costs_no_key_derivation(void)
{
  Fixture f = claimed();
  const int before = kdfCalls;
  TEST_ASSERT_EQUAL(Result::BadPin, f.login("000000", 200 * kSec));
  TEST_ASSERT_EQUAL(before + 1, kdfCalls);
  for (int i = 0; i < 1000; i++)
  {
    TEST_ASSERT_EQUAL(Result::Backoff, f.login("123456", 200 * kSec + i));  // even the right PIN is refused
  }
  TEST_ASSERT_EQUAL(before + 1, kdfCalls);
  TEST_ASSERT_EQUAL_UINT32(1, f.retry);
}

void test_backoff_grows_and_is_capped_over_a_brute_force_run(void)
{
  Fixture f = claimed();
  int64_t now = 1000 * kSec;
  uint32_t expect = 1;
  for (int i = 0; i < 14; i++)
  {
    TEST_ASSERT_EQUAL(Result::BadPin, f.login("999999", now));
    TEST_ASSERT_EQUAL_UINT32(expect, f.retry);
    const int64_t wait = static_cast<int64_t>(f.retry) * kSec;
    // one millisecond early: still blocked
    TEST_ASSERT_EQUAL(Result::Backoff, f.login("999999", now + wait - 1000));
    now += wait;
    expect = expect >= 512 ? 900 : expect * 2;
  }
  TEST_ASSERT_EQUAL_UINT32(900, expect);
  // after the wait the right PIN works and clears the counter
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", now));
  TEST_ASSERT_EQUAL(Result::BadPin, f.login("999999", now + kSec));
  TEST_ASSERT_EQUAL_UINT32(1, f.retry);
}

void test_backoff_blocks_login_but_not_existing_sessions(void)
{
  Fixture f = claimed();
  char keep[SessionTable::kTokenLen + 1];
  memcpy(keep, f.tok, sizeof(keep));
  for (int i = 0; i < 5; i++) f.login("000000", (200 + i * 1000) * kSec);
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(keep, 201 * kSec));
}

void test_key_derivation_never_runs_under_the_lock(void)
{
  Fixture f;
  f.claim("123456", 10 * kSec, 11 * kSec);
  f.login("123456", 100 * kSec);
  uint32_t r;
  f.core.changePin("123456", 6, "654321", 6, 200 * kSec, &r);
  f.login("654321", 300 * kSec);
  TEST_ASSERT_FALSE(kdfSawLock);
  TEST_ASSERT_EQUAL(0, lockDepth);
}

// ---- BOOT press ends the login wait ----------------------------------------------------------------------------
void test_new_boot_press_clears_the_backoff(void)
{
  Fixture f = claimed();
  for (int i = 0; i < 4; i++) f.core.login("000000", 6, (200 + i) * kSec, f.tok, &f.retry, 0);
  TEST_ASSERT_EQUAL(Result::Backoff, f.core.login("123456", 6, 204 * kSec, f.tok, &f.retry, 0));
  // a press nobody has seen yet: the right PIN works at once
  TEST_ASSERT_EQUAL(Result::Ok, f.core.login("123456", 6, 205 * kSec, f.tok, &f.retry, 204 * kSec));
}

void test_the_same_press_does_not_clear_it_twice(void)
{
  Fixture f = claimed();
  TEST_ASSERT_EQUAL(Result::BadPin, f.core.login("000000", 6, 200 * kSec, f.tok, &f.retry, 190 * kSec));
  TEST_ASSERT_EQUAL(Result::Backoff, f.core.login("123456", 6, 200 * kSec, f.tok, &f.retry, 190 * kSec));  // press 190 already seen
  TEST_ASSERT_EQUAL(Result::Ok, f.core.login("123456", 6, 200 * kSec, f.tok, &f.retry, 199 * kSec));      // a newer one
}

void test_press_clears_backoff_for_change_pin_and_status(void)
{
  Fixture f = claimed();
  f.core.login("000000", 6, 200 * kSec, f.tok, &f.retry, 0);
  AuthCore::Status s = f.core.status(0, 200 * kSec);
  TEST_ASSERT_EQUAL_UINT32(1, s.retryAfterS);
  s = f.core.status(200 * kSec, 200 * kSec);
  TEST_ASSERT_EQUAL_UINT32(0, s.retryAfterS);
  f.core.login("000000", 6, 201 * kSec, f.tok, &f.retry, 200 * kSec);
  uint32_t r = 0;
  TEST_ASSERT_EQUAL(Result::Ok, f.core.changePin("123456", 6, "654321", 6, 202 * kSec, &r, 201 * kSec));
}

// ---- change PIN ------------------------------------------------------------------------------------------------
void test_change_pin_ends_all_sessions_and_switches_pin(void)
{
  Fixture f = claimed();
  char other[SessionTable::kTokenLen + 1];
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", 200 * kSec));
  memcpy(other, f.tok, sizeof(other));
  uint32_t r = 0;
  TEST_ASSERT_EQUAL(Result::Ok, f.core.changePin("123456", 6, "654321", 6, 300 * kSec, &r));
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.check(f.tok, 301 * kSec));
  TEST_ASSERT_EQUAL(Result::Unauthorized, f.core.check(other, 301 * kSec));
  TEST_ASSERT_EQUAL(Result::BadPin, f.login("123456", 302 * kSec));
  TEST_ASSERT_EQUAL(Result::Ok, f.login("654321", 304 * kSec));
}

void test_change_pin_wrong_old_pin_counts_and_changes_nothing(void)
{
  Fixture f = claimed();
  uint32_t r = 0;
  const int savesBefore = saves;
  TEST_ASSERT_EQUAL(Result::BadPin, f.core.changePin("000000", 6, "654321", 6, 300 * kSec, &r));
  TEST_ASSERT_EQUAL_UINT32(1, r);
  TEST_ASSERT_EQUAL(Result::Backoff, f.core.changePin("123456", 6, "654321", 6, 300 * kSec, &r));
  TEST_ASSERT_EQUAL(savesBefore, saves);
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(f.tok, 300 * kSec));  // sessions untouched
}

void test_change_pin_rejects_bad_new_pin_without_spending_an_attempt(void)
{
  Fixture f = claimed();
  uint32_t r = 0;
  TEST_ASSERT_EQUAL(Result::BadFormat, f.core.changePin("123456", 6, "12", 2, 300 * kSec, &r));
  TEST_ASSERT_EQUAL(Result::BadFormat, f.core.changePin("000000", 6, "abcdef", 6, 300 * kSec, &r));
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", 301 * kSec));  // no backoff was started
}

void test_change_pin_storage_failure_keeps_the_old_pin(void)
{
  Fixture f = claimed();
  saveFails = true;
  uint32_t r = 0;
  TEST_ASSERT_EQUAL(Result::Internal, f.core.changePin("123456", 6, "654321", 6, 300 * kSec, &r));
  saveFails = false;
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(f.tok, 301 * kSec));
  TEST_ASSERT_EQUAL(Result::Ok, f.login("123456", 302 * kSec));
}

// ---- recovery and unavailable ----------------------------------------------------------------------------------
void test_recovery_returns_to_unclaimed_and_allows_a_new_claim(void)
{
  Fixture f = claimed();
  for (int i = 0; i < 3; i++) f.login("000000", (200 + i * 100) * kSec);
  TEST_ASSERT_EQUAL(Result::Ok, f.core.recover());
  TEST_ASSERT_EQUAL(State::Unclaimed, f.core.state());
  TEST_ASSERT_FALSE(haveStored);
  TEST_ASSERT_FALSE(f.core.status(0, 590 * kSec).pending);
  TEST_ASSERT_EQUAL(Result::NotClaimed, f.core.check(f.tok, 600 * kSec));  // old token is dead
  TEST_ASSERT_EQUAL(Result::Ok, f.claim("246810", 590 * kSec, 600 * kSec));
  TEST_ASSERT_EQUAL(Result::Ok, f.login("246810", 601 * kSec));           // backoff from before was cleared
  TEST_ASSERT_EQUAL(Result::BadPin, f.login("123456", 700 * kSec));       // old PIN is gone
}

void test_recovery_storage_failure_keeps_the_device_claimed(void)
{
  Fixture f = claimed();
  eraseFails = true;
  TEST_ASSERT_EQUAL(Result::Internal, f.core.recover());
  TEST_ASSERT_EQUAL(State::Claimed, f.core.state());
  TEST_ASSERT_EQUAL(Result::Ok, f.core.check(f.tok, 101 * kSec));
}

void test_unavailable_storage_fails_closed_until_recovery(void)
{
  reset();
  AuthCore core(kPlat);
  core.load(State::Unavailable, nullptr);
  char tok[SessionTable::kTokenLen + 1];
  uint32_t r;
  TEST_ASSERT_EQUAL(Result::Unavailable, core.check("x", 0));
  char id[kClaimIdLen + 1];
  TEST_ASSERT_EQUAL(Result::Unavailable, core.claimPost("123456", 6, 10 * kSec, 11 * kSec, id));
  TEST_ASSERT_EQUAL(Result::Unavailable, core.login("123456", 6, 0, tok, &r));
  TEST_ASSERT_EQUAL(0, kdfCalls);
  TEST_ASSERT_EQUAL(Result::Ok, core.recover());  // a corrupt record is cleared by the hold gesture
  TEST_ASSERT_EQUAL(State::Unclaimed, core.state());
}

void test_claimed_without_a_record_is_unavailable(void)
{
  reset();
  AuthCore core(kPlat);
  core.load(State::Claimed, nullptr);
  TEST_ASSERT_EQUAL(State::Unavailable, core.state());
}

void test_status_reports_pending_and_backoff(void)
{
  Fixture f;
  AuthCore::Status s = f.core.status(0, 25 * kSec);
  TEST_ASSERT_EQUAL(State::Unclaimed, s.state);
  TEST_ASSERT_FALSE(s.pending);
  f.claim("123456", 10 * kSec, 25 * kSec);
  f.login("000000", 30 * kSec);
  s = f.core.status(0, 30 * kSec);
  TEST_ASSERT_EQUAL(State::Claimed, s.state);
  TEST_ASSERT_FALSE(s.pending);
  TEST_ASSERT_EQUAL_UINT32(1, s.retryAfterS);
}

void test_recovery_during_a_pending_claim_cancels_it(void)
{
  Fixture f;
  f.post("123456", 10 * kSec);
  TEST_ASSERT_EQUAL(Result::Ok, f.core.recover());
  TEST_ASSERT_EQUAL(ClaimStatus::Unknown, f.poll(11 * kSec, 12 * kSec).status);
  TEST_ASSERT_EQUAL(State::Unclaimed, f.core.state());
}

// ---- HTTP mapping and Authorization parsing -------------------------------------------------------------------
void test_http_mapping_covers_every_result(void)
{
  const Result all[] = {Result::Ok,           Result::Unavailable, Result::NotClaimed, Result::AlreadyClaimed,
                        Result::ButtonRequired, Result::BadFormat, Result::BadPin,     Result::Backoff,
                        Result::Unauthorized, Result::Internal};
  for (Result r : all)
  {
    const Http h = httpFor(r);
    TEST_ASSERT_TRUE(h.status >= 200 && h.status < 600);
    TEST_ASSERT_TRUE(h.code != nullptr && strlen(h.code) > 0 && strlen(h.code) < 40);
    TEST_ASSERT_TRUE(h.reason != nullptr && strlen(h.reason) > 0 && strlen(h.reason) < 100);
    // a reason or code must never carry a PIN or token; they are fixed strings
    TEST_ASSERT_NULL(strstr(h.reason, "%"));
  }
  TEST_ASSERT_EQUAL_INT(401, httpFor(Result::BadPin).status);
  TEST_ASSERT_EQUAL_INT(401, httpFor(Result::Unauthorized).status);
  TEST_ASSERT_EQUAL_INT(429, httpFor(Result::Backoff).status);
  TEST_ASSERT_EQUAL_INT(503, httpFor(Result::Unavailable).status);
  TEST_ASSERT_EQUAL_INT(403, httpFor(Result::ButtonRequired).status);
  TEST_ASSERT_EQUAL_INT(409, httpFor(Result::AlreadyClaimed).status);
  TEST_ASSERT_EQUAL_INT(422, httpFor(Result::BadFormat).status);
}

void test_bearer_parsing(void)
{
  char out[kAuthHeaderMax];
  TEST_ASSERT_TRUE(bearerToken("Bearer abc123", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("abc123", out);
  TEST_ASSERT_TRUE(bearerToken("bearer abc", out, sizeof(out)));
  TEST_ASSERT_TRUE(bearerToken("BEARER abc", out, sizeof(out)));
  const char* bad[] = {"", "Bearer", "Bearer ", "Bearer  abc", "Bearer abc def", "Basic abc", "Bearerabc", "Bearer a\tb",
                       "Bearer a\x7f", "Bearer \xc3\xa9", " Bearer abc", "Token abc"};
  for (const char* h : bad) TEST_ASSERT_FALSE_MESSAGE(bearerToken(h, out, sizeof(out)), h);
  TEST_ASSERT_FALSE(bearerToken(nullptr, out, sizeof(out)));
  TEST_ASSERT_FALSE(bearerToken("Bearer abc", nullptr, 8));
  TEST_ASSERT_FALSE(bearerToken("Bearer abc", out, 0));
  char tiny[4];
  TEST_ASSERT_FALSE(bearerToken("Bearer abcd", tiny, sizeof(tiny)));  // no room for the NUL
  TEST_ASSERT_TRUE(bearerToken("Bearer abc", tiny, sizeof(tiny)));
}
}  // namespace

extern "C" void run_auth_core_tests(void)
{
  RUN_TEST(test_http_mapping_covers_every_result);
  RUN_TEST(test_bearer_parsing);
  RUN_TEST(test_pin_policy);
  RUN_TEST(test_constant_time_eq);
  RUN_TEST(test_record_round_trip_and_corruption);
  RUN_TEST(test_backoff_schedule);
  RUN_TEST(test_session_token_shape_and_rng_failure);
  RUN_TEST(test_session_lookup_rejects_malformed_tokens);
  RUN_TEST(test_unclaimed_device_refuses_everything_but_claim);
  RUN_TEST(test_post_parks_the_pin_and_changes_nothing_else);
  RUN_TEST(test_press_after_the_post_commits_and_hands_over_the_token_once);
  RUN_TEST(test_a_press_before_the_post_does_not_count);
  RUN_TEST(test_window_is_60_seconds_from_the_post);
  RUN_TEST(test_second_post_cancels_both);
  RUN_TEST(test_bad_format_post_leaves_a_pending_claim_alone);
  RUN_TEST(test_post_after_the_press_but_before_anyone_polled_loses_to_the_first);
  RUN_TEST(test_post_on_claimed_or_unavailable_device);
  RUN_TEST(test_commit_storage_failure_reports_failed_and_changes_nothing);
  RUN_TEST(test_commit_with_a_dead_rng_rolls_the_record_back);
  RUN_TEST(test_kdf_or_rng_failure_at_post_parks_nothing);
  RUN_TEST(test_claim_status_ids_are_not_guessable_handles);
  RUN_TEST(test_outcomes_are_forgotten_after_five_minutes_and_the_table_is_bounded);
  RUN_TEST(test_recovery_during_a_pending_claim_cancels_it);
  RUN_TEST(test_login_right_and_wrong_pin);
  RUN_TEST(test_logout_then_replay_is_refused);
  RUN_TEST(test_idle_expiry_30_minutes);
  RUN_TEST(test_absolute_expiry_12_hours_despite_activity);
  RUN_TEST(test_fifth_session_replaces_the_idlest_one);
  RUN_TEST(test_hammering_while_blocked_costs_no_key_derivation);
  RUN_TEST(test_backoff_grows_and_is_capped_over_a_brute_force_run);
  RUN_TEST(test_backoff_blocks_login_but_not_existing_sessions);
  RUN_TEST(test_key_derivation_never_runs_under_the_lock);
  RUN_TEST(test_new_boot_press_clears_the_backoff);
  RUN_TEST(test_the_same_press_does_not_clear_it_twice);
  RUN_TEST(test_press_clears_backoff_for_change_pin_and_status);
  RUN_TEST(test_change_pin_ends_all_sessions_and_switches_pin);
  RUN_TEST(test_change_pin_wrong_old_pin_counts_and_changes_nothing);
  RUN_TEST(test_change_pin_rejects_bad_new_pin_without_spending_an_attempt);
  RUN_TEST(test_change_pin_storage_failure_keeps_the_old_pin);
  RUN_TEST(test_recovery_returns_to_unclaimed_and_allows_a_new_claim);
  RUN_TEST(test_recovery_storage_failure_keeps_the_device_claimed);
  RUN_TEST(test_unavailable_storage_fails_closed_until_recovery);
  RUN_TEST(test_claimed_without_a_record_is_unavailable);
  RUN_TEST(test_status_reports_pending_and_backoff);
}
