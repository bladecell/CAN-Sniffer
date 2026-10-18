#pragma once

// Access-control rules (P2.10c/d, docs/decisions.md 2026-10-18): two-step claim, login, change PIN, recovery and the one
// admin check.
// No ESP-IDF includes. Everything the device provides (key derivation, random bytes, NVS, a short lock) comes in through
// `Platform`, so the host tests drive every rule, including the hostile cases, with fakes.
//
// Locking: `lock`/`unlock` guard only the fields below (state, record, sessions, backoff, claim gate) and are held for
// microseconds. The key derivation (about 100+ ms) always runs with the lock released, so the HTTP task's `check()` is
// never held up behind a login. claimPost, claimStatus, status (when unclaimed), login, changePin and recover must be
// serialised by the caller (one operation mutex); `check` and `logout` may run at any time.
//
// Claim, in two steps so that a bystander on the open AP cannot take the device by pressing BOOT first:
//   1. claimPost(pin) derives the PIN hash and parks it in the one pending slot, stamped with the time of the post.
//   2. A BOOT press after that stamp and within 60 s of it commits the parked PIN (settle(), run by every call that
//      looks at the claim). The poster learns the outcome with claimStatus(id) and gets the session token there.
// A second claimPost while one is pending refuses both: the slot is cleared and the first id reports Conflict.

#include <cstddef>
#include <cstdint>

#include "auth_backoff.hpp"
#include "auth_claim.hpp"
#include "auth_pin.hpp"
#include "auth_sessions.hpp"

namespace auth
{
enum class State : uint8_t
{
  Unavailable,  // NVS unreadable or the record is corrupt: gated routes fail closed until recovery
  Unclaimed,    // no PIN yet
  Claimed,
};

enum class Result : uint8_t
{
  Ok,
  Unavailable,     // 503
  NotClaimed,      // 403: nothing to log in to yet
  AlreadyClaimed,  // 409
  ButtonRequired,  // 403: the BOOT button driver is not running, so a claim could never be confirmed
  ClaimConflict,   // 409: another claim was pending; both are cancelled
  BadFormat,       // 422: PIN is not six digits
  BadPin,          // 401
  Backoff,         // 429
  Unauthorized,    // 401: no or unknown/expired token
  Internal,        // 500: RNG, key derivation or storage failed; nothing changed
};

enum class ClaimStatus : uint8_t
{
  Pending,    // waiting for a BOOT press
  Committed,  // PIN set; the token is delivered once
  Expired,    // no press within 60 s of the post
  Conflict,   // a second claim arrived; both cancelled
  Failed,     // the press came but the record could not be stored; nothing changed
  Unknown,    // no such id (never issued, malformed or forgotten)
};

constexpr size_t kClaimIdLen = 32;  // hex characters

struct ClaimView
{
  ClaimStatus status;
  uint32_t    expiresInS;                          // Pending only
  bool        hasToken;                            // Committed, first time only
  char        token[SessionTable::kTokenLen + 1];
};

struct Platform
{
  Kdf               kdf;
  SessionTable::Rng rng;
  bool (*save)(const uint8_t* record, size_t len);  // persists the record as one blob
  bool (*erase)();                                  // removes it (back to unclaimed)
  void (*lock)();
  void (*unlock)();
  uint32_t iterations;  // PBKDF2 cost for new PINs
  int64_t (*nowUs)();   // esp_timer microseconds; stamps a post after the key derivation, which takes 100+ ms
  uint32_t (*loadFailures)();       // failed-login count kept across a crash or sleep (RTC memory); 0 when none
  void (*storeFailures)(uint32_t);  // called with the new count whenever it changes
};

class AuthCore
{
public:
  explicit AuthCore(const Platform& p) : p_(p) {}

  // Sets the state read from storage. `rec` is used only for Claimed. Also forgets sessions, backoff and the used press.
  void load(State s, const PinRecord* rec)
  {
    p_.lock();
    state_ = (s == State::Claimed && rec == nullptr) ? State::Unavailable : s;
    if (state_ == State::Claimed)
    {
      rec_ = *rec;
    }
    sessions_.clear();
    backoff_.restore(p_.loadFailures(), ms(p_.nowUs()));
    clearClaims();
    seenPressUs_ = 0;
    p_.unlock();
  }

  State state()
  {
    p_.lock();
    const State s = state_;
    p_.unlock();
    return s;
  }

  // The one admin check, called with the Bearer token (or null) before a gated handler runs.
  Result check(const char* token, int64_t nowUs)
  {
    p_.lock();
    Result r = Result::Unauthorized;
    if (state_ == State::Unavailable)
    {
      r = Result::Unavailable;
    }
    else if (state_ == State::Unclaimed)
    {
      r = Result::NotClaimed;
    }
    else if (sessions_.validate(token, ms(nowUs)))
    {
      r = Result::Ok;
    }
    p_.unlock();
    return r;
  }

  Result logout(const char* token)
  {
    p_.lock();
    const bool was = sessions_.revoke(token);
    p_.unlock();
    return was ? Result::Ok : Result::Unauthorized;
  }

  // Step 1 of the claim. Parks the PIN and returns the id the client polls with. Cheap refusals come first: no key
  // derivation for a claimed device, a bad PIN or a conflict.
  Result claimPost(const char* pin, size_t len, int64_t lastPressUs, int64_t nowUs, char* idOut)
  {
    settle(lastPressUs, nowUs);
    p_.lock();
    const State st = state_;
    if (st == State::Unclaimed && !isValidPin(pin, len))
    {
      p_.unlock();
      return Result::BadFormat;  // does not disturb a pending claim
    }
    if (st == State::Unclaimed && pending_.used)
    {
      remember(pending_.id, ClaimStatus::Conflict, nowUs);
      pending_.used = false;
      p_.unlock();
      return Result::ClaimConflict;
    }
    p_.unlock();
    if (st == State::Unavailable)
    {
      return Result::Unavailable;
    }
    if (st == State::Claimed)
    {
      return Result::AlreadyClaimed;
    }
    Pending next;
    uint8_t raw[kClaimIdLen / 2];
    p_.rng(raw, sizeof(raw));
    if (!anyNonZero(raw, sizeof(raw)) || !makeRecord(pin, len, &next.rec))
    {
      return Result::Internal;
    }
    hexEncode(raw, sizeof(raw), next.id);
    next.postedUs = p_.nowUs();  // after the derivation: a press during it does not count
    Result r      = Result::Internal;
    p_.lock();
    if (state_ == State::Unclaimed && !pending_.used)
    {
      pending_      = next;
      pending_.used = true;
      memcpy(idOut, next.id, sizeof(next.id));
      r = Result::Ok;
    }
    p_.unlock();
    return r;
  }

  // Step 2: what became of the claim with this id. Commits it first if the press came.
  ClaimView claimStatus(const char* id, int64_t lastPressUs, int64_t nowUs)
  {
    settle(lastPressUs, nowUs);
    ClaimView v{ClaimStatus::Unknown, 0, false, {}};
    if (id == nullptr || strlen(id) != kClaimIdLen)
    {
      return v;
    }
    p_.lock();
    purgeOutcomes(nowUs);
    if (pending_.used && idEq(pending_.id, id))
    {
      v.status     = ClaimStatus::Pending;
      v.expiresInS = pendingLeftS(nowUs);
    }
    else
    {
      for (Outcome& o : out_)
      {
        if (o.used && idEq(o.id, id))
        {
          v.status = o.status;
          if (o.tokenReady)
          {
            memcpy(v.token, o.token, sizeof(v.token));
            v.hasToken  = true;
            o.tokenReady = false;
            memset(o.token, 0, sizeof(o.token));
          }
        }
      }
    }
    p_.unlock();
    return v;
  }

  // `retryS` is set for Backoff (time left) and BadPin (the wait this failure caused).
  // `lastPressUs`: the newest BOOT press; one the core has not seen yet clears the backoff (the owner is at the device).
  Result login(const char* pin, size_t len, int64_t nowUs, char* tokenOut, uint32_t* retryS, int64_t lastPressUs = 0)
  {
    PinRecord rec;
    Result    r = gateLogin(nowUs, lastPressUs, &rec, retryS);
    if (r != Result::Ok)
    {
      return r;
    }
    const bool good = verifyPin(pin, len, rec, p_.kdf);  // lock released: this is the slow part
    p_.lock();
    if (good && sessions_.create(ms(nowUs), p_.rng, tokenOut))
    {
      resetBackoff();
      r = Result::Ok;
    }
    else if (good)
    {
      r = Result::Internal;
    }
    else
    {
      backoff_.onFailure(ms(nowUs));
      p_.storeFailures(backoff_.failures());
      *retryS = backoff_.retryAfterS(ms(nowUs));
      r       = Result::BadPin;
    }
    p_.unlock();
    return r;
  }

  // Needs the old PIN (a wrong one counts like a failed login). Ends every session, the caller's included.
  Result changePin(const char* oldPin, size_t oldLen, const char* newPin, size_t newLen, int64_t nowUs, uint32_t* retryS,
                   int64_t lastPressUs = 0)
  {
    if (!isValidPin(newPin, newLen))
    {
      return Result::BadFormat;
    }
    PinRecord cur;
    Result    r = gateLogin(nowUs, lastPressUs, &cur, retryS);
    if (r != Result::Ok)
    {
      return r;
    }
    const bool good = verifyPin(oldPin, oldLen, cur, p_.kdf);
    if (!good)
    {
      p_.lock();
      backoff_.onFailure(ms(nowUs));
      p_.storeFailures(backoff_.failures());
      *retryS = backoff_.retryAfterS(ms(nowUs));
      p_.unlock();
      return Result::BadPin;
    }
    PinRecord next;
    if (!makeRecord(newPin, newLen, &next) || !persist(next))
    {
      return Result::Internal;
    }
    p_.lock();
    rec_ = next;
    sessions_.clear();
    resetBackoff();
    p_.unlock();
    return Result::Ok;
  }

  // Hold-BOOT recovery: forget the PIN. Works from any state, including Unavailable (a corrupt record).
  Result recover()
  {
    if (!p_.erase())
    {
      return Result::Internal;
    }
    p_.lock();
    state_ = State::Unclaimed;
    sessions_.clear();
    resetBackoff();
    clearClaims();
    p_.unlock();
    return Result::Ok;
  }

  struct Status
  {
    State    state;
    uint32_t retryAfterS;
    bool     pending;          // a claim waits for the BOOT press
    uint32_t pendingExpiresS;  // seconds left for it
  };

  // Also settles a pending claim, so it must be serialised like claimStatus.
  Status status(int64_t lastPressUs, int64_t nowUs)
  {
    settle(lastPressUs, nowUs);
    p_.lock();
    notePress(lastPressUs);
    Status s{state_, backoff_.retryAfterS(ms(nowUs)), pending_.used, pending_.used ? pendingLeftS(nowUs) : 0};
    p_.unlock();
    return s;
  }

private:
  struct Pending
  {
    bool      used = false;
    char      id[kClaimIdLen + 1] = {};
    PinRecord rec{};
    int64_t   postedUs = 0;
  };

  struct Outcome
  {
    bool        used = false;
    char        id[kClaimIdLen + 1] = {};
    ClaimStatus status = ClaimStatus::Unknown;
    char        token[SessionTable::kTokenLen + 1] = {};
    bool        tokenReady = false;
    int64_t     atUs = 0;
  };

  static constexpr size_t  kOutcomes      = 4;
  static constexpr int64_t kOutcomeKeepUs = 5LL * 60 * 1000000;

  static bool anyNonZero(const uint8_t* b, size_t n)
  {
    uint8_t any = 0;
    for (size_t i = 0; i < n; i++)
    {
      any = static_cast<uint8_t>(any | b[i]);
    }
    return any != 0;
  }

  static bool idEq(const char* a, const char* b)
  {
    return constantTimeEq(reinterpret_cast<const uint8_t*>(a), reinterpret_cast<const uint8_t*>(b), kClaimIdLen);
  }

  // Lock held.
  uint32_t pendingLeftS(int64_t nowUs) const
  {
    const int64_t left = pending_.postedUs + kClaimWindowUs - nowUs;
    return left <= 0 ? 0 : static_cast<uint32_t>((left + 999999) / 1000000);
  }

  // Lock held. Keeps the newest kOutcomes results; the oldest is overwritten.
  Outcome* remember(const char* id, ClaimStatus st, int64_t nowUs)
  {
    Outcome* o = &out_[outNext_];
    outNext_   = static_cast<uint8_t>((outNext_ + 1) % kOutcomes);
    *o         = Outcome();
    o->used    = true;
    memcpy(o->id, id, kClaimIdLen);
    o->status = st;
    o->atUs   = nowUs;
    return o;
  }

  // Lock held.
  void purgeOutcomes(int64_t nowUs)
  {
    for (Outcome& o : out_)
    {
      if (o.used && nowUs - o.atUs > kOutcomeKeepUs)
      {
        o = Outcome();
      }
    }
  }

  // Lock held.
  void clearClaims()
  {
    pending_ = Pending();
    for (Outcome& o : out_)
    {
      o = Outcome();
    }
    outNext_ = 0;
  }

  // Commits or expires the pending claim. The record is stored with the lock released.
  void settle(int64_t lastPressUs, int64_t nowUs)
  {
    p_.lock();
    if (!pending_.used || state_ != State::Unclaimed)
    {
      p_.unlock();
      return;
    }
    const bool pressed = lastPressUs > pending_.postedUs && lastPressUs - pending_.postedUs <= kClaimWindowUs;
    if (!pressed)
    {
      if (nowUs - pending_.postedUs > kClaimWindowUs)
      {
        remember(pending_.id, ClaimStatus::Expired, nowUs);
        pending_.used = false;
      }
      p_.unlock();
      return;
    }
    const Pending mine = pending_;
    p_.unlock();
    const bool stored = persist(mine.rec);
    p_.lock();
    if (pending_.used && idEq(pending_.id, mine.id) && state_ == State::Unclaimed)
    {
      pending_.used = false;
      Outcome* o    = nullptr;
      char     tok[SessionTable::kTokenLen + 1];
      if (stored && sessions_.create(ms(nowUs), p_.rng, tok))
      {
        rec_   = mine.rec;
        state_ = State::Claimed;
        resetBackoff();
        o             = remember(mine.id, ClaimStatus::Committed, nowUs);
        o->tokenReady = true;
        memcpy(o->token, tok, sizeof(tok));
      }
      else
      {
        remember(mine.id, ClaimStatus::Failed, nowUs);
      }
      p_.unlock();
      if (o == nullptr && stored)
      {
        (void)p_.erase();  // stored but not usable: do not leave a record the state does not know
      }
      return;
    }
    p_.unlock();
  }

  // Lock held.
  void resetBackoff()
  {
    backoff_.reset();
    p_.storeFailures(0);
  }

  // Lock held. A BOOT press the core has not seen before ends the login wait: someone is at the device.
  void notePress(int64_t lastPressUs)
  {
    if (lastPressUs > 0 && lastPressUs != seenPressUs_)
    {
      seenPressUs_ = lastPressUs;
      resetBackoff();
    }
  }

  static uint64_t ms(int64_t us) { return us > 0 ? static_cast<uint64_t>(us) / 1000 : 0; }

  // Common front of login and changePin: state, backoff, snapshot of the record.
  Result gateLogin(int64_t nowUs, int64_t lastPressUs, PinRecord* rec, uint32_t* retryS)
  {
    *retryS = 0;
    p_.lock();
    notePress(lastPressUs);
    Result r = Result::Ok;
    if (state_ == State::Unavailable)
    {
      r = Result::Unavailable;
    }
    else if (state_ == State::Unclaimed)
    {
      r = Result::NotClaimed;
    }
    else if (backoff_.retryAfterS(ms(nowUs)) != 0)
    {
      *retryS = backoff_.retryAfterS(ms(nowUs));
      r       = Result::Backoff;  // refused without a key derivation and not counted
    }
    else
    {
      *rec = rec_;
    }
    p_.unlock();
    return r;
  }

  bool makeRecord(const char* pin, size_t len, PinRecord* rec)
  {
    rec->iterations = p_.iterations;
    p_.rng(rec->salt, kSaltLen);
    return p_.kdf(reinterpret_cast<const uint8_t*>(pin), len, rec->salt, kSaltLen, rec->iterations, rec->hash);
  }

  bool persist(const PinRecord& rec)
  {
    uint8_t blob[kRecordSize];
    encodeRecord(rec, blob);
    return p_.save(blob, sizeof(blob));
  }

  Platform     p_;
  State        state_ = State::Unavailable;  // closed until load() says otherwise
  PinRecord    rec_{};
  SessionTable sessions_;
  LoginBackoff backoff_;
  Pending      pending_;
  Outcome      out_[kOutcomes];
  uint8_t      outNext_ = 0;
  int64_t      seenPressUs_ = 0;
};
}  // namespace auth
