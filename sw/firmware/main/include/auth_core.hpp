#pragma once

// Access-control rules (P2.10c, docs/decisions.md 2026-10-18): claim, login, change PIN, recovery and the one admin check.
// No ESP-IDF includes. Everything the device provides (key derivation, random bytes, NVS, a short lock) comes in through
// `Platform`, so the host tests drive every rule, including the hostile cases, with fakes.
//
// Locking: `lock`/`unlock` guard only the fields below (state, record, sessions, backoff, claim gate) and are held for
// microseconds. The key derivation (about 100+ ms) always runs with the lock released, so the HTTP task's `check()` is
// never held up behind a login. Claim, login, changePin and recover must be serialised by the caller (one operation
// mutex); `check` and `logout` may run at any time.

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
  ButtonRequired,  // 403: no BOOT press in the last 60 s
  BadFormat,       // 422: PIN is not six digits
  BadPin,          // 401
  Backoff,         // 429
  Unauthorized,    // 401: no or unknown/expired token
  Internal,        // 500: RNG, key derivation or storage failed; nothing changed
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
    backoff_.reset();
    gate_ = ClaimGate();
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

  // First PIN. Needs a BOOT press at most 60 s old that no earlier claim used. On success the caller is logged in.
  Result claim(const char* pin, size_t len, int64_t lastPressUs, int64_t nowUs, char* tokenOut)
  {
    p_.lock();
    const State st   = state_;
    const bool  open = gate_.open(lastPressUs, nowUs);
    p_.unlock();
    if (st == State::Unavailable)
    {
      return Result::Unavailable;
    }
    if (st == State::Claimed)
    {
      return Result::AlreadyClaimed;
    }
    if (!open)
    {
      return Result::ButtonRequired;
    }
    if (!isValidPin(pin, len))
    {
      return Result::BadFormat;
    }
    PinRecord rec;
    if (!makeRecord(pin, len, &rec) || !persist(rec))
    {
      return Result::Internal;
    }
    p_.lock();
    Result r = Result::Internal;
    if (state_ == State::Unclaimed && sessions_.create(ms(nowUs), p_.rng, tokenOut))
    {
      rec_   = rec;
      state_ = State::Claimed;
      gate_.consume(lastPressUs);
      backoff_.reset();
      r = Result::Ok;
    }
    p_.unlock();
    if (r != Result::Ok)
    {
      (void)p_.erase();  // do not leave a record behind that the state does not know
    }
    return r;
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
      backoff_.onSuccess();
      r = Result::Ok;
    }
    else if (good)
    {
      r = Result::Internal;
    }
    else
    {
      backoff_.onFailure(ms(nowUs));
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
    backoff_.onSuccess();
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
    backoff_.reset();
    p_.unlock();
    return Result::Ok;
  }

  struct Status
  {
    State    state;
    uint32_t retryAfterS;
    uint32_t claimWindowS;
  };

  Status status(int64_t lastPressUs, int64_t nowUs)
  {
    p_.lock();
    notePress(lastPressUs);
    Status s{state_, backoff_.retryAfterS(ms(nowUs)),
             state_ == State::Unclaimed ? gate_.remainingS(lastPressUs, nowUs) : 0};
    p_.unlock();
    return s;
  }

private:
  // Lock held. A BOOT press the core has not seen before ends the login wait: someone is at the device.
  void notePress(int64_t lastPressUs)
  {
    if (lastPressUs > 0 && lastPressUs != seenPressUs_)
    {
      seenPressUs_ = lastPressUs;
      backoff_.reset();
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
  ClaimGate    gate_;
  int64_t      seenPressUs_ = 0;
};
}  // namespace auth
