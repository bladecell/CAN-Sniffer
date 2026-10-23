"""Offline tests of the auth scenarios against an in-memory fake device. Never touches a network."""
import pytest

from auth_scenarios import (
    PinGuard,
    PinRestoreError,
    run_bad_new_format,
    run_idle_expiry,
    run_no_token,
    run_pin_roundtrip,
    run_session_replacement,
    run_wrong_old_pin,
)

PIN = "482915"


class FakeDevice:
    """Models login, backoff (min(2^(N-1), 900) s), 4-session cap, PIN change and the idle timer."""

    def __init__(self, pin=PIN, fail_put_back=False, drop_put_reply=False):
        self.pin, self.now, self.fails, self.blocked_until = pin, 0.0, 0, 0.0
        self.sessions: dict[str, float] = {}
        self.n = 0
        self.fail_put_back, self.drop_put_reply = fail_put_back, drop_put_reply
        self.puts = 0

    def sleep(self, s):
        self.now += s

    def _err(self, status, code, headers=None):
        return status, headers or {}, {"status": "error", "code": code}

    def _check_pin(self, pin):
        if self.now < self.blocked_until:
            return self._err(429, "LOGIN_BACKOFF", {"Retry-After": str(int(self.blocked_until - self.now) + 1)})
        if pin == self.pin:
            self.fails = 0
            return None
        self.fails += 1
        wait = min(2 ** (self.fails - 1), 900)
        self.blocked_until = self.now + wait
        return self._err(401, "BAD_PIN", {"Retry-After": str(wait)})

    def _live(self, token):
        t = self.sessions.get(token)
        if t is None or self.now - t > 1800:
            self.sessions.pop(token, None)
            return False
        self.sessions[token] = self.now
        return True

    def __call__(self, method, path, body=None, token=None):
        if path == "/api/v1/auth/login":
            bad = self._check_pin(body["pin"])
            if bad:
                return bad
            if len(self.sessions) >= 4:
                del self.sessions[min(self.sessions, key=self.sessions.get)]
            self.n += 1
            tok = f"{self.n:032x}"
            self.sessions[tok] = self.now
            return 200, {}, {"status": "ok", "token": tok}
        if path == "/api/v1/auth" and method == "GET":
            return 200, {}, {"state": "claimed", "authenticated": bool(token) and self._live(token)}
        if not (token and self._live(token)):
            return self._err(401, "UNAUTHORIZED")
        if path == "/api/v1/auth/logout":
            del self.sessions[token]
            return 204, {}, None
        if path == "/api/v1/auth/pin":
            new = body.get("new_pin")
            if not (isinstance(new, str) and len(new) == 6 and new.isdigit()):
                return self._err(422, "BAD_PIN_FORMAT")
            bad = self._check_pin(body.get("old_pin"))
            if bad:
                return bad
            self.puts += 1
            if self.fail_put_back and self.puts > 1:
                return self._err(500, "INTERNAL")
            self.pin = new
            self.sessions.clear()
            if self.drop_put_reply and self.puts == 1:
                raise ConnectionError("reply lost")
            return 204, {}, None
        raise AssertionError(path)


def test_round_trip_restores_pin():
    dev = FakeDevice()
    run_pin_roundtrip(PinGuard(dev, PIN, dev.sleep), dev.sleep)
    assert dev.pin == PIN and not dev.sessions


def test_restore_runs_when_an_assertion_fails_midway():
    class OldPinStillWorks(FakeDevice):
        def _check_pin(self, pin):
            return None if pin in (self.pin, PIN) else super()._check_pin(pin)

    dev = OldPinStillWorks()
    with pytest.raises(AssertionError):
        run_pin_roundtrip(PinGuard(dev, PIN, dev.sleep), dev.sleep)
    assert dev.pin == PIN and not dev.sessions


def test_restore_after_lost_put_reply():
    dev = FakeDevice(drop_put_reply=True)
    guard = PinGuard(dev, PIN, dev.sleep)
    with pytest.raises(ConnectionError):
        run_pin_roundtrip(guard, dev.sleep)
    assert guard.state == "orig" and dev.pin == PIN  # the finally-restore found the changed PIN and undid it


def test_restore_failure_is_reported_without_pin():
    dev = FakeDevice(fail_put_back=True)
    guard = PinGuard(dev, PIN, dev.sleep)
    with pytest.raises(PinRestoreError) as exc:
        run_pin_roundtrip(guard, dev.sleep)
    assert PIN not in str(exc.value) and guard.new not in str(exc.value)


def test_restore_waits_out_backoff():
    dev = FakeDevice()
    guard = PinGuard(dev, PIN, dev.sleep)
    for _ in range(5):
        dev("POST", "/api/v1/auth/login", {"pin": "111111"})
    assert dev.blocked_until > dev.now
    guard.restore()
    assert dev.pin == PIN


def test_wrong_old_pin_keeps_pin():
    dev = FakeDevice()
    run_wrong_old_pin(dev, PIN, dev.sleep)
    assert dev.pin == PIN


def test_bad_format_and_no_token_keep_pin():
    dev = FakeDevice()
    run_bad_new_format(dev, PIN, dev.sleep)
    run_no_token(dev, PIN, dev.sleep)
    assert dev.pin == PIN and dev.puts == 0


def test_session_replacement_and_cleanup():
    dev = FakeDevice()
    tokens: list[str] = []
    run_session_replacement(dev, PIN, tokens, dev.sleep)
    assert len(tokens) == 5 and len([t for t in tokens if t in dev.sessions]) == 4


def test_session_replacement_with_foreign_session():
    dev = FakeDevice()
    dev("POST", "/api/v1/auth/login", {"pin": PIN})
    run_session_replacement(dev, PIN, [], dev.sleep)


def test_session_replacement_detects_no_eviction():
    dev = FakeDevice()
    class NoCap(FakeDevice):
        def __call__(self, method, path, body=None, token=None):
            if path == "/api/v1/auth/login":
                self.n += 1
                tok = f"{self.n:032x}"
                self.sessions[tok] = self.now
                return 200, {}, {"token": tok}
            return super().__call__(method, path, body, token)

    with pytest.raises(AssertionError):
        run_session_replacement(NoCap(), PIN, [], dev.sleep)


def test_idle_expiry():
    dev = FakeDevice()
    run_idle_expiry(dev, PIN, dev.sleep)
