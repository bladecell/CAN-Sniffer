"""Scenario logic for the mutating auth tests, written against an injected HTTP layer so a fake device can run it offline.

No PIN is ever put into an exception message, assertion message or log line.
"""
import secrets
import time
from collections.abc import Callable, Mapping

from auth_session import LOGIN_PATH, LOGOUT_PATH, AuthSkip, LoginError, logout

AUTH_PATH = "/api/v1/auth"
PIN_PATH = "/api/v1/auth/pin"
MAX_BACKOFF_S = 910.0  # the device caps the wait at 900 s

# call(method, path, body=None, token=None) -> (status, headers, parsed JSON body or None)
Call = Callable[..., tuple[int, Mapping[str, str], object]]


class PinRestoreError(Exception):
    """The original PIN could not be put back. Recovery is a 10 s BOOT hold on the device."""


def code_of(body: object) -> object:
    return body.get("code") if isinstance(body, dict) else None


def retry_after(headers: Mapping[str, str]) -> float:
    for key, value in headers.items():
        if key.lower() == "retry-after":
            try:
                return max(0.0, float(value))
            except ValueError:
                break
    return 1.0


def post_of(call: Call) -> Callable[..., tuple[int, Mapping[str, str], object]]:
    return lambda path, body=None, token=None: call("POST", path, body, token)


def try_login(
    call: Call, pin: str, sleep: Callable[[float], None] = time.sleep, budget_s: float = 2 * MAX_BACKOFF_S
) -> tuple[int, Mapping[str, str], object]:
    """POST /auth/login, waiting out every 429 LOGIN_BACKOFF. Returns the first non-429 answer."""
    waited = 0.0
    while True:
        status, headers, body = call("POST", LOGIN_PATH, {"pin": pin}, None)
        if status != 429:
            return status, headers, body
        wait = retry_after(headers) + 0.25
        waited += wait
        if waited > budget_s:
            raise AuthSkip("login backoff did not clear within the budget")
        sleep(wait)


def login_token(call: Call, pin: str, sleep: Callable[[float], None] = time.sleep) -> str:
    status, _, body = try_login(call, pin, sleep)
    token = body.get("token") if isinstance(body, dict) else None
    if status != 200 or not isinstance(token, str) or not token:
        raise LoginError(f"login failed (HTTP {status})")
    return token


def different_pin(*avoid: str) -> str:
    while True:
        pin = f"{secrets.randbelow(10**6):06d}"
        if pin not in avoid:
            return pin


def alive(call: Call, token: str) -> bool:
    """True when the token is a live session. GET /auth also restarts its idle timer."""
    status, _, body = call("GET", AUTH_PATH, None, token)
    return status == 200 and isinstance(body, dict) and body.get("authenticated") is True


class PinGuard:
    """Tracks what the device PIN may be, and puts the original back.

    state is "orig" (known), "new" (known to be the temporary PIN) or "unknown" (a change was
    sent and its answer is not known).
    """

    def __init__(self, call: Call, orig: str, sleep: Callable[[float], None] = time.sleep) -> None:
        self.call = call
        self.orig = orig
        self.new = different_pin(orig)
        self.sleep = sleep
        self.state = "orig"

    def change_to_new(self, token: str) -> tuple[int, Mapping[str, str], object]:
        self.state = "unknown"
        status, headers, body = self.call("PUT", PIN_PATH, {"old_pin": self.orig, "new_pin": self.new}, token)
        if status == 204:
            self.state = "new"
        elif status in (400, 401, 403, 422, 429, 503):
            self.state = "orig"  # refused before any write
        return status, headers, body

    def restore(self) -> None:
        """Make the original PIN valid again and prove it by a login. Raises PinRestoreError."""
        try:
            if self.state != "orig" and (self.state == "new" or not self._orig_logs_in()):
                token = login_token(self.call, self.new, self.sleep)
                status, _, _ = self.call("PUT", PIN_PATH, {"old_pin": self.new, "new_pin": self.orig}, token)
                if status != 204:
                    logout(post_of(self.call), token)
                    raise PinRestoreError(f"PUT /auth/pin back to the original PIN answered HTTP {status}")
            self.state = "orig"
            token = login_token(self.call, self.orig, self.sleep)
            logout(post_of(self.call), token)
        except PinRestoreError:
            raise
        except Exception as exc:  # noqa: BLE001 - any failure here must name the recovery
            raise PinRestoreError(f"restore failed ({type(exc).__name__})") from None

    def _orig_logs_in(self) -> bool:
        status, _, body = try_login(self.call, self.orig, self.sleep)
        token = body.get("token") if isinstance(body, dict) else None
        if status == 200 and isinstance(token, str):
            logout(post_of(self.call), token)
            return True
        return False


def run_pin_roundtrip(guard: PinGuard, sleep: Callable[[float], None] = time.sleep) -> None:
    """Change the PIN, prove new works and old fails, then restore. Raises AssertionError on a mismatch."""
    call = guard.call
    token = login_token(call, guard.orig, sleep)  # proof the current PIN works before any change
    try:
        status, _, body = guard.change_to_new(token)
        assert status == 204, f"PUT /auth/pin with the correct old PIN answered {status}"
        assert call("POST", LOGOUT_PATH, None, token)[0] == 401, "PUT /auth/pin must end the caller's session"

        status, headers, body = try_login(call, guard.orig, sleep)
        assert status == 401 and code_of(body) == "BAD_PIN", f"old PIN after change answered {status}"
        assert retry_after(headers) >= 1, "BAD_PIN must carry Retry-After"
        sleep(retry_after(headers) + 0.25)

        new_token = login_token(call, guard.new, sleep)
        assert alive(call, new_token), "login with the new PIN gave a dead session"
        logout(post_of(call), new_token)
    finally:
        guard.restore()


def run_wrong_old_pin(call: Call, orig: str, sleep: Callable[[float], None] = time.sleep) -> None:
    """A wrong old PIN gives 401 BAD_PIN with Retry-After; the PIN stays. new_pin is the original, so even a
    broken firmware that accepted the wrong old PIN would leave the PIN unchanged."""
    wrong = different_pin(orig)
    token = login_token(call, orig, sleep)
    try:
        status, headers, body = call("PUT", PIN_PATH, {"old_pin": wrong, "new_pin": orig}, token)
        assert status == 401 and code_of(body) == "BAD_PIN", f"wrong old PIN answered {status}"
        wait = retry_after(headers)
        assert wait >= 1, "BAD_PIN must carry Retry-After"
        sleep(wait + 0.25)
    finally:
        logout(post_of(call), token)
        verify_login(call, orig, sleep)


def run_bad_new_format(call: Call, orig: str, sleep: Callable[[float], None] = time.sleep) -> None:
    token = login_token(call, orig, sleep)
    try:
        for bad in ("12345", "1234567", "12345a", "", 123456, None):
            status, _, body = call("PUT", PIN_PATH, {"old_pin": orig, "new_pin": bad}, token)
            assert status == 422 and code_of(body) == "BAD_PIN_FORMAT", f"bad new_pin answered {status}"
        status, _, body = call("PUT", PIN_PATH, {"old_pin": orig}, token)
        assert status == 422 and code_of(body) == "BAD_PIN_FORMAT", f"missing new_pin answered {status}"
    finally:
        logout(post_of(call), token)
        verify_login(call, orig, sleep)


def run_no_token(call: Call, orig: str, sleep: Callable[[float], None] = time.sleep) -> None:
    """new_pin is the original, so a firmware that skipped the session check would not change the PIN."""
    try:
        status, _, body = call("PUT", PIN_PATH, {"old_pin": orig, "new_pin": orig}, None)
        assert status == 401 and code_of(body) == "UNAUTHORIZED", f"PUT /auth/pin without a token answered {status}"
    finally:
        verify_login(call, orig, sleep)


def verify_login(call: Call, pin: str, sleep: Callable[[float], None] = time.sleep) -> None:
    logout(post_of(call), login_token(call, pin, sleep))


def run_session_replacement(
    call: Call, pin: str, tokens: list[str], sleep: Callable[[float], None] = time.sleep
) -> None:
    """Log in 5 times. Tokens are appended to `tokens` as they appear so the caller can log all out.

    Other sessions (the web UI, a prior run) count toward the cap of 4 and are older, so they may be
    evicted first. What always holds: at least one of the 5 is dead, the dead ones are the oldest
    prefix, and the newest is alive.
    """
    for _ in range(5):
        tokens.append(login_token(call, pin, sleep))
    # GET /auth restarts the probed token's idle timer, but every eviction has already happened.
    states = [alive(call, t) for t in tokens]
    dead = states.count(False)
    assert dead >= 1, "a fifth login must replace a session"
    assert states == [False] * dead + [True] * (5 - dead), "the oldest sessions must be the ones replaced"
    assert states[-1], "the newest session must work"
    dead_token = tokens[0]
    assert call("POST", LOGOUT_PATH, None, dead_token)[0] == 401, "a replaced token must answer 401 UNAUTHORIZED"


def run_idle_expiry(call: Call, pin: str, sleep: Callable[[float], None], idle_s: float = 30 * 60) -> None:
    """The session must be dead after idle_s with no request. The only requests are the login and the final probe."""
    token = login_token(call, pin, sleep)
    try:
        sleep(idle_s + 60)
        status, _, body = call("POST", LOGOUT_PATH, None, token)
        assert status == 401 and code_of(body) == "UNAUTHORIZED", f"idle session answered {status}, expected 401"
    finally:
        logout(post_of(call), token)
