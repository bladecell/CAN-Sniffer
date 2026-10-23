"""Hardware tests for PUT /auth/pin, the 4-session cap and idle expiry (P2.10).

The mutating tests change the real device's PIN or fill its session slots, so they are opt-in:
  CAN_SNIFFER_PIN=<pin> CAN_SNIFFER_ENABLE_AUTH_MUTATING=1 pytest --run-auth-mutating tests/rest_api/test_auth.py
Run them on their own, before or without other gated tests: PUT /auth/pin ends every session and a fifth
login can evict the suite's shared auth_token. Recovery if a restore fails is a 10 s BOOT hold.
No PIN is printed. The original comes only from CAN_SNIFFER_PIN; the temporary one is generated here.
"""
import os
import time

import pytest

from auth_scenarios import (
    AUTH_PATH,
    PinGuard,
    PinRestoreError,
    login_token,
    post_of,
    run_bad_new_format,
    run_idle_expiry,
    run_no_token,
    run_pin_roundtrip,
    run_session_replacement,
    run_wrong_old_pin,
    verify_login,
)
from auth_session import AuthSkip, LoginError, logout, pin_from_env

pytestmark = pytest.mark.integration


@pytest.fixture
def auth_mutating_enabled(request):
    if not request.config.getoption("--run-auth-mutating"):
        pytest.skip("pass --run-auth-mutating to opt in to PIN changes and session filling")
    if os.environ.get("CAN_SNIFFER_ENABLE_AUTH_MUTATING") != "1":
        pytest.skip("set CAN_SNIFFER_ENABLE_AUTH_MUTATING=1 to allow a temporary PIN change")


@pytest.fixture
def proven_pin(auth_call):
    """The original PIN, proven to log in on a claimed device before anything is changed. Proves it again after."""
    try:
        pin = pin_from_env(os.environ)
    except LoginError as exc:
        pytest.fail(str(exc), pytrace=False)
    if pin is None:
        pytest.skip("set CAN_SNIFFER_PIN")
    status, _, body = auth_call("GET", AUTH_PATH)
    if status != 200 or not isinstance(body, dict) or body.get("state") != "claimed":
        pytest.skip("device is not claimed (or auth is unavailable)")
    if body.get("dev_bypass") is True:
        pytest.skip("dev auth bypass image: PIN behaviour is not testable")
    try:
        verify_login(auth_call, pin)
    except AuthSkip as exc:
        pytest.skip(str(exc))
    except LoginError:
        pytest.fail("CAN_SNIFFER_PIN does not log in; refusing to touch the PIN", pytrace=False)
    yield pin
    try:
        verify_login(auth_call, pin)
    except Exception:  # noqa: BLE001
        pytest.fail("original PIN no longer logs in: hold BOOT for 10 s on the device to unclaim it", pytrace=False)


@pytest.mark.auth_mutating
def test_pin_change_round_trip(auth_mutating_enabled, auth_call, proven_pin):
    guard = PinGuard(auth_call, proven_pin)
    try:
        run_pin_roundtrip(guard)
    except PinRestoreError:
        pytest.fail("PIN restore FAILED: hold BOOT for 10 s on the device to unclaim it", pytrace=False)


@pytest.mark.auth_mutating
def test_wrong_old_pin_is_a_counted_failed_login(auth_mutating_enabled, auth_call, proven_pin):
    run_wrong_old_pin(auth_call, proven_pin)


@pytest.mark.auth_mutating
def test_bad_new_pin_format_is_422(auth_mutating_enabled, auth_call, proven_pin):
    run_bad_new_format(auth_call, proven_pin)


@pytest.mark.auth_mutating
def test_pin_change_without_token_is_401(auth_mutating_enabled, auth_call, proven_pin):
    run_no_token(auth_call, proven_pin)


@pytest.mark.auth_mutating
def test_fifth_login_replaces_the_oldest_session(auth_mutating_enabled, auth_call, proven_pin):
    tokens: list[str] = []
    try:
        run_session_replacement(auth_call, proven_pin, tokens)
    finally:
        post = post_of(auth_call)
        for token in tokens:
            logout(post, token)


@pytest.fixture
def slow_auth_enabled():
    if os.environ.get("CAN_SNIFFER_SLOW_AUTH") != "1":
        pytest.skip("set CAN_SNIFFER_SLOW_AUTH=1 to run the 31 minute idle-expiry test")


@pytest.mark.auth_slow
def test_session_expires_after_30_minutes_idle(slow_auth_enabled, auth_call, proven_pin):
    """Takes about 31 minutes. The 12 h absolute expiry is not tested: it cannot be shortened without a firmware
    change and a 12 h run is not practical."""
    run_idle_expiry(auth_call, proven_pin, time.sleep)
