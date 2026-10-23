"""Offline tests of the login helper; they use a fake HTTP layer and never touch a device."""
import pytest

from auth_session import AuthSkip, LoginError, login, logout, pin_from_env, redact

PIN = "482915"
TOKEN = "a" * 32
OK = (200, {}, {"status": "ok", "token": TOKEN})


class FakePost:
    def __init__(self, *responses):
        self.responses = list(responses)
        self.calls = []

    def __call__(self, path, body=None, token=None):
        self.calls.append((path, body, token))
        return self.responses.pop(0)


def test_pin_from_env_unset_or_blank():
    assert pin_from_env({}) is None
    assert pin_from_env({"CAN_SNIFFER_PIN": "  "}) is None
    assert pin_from_env({"CAN_SNIFFER_PIN": PIN}) == PIN


def test_pin_from_env_rejects_non_six_digits_without_echo():
    with pytest.raises(LoginError) as exc:
        pin_from_env({"CAN_SNIFFER_PIN": "12ab"})
    assert "12ab" not in str(exc.value)


def test_login_success_sends_pin_once():
    post = FakePost(OK)
    assert login(post, PIN, sleep=lambda s: None) == TOKEN
    assert post.calls == [("/api/v1/auth/login", {"pin": PIN}, None)]


def test_login_retries_after_429_honoring_retry_after():
    slept = []
    post = FakePost((429, {"Retry-After": "3"}, {"code": "LOGIN_BACKOFF"}), OK)
    assert login(post, PIN, sleep=slept.append) == TOKEN
    assert slept == [3.0]


def test_login_skips_when_backoff_too_long():
    post = FakePost((429, {"Retry-After": "900"}, {}))
    with pytest.raises(AuthSkip):
        login(post, PIN, sleep=lambda s: pytest.fail("must not sleep"), max_wait=60)


def test_login_gives_up_after_repeated_429():
    post = FakePost(*[(429, {"Retry-After": "1"}, {})] * 5)
    with pytest.raises(AuthSkip):
        login(post, PIN, sleep=lambda s: None, attempts=3)
    assert len(post.calls) == 3


def test_bad_pin_is_not_retried_and_never_leaked():
    post = FakePost((401, {"Retry-After": "1"}, {"code": "BAD_PIN"}))
    with pytest.raises(LoginError) as exc:
        login(post, PIN, sleep=lambda s: None)
    assert PIN not in str(exc.value)
    assert len(post.calls) == 1


@pytest.mark.parametrize("status", [403, 503])
def test_unclaimed_or_unavailable_skips(status):
    with pytest.raises(AuthSkip):
        login(FakePost((status, {}, {})), PIN, sleep=lambda s: None)


def test_missing_token_is_login_error():
    with pytest.raises(LoginError):
        login(FakePost((200, {}, {"status": "ok"})), PIN, sleep=lambda s: None)


def test_logout_sends_bearer_and_swallows_errors():
    post = FakePost((204, {}, None))
    logout(post, TOKEN)
    assert post.calls == [("/api/v1/auth/logout", None, TOKEN)]

    def boom(*a, **k):
        raise OSError("down")

    logout(boom, TOKEN)


def test_redact_hides_pin_and_token():
    assert redact(f"pin={PIN} tok={TOKEN}", PIN, TOKEN) == "pin=*** tok=***"
