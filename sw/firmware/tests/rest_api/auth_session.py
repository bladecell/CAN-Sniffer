"""Login helper for the gated routes. The PIN comes only from the environment and is never echoed."""
import time
from collections.abc import Callable, Mapping

PIN_ENV = "CAN_SNIFFER_PIN"
LOGIN_PATH = "/api/v1/auth/login"
LOGOUT_PATH = "/api/v1/auth/logout"

# post(path, body=None, token=None) -> (status, headers, parsed JSON body or None)
Post = Callable[..., tuple[int, Mapping[str, str], object]]


class AuthSkip(Exception):
    """Gated tests cannot run (backoff too long, device unclaimed or auth unavailable)."""


class LoginError(Exception):
    """Login failed in a way that should fail the run. Messages never contain the PIN."""


def pin_from_env(env: Mapping[str, str]) -> str | None:
    pin = env.get(PIN_ENV, "").strip()
    if not pin:
        return None
    if not (len(pin) == 6 and pin.isascii() and pin.isdigit()):
        raise LoginError(f"{PIN_ENV} must be exactly 6 digits")
    return pin


def redact(text: str, *secrets: str) -> str:
    for secret in secrets:
        if secret:
            text = text.replace(secret, "***")
    return text


def _retry_after(headers: Mapping[str, str]) -> float:
    for key, value in headers.items():
        if key.lower() == "retry-after":
            try:
                return max(0.0, float(value))
            except ValueError:
                break
    return 1.0


def login(
    post: Post,
    pin: str,
    sleep: Callable[[float], None] = time.sleep,
    max_wait: float = 60.0,
    attempts: int = 3,
) -> str:
    """Return a session token. Only a 429 is retried; a wrong PIN is never retried (it feeds the backoff)."""
    for _ in range(attempts):
        status, headers, body = post(LOGIN_PATH, {"pin": pin})
        if status == 200:
            token = body.get("token") if isinstance(body, dict) else None
            if not isinstance(token, str) or not token:
                raise LoginError("login answered 200 without a token")
            return token
        if status == 429:
            wait = _retry_after(headers)
            if wait > max_wait:
                raise AuthSkip(f"login backoff of {wait:.0f}s exceeds {max_wait:.0f}s")
            sleep(wait)
            continue
        if status in (403, 503):
            raise AuthSkip(f"auth unavailable (HTTP {status}): device not claimed or auth down")
        raise LoginError(f"login rejected (HTTP {status}); check {PIN_ENV}")
    raise AuthSkip(f"login still rate limited after {attempts} attempts")


def logout(post: Post, token: str) -> None:
    """Free the session slot (the device keeps at most 4). Never raises."""
    try:
        post(LOGOUT_PATH, None, token)
    except Exception:  # noqa: BLE001 - best-effort cleanup; the session expires after 30 min idle
        pass
