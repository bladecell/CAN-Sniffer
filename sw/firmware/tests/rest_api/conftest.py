import http.client
import os
import socket
import ssl
import threading
import time
from dataclasses import dataclass
from urllib.parse import urljoin, urlsplit

import pytest
import requests

from auth_session import AuthSkip, LoginError, login, logout, pin_from_env


CONNECT_TIMEOUT = 2.0
READ_TIMEOUT = 5.0
REQUEST_TIMEOUT = (CONNECT_TIMEOUT, READ_TIMEOUT)
REQUEST_INTERVAL = 0.05  # 20 requests/second, below the firmware-wide 50 requests/second limit.


def pytest_addoption(parser):
    parser.addoption(
        "--run-sd-write",
        action="store_true",
        default=False,
        help="allow tests marked sd_write (CAN_SNIFFER_ENABLE_SD_WRITE=1 is also required)",
    )
    parser.addoption(
        "--run-mutating-sd-security",
        action="store_true",
        default=False,
        help="allow potentially destructive SD POST/DELETE security probes",
    )
    parser.addoption(
        "--run-pid-def-write",
        action="store_true",
        default=False,
        help="allow the opt-in PID definition replacement/clear roundtrip",
    )
    parser.addoption(
        "--run-auth-mutating",
        action="store_true",
        default=False,
        help="allow tests that change the device PIN or fill its session slots (CAN_SNIFFER_ENABLE_AUTH_MUTATING=1 and CAN_SNIFFER_PIN also required)",
    )


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "auth_mutating: temporarily changes the PIN / fills session slots (needs --run-auth-mutating)"
    )
    config.addinivalue_line("markers", "auth_slow: 31 minute idle-expiry test (needs CAN_SNIFFER_SLOW_AUTH=1)")


def pytest_collection_modifyitems(config, items):
    skip_write = pytest.mark.skip(reason="pass --run-sd-write to opt in to SD write tests")
    skip_security = pytest.mark.skip(
        reason="pass --run-mutating-sd-security to opt in to mutating SD security probes"
    )
    skip_pid_def_write = pytest.mark.skip(
        reason="pass --run-pid-def-write and set CAN_SNIFFER_ENABLE_PID_DEF_WRITE=1"
    )
    skip_auth_mutating = pytest.mark.skip(
        reason="pass --run-auth-mutating and set CAN_SNIFFER_ENABLE_AUTH_MUTATING=1"
    )
    for item in items:
        if "auth_mutating" in item.keywords and not config.getoption("--run-auth-mutating"):
            item.add_marker(skip_auth_mutating)
        if "sd_write" in item.keywords and not config.getoption("--run-sd-write"):
            item.add_marker(skip_write)
        if "sd_mutating_security" in item.keywords and not config.getoption("--run-mutating-sd-security"):
            item.add_marker(skip_security)
        if "pid_def_write" in item.keywords and not config.getoption("--run-pid-def-write"):
            item.add_marker(skip_pid_def_write)


class RateLimiter:
    def __init__(self, interval: float = REQUEST_INTERVAL):
        self.interval = interval
        self._lock = threading.Lock()
        self._next_request = 0.0

    def wait(self):
        with self._lock:
            now = time.monotonic()
            delay = self._next_request - now
            if delay > 0:
                time.sleep(delay)
            self._next_request = time.monotonic() + self.interval


class ApiClient:
    def __init__(self, base_url: str, limiter: RateLimiter, token: str | None = None):
        self.token = token
        self.base_url = base_url.rstrip("/") + "/"
        self.limiter = limiter
        self.session = requests.Session()
        self.session.trust_env = False

    def request(self, method: str, path: str, **kwargs):
        kwargs.setdefault("timeout", REQUEST_TIMEOUT)
        kwargs.setdefault("allow_redirects", False)
        if method == "PUT" and "data" in kwargs and "json" not in kwargs:
            headers = dict(kwargs.get("headers") or {})
            if not any(k.lower() == "content-type" for k in headers):
                headers["Content-Type"] = "application/octet-stream"
            kwargs["headers"] = headers
        if self.token:
            kwargs["headers"] = {**(kwargs.get("headers") or {}), "Authorization": f"Bearer {self.token}"}
        self.limiter.wait()
        response = self.session.request(method, urljoin(self.base_url, path.lstrip("/")), **kwargs)
        if response.status_code == 429:
            retry_after = response.headers.get("Retry-After", "not provided")
            raise RuntimeError(f"firmware rate limit returned HTTP 429 (Retry-After: {retry_after})")
        return response

    def close(self):
        self.session.close()


@dataclass(frozen=True)
class RawResponse:
    status: int
    headers: tuple[tuple[str, str], ...]
    body: bytes


def raw_request(base_url: str, method: str, target: str, limiter: RateLimiter, body: bytes = b"") -> RawResponse:
    """Send an exact request-target without requests/urllib path normalization."""
    parsed = urlsplit(base_url)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError(f"Unsupported CAN_SNIFFER_URL: {base_url!r}")
    if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
        raise ValueError("CAN_SNIFFER_URL must contain only scheme and authority")
    if not target.startswith("/"):
        raise ValueError("raw request target must start with '/'")

    connection_type = http.client.HTTPSConnection if parsed.scheme == "https" else http.client.HTTPConnection
    connection = connection_type(parsed.hostname, parsed.port, timeout=CONNECT_TIMEOUT)
    try:
        limiter.wait()
        connection.connect()
        connection.sock.settimeout(READ_TIMEOUT)
        headers = {"Content-Length": str(len(body)), "Connection": "close"}
        connection.request(method, target, body=body, headers=headers)
        response = connection.getresponse()
        result = RawResponse(response.status, tuple(response.getheaders()), response.read(64 * 1024))
        if result.status == 429:
            retry_after = dict(result.headers).get("Retry-After", "not provided")
            raise RuntimeError(f"firmware rate limit returned HTTP 429 (Retry-After: {retry_after})")
        return result
    finally:
        connection.close()


def raw_request_with_headers(
    base_url: str, method: str, target: str, limiter: RateLimiter, headers: tuple[str, ...], body: bytes = b""
) -> RawResponse:
    """Send exact header lines, including deliberately absent or malformed fields."""
    parsed = urlsplit(base_url)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError(f"Unsupported CAN_SNIFFER_URL: {base_url!r}")
    if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
        raise ValueError("CAN_SNIFFER_URL must contain only scheme and authority")
    if not target.startswith("/"):
        raise ValueError("raw request target must start with '/'")

    port = parsed.port or (443 if parsed.scheme == "https" else 80)
    limiter.wait()
    sock = socket.create_connection((parsed.hostname, port), timeout=CONNECT_TIMEOUT)
    try:
        sock.settimeout(READ_TIMEOUT)
        if parsed.scheme == "https":
            context = ssl.create_default_context()
            sock = context.wrap_socket(sock, server_hostname=parsed.hostname)
            sock.settimeout(READ_TIMEOUT)

        host = parsed.hostname
        if ":" in host and not host.startswith("["):
            host = f"[{host}]"
        if parsed.port:
            host = f"{host}:{parsed.port}"
        request = (
            f"{method} {target} HTTP/1.1\r\n"
            f"Host: {host}\r\n"
            + "".join(f"{header}\r\n" for header in headers)
            + "Connection: close\r\n\r\n"
        ).encode("ascii")
        sock.sendall(request + body)

        response = http.client.HTTPResponse(sock)
        response.begin()
        result = RawResponse(response.status, tuple(response.getheaders()), response.read(64 * 1024))
        if result.status == 429:
            retry_after = dict(result.headers).get("Retry-After", "not provided")
            raise RuntimeError(f"firmware rate limit returned HTTP 429 (Retry-After: {retry_after})")
        return result
    finally:
        sock.close()


def fragmented_raw_request(
    base_url: str, method: str, target: str, limiter: RateLimiter, body: bytes
) -> RawResponse:
    """Send the request body in several deliberate TCP writes."""
    parsed = urlsplit(base_url)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError(f"Unsupported CAN_SNIFFER_URL: {base_url!r}")
    if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
        raise ValueError("CAN_SNIFFER_URL must contain only scheme and authority")
    if not target.startswith("/"):
        raise ValueError("raw request target must start with '/'")

    port = parsed.port or (443 if parsed.scheme == "https" else 80)
    limiter.wait()
    sock = socket.create_connection((parsed.hostname, port), timeout=CONNECT_TIMEOUT)
    try:
        sock.settimeout(READ_TIMEOUT)
        if parsed.scheme == "https":
            context = ssl.create_default_context()
            sock = context.wrap_socket(sock, server_hostname=parsed.hostname)
            sock.settimeout(READ_TIMEOUT)

        host = parsed.hostname
        if ":" in host and not host.startswith("["):
            host = f"[{host}]"
        if parsed.port:
            host = f"{host}:{parsed.port}"
        headers = (
            f"{method} {target} HTTP/1.1\r\n"
            f"Host: {host}\r\n"
            f"Content-Length: {len(body)}\r\n"
            "Content-Type: application/json\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii")
        sock.sendall(headers)

        split_one = max(1, len(body) // 3)
        split_two = max(split_one + 1, (2 * len(body)) // 3)
        chunks = (body[:split_one], body[split_one:split_two], body[split_two:])
        for chunk in chunks:
            if chunk:
                sock.sendall(chunk)
                time.sleep(0.02)

        response = http.client.HTTPResponse(sock)
        response.begin()
        result = RawResponse(response.status, tuple(response.getheaders()), response.read(64 * 1024))
        if result.status == 429:
            retry_after = dict(result.headers).get("Retry-After", "not provided")
            raise RuntimeError(f"firmware rate limit returned HTTP 429 (Retry-After: {retry_after})")
        return result
    finally:
        sock.close()


@pytest.fixture(scope="session")
def base_url():
    return os.environ.get("CAN_SNIFFER_URL", "http://can-sniffer.local").rstrip("/")


@pytest.fixture(scope="session")
def request_limiter():
    return RateLimiter()


@pytest.fixture(scope="session")
def api(base_url, request_limiter):
    client = ApiClient(base_url, request_limiter)
    yield client
    client.close()


@pytest.fixture(scope="session")
def raw_http(base_url, request_limiter):
    return lambda method, target, body=b"": raw_request(base_url, method, target, request_limiter, body)


@pytest.fixture(scope="session")
def raw_http_headers(base_url, request_limiter):
    return lambda method, target, headers, body=b"": raw_request_with_headers(
        base_url, method, target, request_limiter, headers, body
    )


@pytest.fixture(scope="session")
def fragmented_raw_http(base_url, request_limiter):
    return lambda method, target, body=b"": fragmented_raw_request(
        base_url, method, target, request_limiter, body
    )


@pytest.fixture
def sd_write_enabled(request):
    if not request.config.getoption("--run-sd-write"):
        pytest.skip("pass --run-sd-write to opt in to SD writes")
    if os.environ.get("CAN_SNIFFER_ENABLE_SD_WRITE") != "1":
        pytest.skip("set CAN_SNIFFER_ENABLE_SD_WRITE=1 to enable temporary SD writes")


@pytest.fixture
def mutating_sd_security_enabled(request):
    if not request.config.getoption("--run-mutating-sd-security"):
        pytest.skip("pass --run-mutating-sd-security to enable mutating SD security probes")
    if os.environ.get("CAN_SNIFFER_ENABLE_SD_WRITE") != "1":
        pytest.skip("set CAN_SNIFFER_ENABLE_SD_WRITE=1 to enable mutating SD security probes")


@pytest.fixture
def pid_def_write_enabled(request):
    if not request.config.getoption("--run-pid-def-write"):
        pytest.skip("pass --run-pid-def-write to opt in to PID definition replacement")
    if os.environ.get("CAN_SNIFFER_ENABLE_PID_DEF_WRITE") != "1":
        pytest.skip("set CAN_SNIFFER_ENABLE_PID_DEF_WRITE=1 to enable PID definition replacement")


@pytest.fixture(scope="session")
def device_reachable(api):
    try:
        response = api.request("GET", "/api/v1/system")
    except requests.RequestException as exc:
        pytest.skip(f"CAN Sniffer is unreachable: {exc}")
    if response.status_code >= 500:
        pytest.skip(f"CAN Sniffer is not ready: HTTP {response.status_code}")
    return True


@pytest.fixture(scope="session")
def sd_info(api, device_reachable):
    response = api.request("GET", "/api/v1/sd_card/info")
    assert response.status_code == 200
    payload = response.json()
    assert isinstance(payload, dict)
    return payload


@pytest.fixture(scope="session")
def mounted_sd(sd_info):
    if not sd_info.get("is_mounted"):
        pytest.skip("SD-card-dependent test skipped: SD card is not mounted")
    return sd_info


@pytest.fixture(scope="session")
def auth_token(base_url, request_limiter, device_reachable):
    """One login per run; logs out at the end. Skips when CAN_SNIFFER_PIN is unset."""
    try:
        pin = pin_from_env(os.environ)
    except LoginError as exc:
        pytest.fail(str(exc), pytrace=False)
    if pin is None:
        pytest.skip("set CAN_SNIFFER_PIN to run gated-route tests")

    session = requests.Session()
    session.trust_env = False

    def post(path, body=None, token=None):
        request_limiter.wait()
        headers = {"Authorization": f"Bearer {token}"} if token else {}
        response = session.post(
            urljoin(base_url + "/", path.lstrip("/")),
            json=body,
            headers=headers,
            timeout=REQUEST_TIMEOUT,
            allow_redirects=False,
        )
        try:
            payload = response.json()
        except ValueError:
            payload = None
        return response.status_code, dict(response.headers), payload

    try:
        token = login(post, pin)
    except AuthSkip as exc:
        session.close()
        pytest.skip(str(exc))
    except LoginError as exc:
        session.close()
        pytest.fail(str(exc), pytrace=False)
    except requests.RequestException as exc:
        session.close()
        pytest.skip(f"login request failed: {type(exc).__name__}")
    yield token
    logout(post, token)
    session.close()


@pytest.fixture(scope="session")
def auth_api(base_url, request_limiter, auth_token):
    """ApiClient that sends the Bearer token. Use `api` for unauthenticated checks."""
    client = ApiClient(base_url, request_limiter, token=auth_token)
    yield client
    client.close()


@pytest.fixture
def auth_call(base_url, request_limiter, device_reachable):
    """call(method, path, body=None, token=None) -> (status, headers, JSON); never raises on 429."""
    session = requests.Session()
    session.trust_env = False

    def call(method, path, body=None, token=None):
        request_limiter.wait()
        headers = {"Authorization": f"Bearer {token}"} if token else {}
        response = session.request(
            method,
            urljoin(base_url + "/", path.lstrip("/")),
            json=body,
            headers=headers,
            timeout=REQUEST_TIMEOUT,
            allow_redirects=False,
        )
        try:
            payload = response.json()
        except ValueError:
            payload = None
        return response.status_code, dict(response.headers), payload

    yield call
    session.close()
