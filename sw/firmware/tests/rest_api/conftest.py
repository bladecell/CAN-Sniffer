import http.client
import os
import threading
import time
from dataclasses import dataclass
from urllib.parse import urljoin, urlsplit

import pytest
import requests


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


def pytest_collection_modifyitems(config, items):
    skip_write = pytest.mark.skip(reason="pass --run-sd-write to opt in to SD write tests")
    skip_security = pytest.mark.skip(
        reason="pass --run-mutating-sd-security to opt in to mutating SD security probes"
    )
    for item in items:
        if "sd_write" in item.keywords and not config.getoption("--run-sd-write"):
            item.add_marker(skip_write)
        if "sd_mutating_security" in item.keywords and not config.getoption("--run-mutating-sd-security"):
            item.add_marker(skip_security)


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
    def __init__(self, base_url: str, limiter: RateLimiter):
        self.base_url = base_url.rstrip("/") + "/"
        self.limiter = limiter
        self.session = requests.Session()
        self.session.trust_env = False

    def request(self, method: str, path: str, **kwargs):
        kwargs.setdefault("timeout", REQUEST_TIMEOUT)
        kwargs.setdefault("allow_redirects", False)
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
