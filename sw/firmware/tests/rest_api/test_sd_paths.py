import uuid

import pytest


pytestmark = pytest.mark.integration

FILE_ROUTE = "/api/v1/sd_card/file"
NONEXISTENT = f"__pytest_path_guard_{uuid.uuid4().hex}__"


@pytest.mark.parametrize("suffix", ["/", "/.", "/..", "/%2e", "/%2e%2e"])
def test_file_root_and_normalized_roots_are_rejected_by_get(raw_http, device_reachable, suffix):
    response = raw_http("GET", FILE_ROUTE + suffix)
    assert response.status == 400, ("GET", suffix, response.status, response.body)


MALICIOUS_SUFFIXES = [
    f"/%2e%2e/{NONEXISTENT}/file.txt",
    f"/%2E/{NONEXISTENT}/file.txt",
    f"/{NONEXISTENT}//file.txt",
    f"/{NONEXISTENT}/file%2ftail.txt",
    f"/{NONEXISTENT}/file%5ctail.txt",
    f"/{NONEXISTENT}/file%3ftail.txt",
    f"/{NONEXISTENT}/file%23tail.txt",
    f"/{NONEXISTENT}/file%22tail.txt",
    f"/{NONEXISTENT}/file%01tail.txt",
    f"/{NONEXISTENT}/file%",
    f"/{NONEXISTENT}/file%2",
    f"/{NONEXISTENT}/file%GG",
]


@pytest.mark.parametrize("suffix", MALICIOUS_SUFFIXES)
def test_exact_malicious_get_targets_are_rejected(raw_http, device_reachable, suffix):
    # Raw transport is intentional: requests requotes malformed '%' sequences
    # and may normalize dot segments before sending them.
    response = raw_http("GET", FILE_ROUTE + suffix)
    assert response.status == 400, ("GET", suffix, response.status, response.body)


@pytest.mark.sd_mutating_security
@pytest.mark.parametrize("method", ["POST", "DELETE"])
@pytest.mark.parametrize("suffix", ["/", "/.", "/..", "/%2e", "/%2e%2e"] + MALICIOUS_SUFFIXES)
def test_mutating_file_targets_are_rejected(
    raw_http, device_reachable, mutating_sd_security_enabled, method, suffix
):
    response = raw_http(method, FILE_ROUTE + suffix)
    assert response.status == 400, (method, suffix, response.status, response.body)


@pytest.mark.parametrize("suffix", [
    "/%2e%2e/escape",
    "/safe//child",
    "/safe/%2e/child",
    "/safe/%5cchild",
    "/safe/%23child",
    "/safe/%01child",
    "/safe/%",
])
def test_tree_rejects_malicious_exact_targets(raw_http, device_reachable, suffix):
    response = raw_http("GET", "/api/v1/sd_card/tree" + suffix)
    assert response.status == 400, (suffix, response.status, response.body)
