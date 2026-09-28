import uuid

import pytest


pytestmark = pytest.mark.integration

FILE_ROUTE = "/api/v1/sd_card/file"
NONEXISTENT = f"__pytest_path_guard_{uuid.uuid4().hex}__"


@pytest.mark.parametrize(
    "query",
    [
        pytest.param("", id="absent"),
        pytest.param("?download=true", id="true"),
        pytest.param("?download=false", id="false"),
        pytest.param("?download=1", id="one"),
        pytest.param("?download=0", id="zero"),
    ],
)
def test_file_read_accepts_canonical_download_queries(raw_http, device_reachable, query):
    response = raw_http("GET", f"{FILE_ROUTE}/{NONEXISTENT}{query}")

    # The target deliberately does not exist. Any result other than 400 proves
    # the query passed validation without creating or changing SD-card data.
    assert response.status != 400, (query, response.status, response.body)


@pytest.mark.parametrize(
    "query",
    [
        pytest.param("?", id="empty-query"),
        pytest.param("?download", id="missing-equals"),
        pytest.param("?download=", id="empty-value"),
        pytest.param("?download=true&", id="trailing-ampersand"),
        pytest.param("?&download=true", id="leading-empty-component"),
        pytest.param("?download=true&&", id="empty-component"),
        pytest.param("?download=true&download=false", id="duplicate-key"),
        pytest.param("?preview=true", id="unknown-key"),
        pytest.param("?download=true&preview=false", id="mixed-known-and-unknown-keys"),
        pytest.param("?Download=true", id="case-variant-key"),
        pytest.param("?download=True", id="case-variant-value"),
        pytest.param("?download=01", id="malformed-value"),
        pytest.param("?download=%74rue", id="encoded-value"),
        pytest.param("?download=true=false", id="extra-equals-in-value"),
    ],
)
def test_file_read_rejects_noncanonical_download_queries_before_sd_access(
    raw_http, device_reachable, query
):
    response = raw_http("GET", f"{FILE_ROUTE}/{NONEXISTENT}{query}")

    assert response.status == 400, (query, response.status, response.body)
    assert "application/json" in dict(response.headers).get("Content-Type", "")


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
