import json

import pytest


pytestmark = pytest.mark.integration


@pytest.mark.parametrize(
    "body",
    [
        pytest.param(b"[", id="incomplete-json"),
        pytest.param(b"{}{}", id="trailing-json"),
        pytest.param(b"{} trailing", id="trailing-non-whitespace"),
    ],
)
def test_copy_file_rejects_incomplete_or_trailing_json(api, device_reachable, body):
    response = api.request(
        "POST",
        "/api/v1/sd_card/copy",
        data=body,
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == 400, (response.status_code, response.text)


def test_copy_file_parses_valid_json_sent_in_fragments(fragmented_raw_http, device_reachable):
    # The paths contain "..": middleware validation rejects them before any
    # filesystem operation, so this test never mutates state.
    body = b'{"source":"a/../source","destination":"b/../destination"}'
    response = fragmented_raw_http("POST", "/api/v1/sd_card/copy", body)

    assert response.status == 400, response.body
    assert "application/json" in dict(response.headers).get("Content-Type", "")
    payload = json.loads(response.body)
    assert payload["status"] == "error"
    assert payload["reason"] == "source and destination must be valid relative file paths"
