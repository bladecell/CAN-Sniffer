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
        "/api/v1/system/copy_file",
        data=body,
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == 400, (response.status_code, response.text)


def test_copy_file_parses_valid_json_sent_in_fragments(fragmented_raw_http, device_reachable):
    # The paths are intentionally outside /sdcard: middleware validation rejects
    # them before any filesystem operation, so this test never mutates state.
    body = b'{"source_path":"/tmp/source","destination_path":"/tmp/destination"}'
    response = fragmented_raw_http("POST", "/api/v1/system/copy_file", body)

    assert response.status == 400, response.body
    assert "application/json" in dict(response.headers).get("Content-Type", "")
    payload = json.loads(response.body)
    assert payload == {
        "status": "error",
        "reason": "source_path and destination_path must be under /sdcard",
    }
