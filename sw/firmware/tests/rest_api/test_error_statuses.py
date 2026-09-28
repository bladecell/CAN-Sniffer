import uuid

import pytest


pytestmark = pytest.mark.integration


def assert_json_error(response, expected_status=None):
    assert not 200 <= response.status_code < 300, response.text
    if expected_status is not None:
        assert response.status_code == expected_status, response.text
    assert "application/json" in response.headers.get("Content-Type", "")
    payload = response.json()
    assert isinstance(payload, dict), payload
    assert payload.get("status") == "error", payload
    assert isinstance(payload.get("reason"), str), payload
    assert payload["reason"], payload


def test_pid_post_rejects_json_object_with_json_error(api, device_reachable):
    response = api.request(
        "POST",
        "/api/v1/pid_def",
        data=b"{}",
        headers={"Content-Type": "application/json"},
    )

    assert_json_error(response, 400)


@pytest.mark.parametrize(
    "body",
    [
        pytest.param(b"1", id="scalar"),
        pytest.param(b"{}", id="missing-section-name"),
    ],
)
def test_settings_rejects_scalar_or_malformed_payload_with_json_error(
    api, device_reachable, body
):
    response = api.request(
        "POST",
        "/api/v1/settings",
        data=body,
        headers={"Content-Type": "application/json"},
    )

    assert_json_error(response)


@pytest.mark.parametrize(
    "body",
    [
        pytest.param(b"{}", id="missing-fields"),
        pytest.param(
            b'{"source_path":"/sdcard/source","destination_path":1}',
            id="invalid-field-type",
        ),
    ],
)
def test_copy_file_rejects_malformed_or_missing_fields_with_json_error(
    api, device_reachable, body
):
    response = api.request(
        "POST",
        "/api/v1/system/copy_file",
        data=body,
        headers={"Content-Type": "application/json"},
    )

    assert_json_error(response, 400)


def test_copy_file_reports_confirmed_missing_source_as_not_found(api, mounted_sd):
    source_path = f"/sdcard/gate6-missing-{uuid.uuid4().hex}.json"
    destination_path = f"/sdcard/gate6-destination-{uuid.uuid4().hex}.json"

    response = api.request(
        "POST",
        "/api/v1/system/copy_file",
        json={"source_path": source_path, "destination_path": destination_path},
    )

    assert_json_error(response, 404)


def test_pid_definition_load_reports_confirmed_missing_file_as_not_found(api, mounted_sd):
    path = f"/sdcard/gate6-missing-{uuid.uuid4().hex}.json"

    response = api.request("POST", "/api/v1/pid_def/load", json={"pid_def_path": path})

    assert_json_error(response, 404)


def test_dtc_rejects_unsupported_mode_before_can_work(api, device_reachable):
    response = api.request("GET", "/api/v1/dtc", params={"mode": "999"})

    assert_json_error(response, 422)


@pytest.mark.parametrize(
    "codes",
    [
        pytest.param("ZZZZZ", id="unsupported-system"),
        pytest.param("p0123", id="lowercase"),
        pytest.param("P4123", id="invalid-second-digit"),
        pytest.param("P01G3", id="invalid-hex-tail"),
        pytest.param("P012", id="wrong-length"),
        pytest.param("", id="empty-list"),
        pytest.param(",P0123", id="leading-empty-item"),
        pytest.param("P0123,,B3ABC", id="middle-empty-item"),
        pytest.param("P0123,", id="trailing-empty-item"),
    ],
)
def test_dtc_description_rejects_invalid_codes_before_lookup(api, device_reachable, codes):
    response = api.request("GET", "/api/v1/dtc", params={"codes": codes})

    assert response.status_code == 400, response.text


def test_dtc_description_accepts_multiple_standard_codes(api, device_reachable):
    codes = "P0123,B3ABC,C0DEF,U3000"
    response = api.request("GET", "/api/v1/dtc", params={"codes": codes})

    assert response.request.path_url.endswith("codes=P0123%2CB3ABC%2CC0DEF%2CU3000")

    # Description data is optional on a device, but a valid query must reach
    # the description lookup rather than fail request validation.
    assert response.status_code != 400, response.text
    if response.status_code == 200:
        payload = response.json()
        assert payload["dtc_count"] == 4
        assert [item["dtc"] for item in payload["dtcs"]] == codes.split(",")


def test_dtc_description_accepts_literal_commas(raw_http, device_reachable):
    response = raw_http("GET", "/api/v1/dtc?codes=P0123,B3ABC,C0DEF,U3000")

    # As above, the description database is optional, but literal separators
    # must also get past query validation.
    assert response.status != 400


@pytest.mark.parametrize(
    "target",
    [
        pytest.param("/api/v1/dtc?codes=P0123%2", id="malformed-percent-escape"),
        pytest.param("/api/v1/dtc?codes=P0123%2C%2CB3ABC", id="encoded-empty-item"),
        pytest.param(
            "/api/v1/dtc?codes=" + "%2C".join(["P0123"] * 31),
            id="more-than-maximum-code-count",
        ),
    ],
)
def test_dtc_description_rejects_invalid_encoded_lists(raw_http, device_reachable, target):
    response = raw_http("GET", target)

    assert response.status == 400


@pytest.mark.parametrize(
    "mode",
    [
        pytest.param(259, id="positive-wraps-to-confirmed"),
        pytest.param(263, id="positive-wraps-to-pending"),
        pytest.param(266, id="positive-wraps-to-permanent"),
        pytest.param(-253, id="negative-wraps-to-confirmed"),
        pytest.param(-249, id="negative-wraps-to-pending"),
        pytest.param(-246, id="negative-wraps-to-permanent"),
    ],
)
def test_dtc_request_rejects_narrowing_values_before_can_work(api, device_reachable, mode):
    response = api.request("POST", "/api/v1/req/dtc", params={"mode": str(mode)})

    assert_json_error(response, 422)
    assert response.json()["reason"] == "Unsupported DTC mode"


def test_sd_tree_reports_unavailable_card_with_json_error(api, device_reachable, sd_info):
    if sd_info.get("is_mounted") is not False:
        pytest.skip("SD-card-unavailable case is not supported by this environment")

    response = api.request("GET", "/api/v1/sd_card/tree")

    assert_json_error(response, 503)
