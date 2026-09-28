import json
import uuid
from urllib.parse import quote

import pytest


pytestmark = [pytest.mark.integration, pytest.mark.sd_write]


def assert_success(response):
    assert response.status_code == 200, response.text
    assert response.json().get("status") == "success", response.text


def assert_json_error(response, expected_status):
    assert response.status_code == expected_status, response.text
    assert "application/json" in response.headers.get("Content-Type", "")
    payload = response.json()
    assert payload.get("status") == "error", payload
    assert isinstance(payload.get("reason"), str) and payload["reason"], payload


def assert_raw_json_error(response, expected_status):
    assert response.status == expected_status, response.body.decode(errors="replace")
    assert "application/json" in dict(response.headers).get("Content-Type", "")
    payload = json.loads(response.body)
    assert payload.get("status") == "error", payload
    assert isinstance(payload.get("reason"), str) and payload["reason"], payload


def assert_raw_content_length_error(response):
    """Accept either the route JSON error or ESP-IDF's parser-level 400."""
    assert response.status == 400, response.body.decode(errors="replace")
    content_type = dict(response.headers).get("Content-Type", "")
    if "application/json" in content_type:
        payload = json.loads(response.body)
        assert payload.get("status") == "error", payload
        assert isinstance(payload.get("reason"), str) and payload["reason"], payload
    else:
        assert "text/plain" in content_type, content_type


def assert_raw_success(response):
    assert response.status == 200, response.body.decode(errors="replace")
    payload = json.loads(response.body)
    assert payload.get("status") == "success", payload


def encoded_file_route(path):
    """Encode an SD REST path while retaining only its structural slashes."""
    return f"/api/v1/sd_card/file{quote(path, safe='/')}"


def test_unique_sd_directory_file_roundtrip(api, mounted_sd, sd_write_enabled):
    dirname = f"pytest-{uuid.uuid4().hex}"
    directory_path = f"/{dirname}/"
    file_path = f"/{dirname}/roundtrip.txt"
    directory_created = False
    file_created = False
    payload = b"CAN Sniffer pytest SD roundtrip\n"

    try:
        response = api.request("POST", f"/api/v1/sd_card/file{directory_path}", data=b"")
        assert_success(response)
        directory_created = True

        response = api.request("POST", f"/api/v1/sd_card/file{file_path}", data=payload)
        assert_success(response)
        file_created = True

        response = api.request("GET", f"/api/v1/sd_card/file{file_path}")
        assert response.status_code == 200
        assert response.content == payload

        response = api.request("GET", f"/api/v1/sd_card/tree{directory_path}")
        assert_success(response)
        assert response.json().get("path") == directory_path

        response = api.request("DELETE", f"/api/v1/sd_card/file{file_path}")
        assert_success(response)
        file_created = False

        response = api.request("DELETE", f"/api/v1/sd_card/file{file_path}")
        assert_json_error(response, 404)
    finally:
        if file_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{file_path}"))
        if directory_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{directory_path}"))


def test_encoded_filename_roundtrip_through_upload_tree_read_and_delete(api, mounted_sd, sd_write_enabled):
    dirname = f"pytest-encoded-{uuid.uuid4().hex}"
    directory_path = f"/{dirname}/"
    filename = "space -_.txt"
    file_path = f"{directory_path}{filename}"
    payload = b"encoded filename roundtrip\n"
    directory_created = False
    file_created = False

    try:
        assert_success(api.request("POST", encoded_file_route(directory_path), data=b""))
        directory_created = True
        assert_success(api.request("POST", encoded_file_route(file_path), data=payload))
        file_created = True

        response = api.request("GET", f"/api/v1/sd_card/tree{quote(directory_path, safe='/')}")
        assert_success(response)
        tree = response.json()
        assert tree["path"] == directory_path
        assert any(child.get("name") == filename for child in tree["children"])

        response = api.request("GET", encoded_file_route(file_path))
        assert response.status_code == 200, response.text
        assert response.content == payload

        assert_success(api.request("DELETE", encoded_file_route(file_path)))
        file_created = False
        response = api.request("GET", encoded_file_route(file_path))
        assert response.status_code == 404, response.text
    finally:
        if file_created:
            assert_success(api.request("DELETE", encoded_file_route(file_path)))
        if directory_created:
            assert_success(api.request("DELETE", encoded_file_route(directory_path)))


def test_file_child_of_regular_file_is_not_found(api, mounted_sd, sd_write_enabled):
    file_path = f"/pytest-regular-file-{uuid.uuid4().hex}.txt"
    file_created = False

    try:
        assert_success(api.request("POST", f"/api/v1/sd_card/file{file_path}", data=b"not a directory"))
        file_created = True

        response = api.request("GET", f"/api/v1/sd_card/file{file_path}/child.txt")
        assert response.status_code == 404, response.text
    finally:
        if file_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{file_path}"))


def test_upload_accepts_explicit_zero_content_length(api, mounted_sd, raw_http_headers, sd_write_enabled):
    file_path = f"/pytest-content-length-zero-{uuid.uuid4().hex}.txt"
    file_created = False

    try:
        # Raw transport makes the explicit header distinct from an omitted one.
        response = raw_http_headers("POST", encoded_file_route(file_path), ("Content-Length: 0",))
        assert_raw_success(response)
        file_created = True

        response = api.request("GET", encoded_file_route(file_path))
        assert response.status_code == 200, response.text
        assert response.content == b""
    finally:
        if file_created:
            assert_success(api.request("DELETE", encoded_file_route(file_path)))


@pytest.mark.parametrize(
    "headers",
    [
        pytest.param((), id="missing"),
        pytest.param(("Content-Length: invalid",), id="nonnumeric"),
        pytest.param(("Content-Length: 0 0",), id="malformed"),
    ],
)
def test_upload_requires_a_valid_content_length_before_truncating(
    api, mounted_sd, raw_http_headers, sd_write_enabled, headers
):
    file_path = f"/pytest-content-length-{uuid.uuid4().hex}.txt"
    absent_file_path = f"/pytest-content-length-absent-{uuid.uuid4().hex}.txt"
    original_payload = b"must survive an invalid upload header\n"
    file_created = False

    try:
        assert_success(api.request("POST", f"/api/v1/sd_card/file{file_path}", data=original_payload))
        file_created = True

        response = raw_http_headers("POST", f"/api/v1/sd_card/file{file_path}", headers)
        assert_raw_content_length_error(response)

        response = api.request("GET", f"/api/v1/sd_card/file{file_path}")
        assert response.status_code == 200, response.text
        assert response.content == original_payload

        response = raw_http_headers("POST", f"/api/v1/sd_card/file{absent_file_path}", headers)
        assert_raw_content_length_error(response)

        response = api.request("GET", f"/api/v1/sd_card/file{absent_file_path}")
        assert response.status_code == 404, response.text
    finally:
        if file_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{file_path}"))


def test_oversized_upload_is_rejected_before_a_body_or_target_mutation(
    api, mounted_sd, raw_http_headers, sd_write_enabled
):
    file_path = f"/pytest-content-length-large-{uuid.uuid4().hex}.txt"
    original_payload = b"must survive an oversized upload declaration\n"
    file_created = False

    try:
        assert_success(api.request("POST", encoded_file_route(file_path), data=original_payload))
        file_created = True

        # Deliberately send only the request headers. A 16 MiB + 1 declaration
        # must be rejected before the handler waits for or writes any body bytes.
        response = raw_http_headers(
            "POST", encoded_file_route(file_path), ("Content-Length: 16777217",)
        )
        assert_raw_json_error(response, 413)

        response = api.request("GET", encoded_file_route(file_path))
        assert response.status_code == 200, response.text
        assert response.content == original_payload
    finally:
        if file_created:
            assert_success(api.request("DELETE", encoded_file_route(file_path)))


def test_file_serving_uses_safe_headers_mime_and_disposition(api, mounted_sd, sd_write_enabled):
    dirname = f"pytest-serving-{uuid.uuid4().hex}"
    directory_path = f"/{dirname}/"
    text_name = "report.JSON.TXT"
    fallback_name = "download name;unsafe.bin"
    files = {
        text_name: b"final extension controls MIME\n",
        fallback_name: b"unknown content\x00",
    }
    directory_created = False
    created_files = set()

    def assert_security_headers(response):
        assert response.headers.get("X-Content-Type-Options") == "nosniff"
        assert response.headers.get("Cache-Control") == "no-store"

    try:
        assert_success(api.request("POST", encoded_file_route(directory_path), data=b""))
        directory_created = True
        for filename, payload in files.items():
            file_path = f"{directory_path}{filename}"
            assert_success(api.request("POST", encoded_file_route(file_path), data=payload))
            created_files.add(file_path)

        response = api.request("GET", encoded_file_route(f"{directory_path}{text_name}"))
        assert response.status_code == 200, response.text
        assert response.content == files[text_name]
        assert_security_headers(response)
        assert "text/plain" in response.headers.get("Content-Type", "")
        assert response.headers.get("Content-Disposition") == "inline"

        text_route = encoded_file_route(f"{directory_path}{text_name}")
        response = api.request("GET", f"{text_route}?download=true")
        assert response.status_code == 200, response.text
        assert_security_headers(response)
        assert "text/plain" in response.headers.get("Content-Type", "")
        assert response.headers.get("Content-Disposition") == 'attachment; filename="report.JSON.TXT"'

        response = api.request("GET", encoded_file_route(f"{directory_path}{fallback_name}"))
        assert response.status_code == 200, response.text
        assert response.content == files[fallback_name]
        assert_security_headers(response)
        assert "application/octet-stream" in response.headers.get("Content-Type", "")
        assert response.headers.get("Content-Disposition") == 'attachment; filename="download_name_unsafe.bin"'
    finally:
        for file_path in created_files:
            assert_success(api.request("DELETE", encoded_file_route(file_path)))
        if directory_created:
            assert_success(api.request("DELETE", encoded_file_route(directory_path)))


def test_copy_file_rejects_self_copy_without_truncating_source(api, mounted_sd, sd_write_enabled):
    dirname = f"pytest-copy-self-{uuid.uuid4().hex}"
    directory_path = f"/{dirname}/"
    file_path = f"{directory_path}source.txt"
    device_file_path = f"/sdcard{file_path}"
    payload = b"self-copy must preserve this content\n"
    directory_created = False
    file_created = False

    try:
        assert_success(api.request("POST", f"/api/v1/sd_card/file{directory_path}", data=b""))
        directory_created = True
        assert_success(api.request("POST", f"/api/v1/sd_card/file{file_path}", data=payload))
        file_created = True

        case_alias_path = "/sdcard" + file_path.upper()
        for destination_path in (device_file_path, f"{device_file_path}/", case_alias_path):
            response = api.request(
                "POST",
                "/api/v1/system/copy_file",
                json={"source_path": device_file_path, "destination_path": destination_path},
            )
            assert_json_error(response, 400)

        response = api.request("GET", f"/api/v1/sd_card/file{file_path}")
        assert response.status_code == 200, response.text
        assert response.content == payload
    finally:
        if file_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{file_path}"))
        if directory_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{directory_path}"))
