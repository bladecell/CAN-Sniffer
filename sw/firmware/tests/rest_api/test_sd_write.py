import uuid

import pytest


pytestmark = [pytest.mark.integration, pytest.mark.sd_write]


def assert_success(response):
    assert response.status_code == 200, response.text
    assert response.json().get("status") == "success", response.text


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
    finally:
        if file_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{file_path}"))
        if directory_created:
            assert_success(api.request("DELETE", f"/api/v1/sd_card/file{directory_path}"))
