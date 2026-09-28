import json
import uuid

import pytest


pytestmark = pytest.mark.integration

PID_DEF_ROUTE = "/api/v1/pid_def"


def valid_definition(pid=4, definition_id=2015):
    return {
        "id": definition_id,
        "mode": 1,
        "pid": pid,
        "len": 2,
        "name": "pytest PID",
        "unit": "%",
        "desc": "temporary test definition",
        "formula": "A",
        "minV": 0.0,
        "maxV": 100.0,
        "priority": 0,
        "interval": 0,
        "color": 0,
        "icon": "",
    }


def json_body(payload):
    return json.dumps(payload).encode()


def get_pid_definitions(api):
    response = api.request("GET", PID_DEF_ROUTE)
    assert response.status_code == 200, response.text
    snapshot = response.json()
    assert isinstance(snapshot, dict)
    assert isinstance(snapshot.get("data"), list)
    assert isinstance(snapshot.get("count"), int)
    assert snapshot["count"] == len(snapshot["data"])
    return snapshot


INVALID_PID_DEF_PAYLOADS = (
    pytest.param(b"[", 400, id="malformed-json"),
    pytest.param(json_body({}), 400, id="top-level-object"),
    pytest.param(
        json_body(
            [valid_definition(), {**valid_definition(pid=5, definition_id=2016), "len": 0}]
        ),
        422,
        id="valid-first-invalid-second",
    ),
    pytest.param(
        json_body([valid_definition(), valid_definition(definition_id=2016)]),
        422,
        id="duplicate-pid",
    ),
)


def restore_pid_definitions(api, snapshot):
    current = get_pid_definitions(api)
    if current != snapshot:
        assert_successful_put(
            api.request(
                "PUT",
                PID_DEF_ROUTE,
                json=[to_put_definition(definition, "length") for definition in snapshot["data"]],
            )
        )


@pytest.mark.pid_def_write
@pytest.mark.parametrize("body, expected_status", INVALID_PID_DEF_PAYLOADS)
def test_invalid_pid_def_put_is_atomic(
    api, device_reachable, pid_def_write_enabled, body, expected_status
):
    before = get_pid_definitions(api)

    try:
        response = api.request(
            "PUT",
            PID_DEF_ROUTE,
            data=body,
            headers={"Content-Type": "application/json"},
        )
        assert response.status_code == expected_status, (response.status_code, response.text)

        after = get_pid_definitions(api)
        assert after == before
    finally:
        restore_pid_definitions(api, before)


@pytest.mark.pid_def_write
def test_conflicting_length_aliases_are_rejected_atomically(
    api, device_reachable, pid_def_write_enabled
):
    before = get_pid_definitions(api)
    conflicting = valid_definition()
    conflicting["length"] = 2
    conflicting["len"] = 3

    try:
        response = api.request(
            "PUT",
            PID_DEF_ROUTE,
            json=[conflicting],
            headers={"Content-Type": "application/json"},
        )
        assert response.status_code == 422, response.text
        assert get_pid_definitions(api) == before
    finally:
        restore_pid_definitions(api, before)


def to_put_definition(definition, length_key="len"):
    payload = {
        "id": definition["id"],
        "mode": definition["mode"],
        "pid": definition["pid"],
        "name": definition["name"],
        "unit": definition["unit"],
        "desc": definition["description"],
        "formula": definition["formula"],
        "minV": definition["minValue"],
        "maxV": definition["maxValue"],
        "priority": definition["priority"],
        "interval": definition["update_interval_ms"],
        "color": definition["color"],
        "icon": definition["icon"],
    }
    payload[length_key] = definition["length"]
    return payload


def storage_definition_to_get_shape(definition):
    """Translate the PID save-file aliases to the normalized GET response schema."""
    normalized = dict(definition)
    for storage_name, get_name in {
        "desc": "description",
        "minV": "minValue",
        "maxV": "maxValue",
        "interval": "update_interval_ms",
    }.items():
        normalized[get_name] = normalized.pop(storage_name)
    return normalized


def assert_successful_put(response):
    assert response.status_code == 204, response.text
    assert response.content == b""


def assert_successful_sd_upload(response):
    assert response.status_code == 200, response.text
    assert response.json().get("status") == "success", response.text


@pytest.mark.pid_def_write
def test_pid_def_put_accepts_canonical_length_and_legacy_len(
    api, device_reachable, pid_def_write_enabled
):
    snapshot = get_pid_definitions(api)

    try:
        canonical = [to_put_definition(definition, "length") for definition in snapshot["data"]]
        assert_successful_put(api.request("PUT", PID_DEF_ROUTE, json=canonical))
        assert get_pid_definitions(api) == snapshot

        legacy = [to_put_definition(definition, "len") for definition in snapshot["data"]]
        assert_successful_put(api.request("PUT", PID_DEF_ROUTE, json=legacy))
        assert get_pid_definitions(api) == snapshot
    finally:
        restore_pid_definitions(api, snapshot)

    assert get_pid_definitions(api) == snapshot


@pytest.mark.pid_def_write
def test_invalid_pid_def_save_load_body_does_not_use_default_operation(
    api, device_reachable, pid_def_write_enabled
):
    snapshot = get_pid_definitions(api)

    try:
        for endpoint in ("/api/v1/pid_def/save", "/api/v1/pid_def/load"):
            response = api.request(
                "POST",
                endpoint,
                data=b"[",
                headers={"Content-Type": "application/json"},
            )
            assert response.status_code == 400, (endpoint, response.status_code, response.text)
    finally:
        current = get_pid_definitions(api)
        if current != snapshot:
            response = api.request(
                "PUT",
                PID_DEF_ROUTE,
                json=[to_put_definition(definition) for definition in snapshot["data"]],
                headers={"Content-Type": "application/json"},
            )
            assert_successful_put(response)

    assert get_pid_definitions(api) == snapshot


@pytest.mark.pid_def_write
def test_pid_def_put_empty_clear_restores_snapshot(
    api, device_reachable, pid_def_write_enabled
):
    snapshot = get_pid_definitions(api)
    restore_payload = [to_put_definition(definition) for definition in snapshot["data"]]

    try:
        response = api.request(
            "PUT",
            PID_DEF_ROUTE,
            json=[],
            headers={"Content-Type": "application/json"},
        )
        assert_successful_put(response)
    finally:
        response = api.request(
            "PUT",
            PID_DEF_ROUTE,
            json=restore_payload,
            headers={"Content-Type": "application/json"},
        )
        assert_successful_put(response)

    assert get_pid_definitions(api) == snapshot


@pytest.mark.pid_def_write
@pytest.mark.sd_write
def test_pid_def_save_load_persists_canonical_length_and_accepts_legacy_len(
    api, device_reachable, mounted_sd, pid_def_write_enabled, sd_write_enabled
):
    snapshot = get_pid_definitions(api)
    directory_name = f"pytest-pid-def-{uuid.uuid4().hex}"
    directory_path = f"/{directory_name}/"
    saved_path = f"{directory_path}saved.json"
    legacy_path = f"{directory_path}legacy.json"
    conflicting_path = f"{directory_path}conflicting.json"
    corrupt_paths = {
        f"{directory_path}truncated.json": b"[",
        f"{directory_path}trailing.json": b"[] trailing",
    }
    device_saved_path = f"/sdcard{saved_path}"
    device_legacy_path = f"/sdcard{legacy_path}"
    device_conflicting_path = f"/sdcard{conflicting_path}"
    created_files = set()
    directory_created = False

    temporary = valid_definition(pid=4, definition_id=2015)
    temporary["length"] = temporary.pop("len")
    legacy = dict(temporary)
    legacy["len"] = legacy.pop("length")
    conflicting = dict(temporary)
    conflicting["len"] = conflicting["length"] + 1

    try:
        response = api.request("POST", f"/api/v1/sd_card/file{directory_path}", data=b"")
        assert_successful_sd_upload(response)
        directory_created = True

        response = api.request(
            "PUT",
            PID_DEF_ROUTE,
            json=[temporary],
            headers={"Content-Type": "application/json"},
        )
        assert_successful_put(response)
        expected = get_pid_definitions(api)
        assert expected["data"], "temporary definition setup unexpectedly produced an empty set"

        response = api.request(
            "POST",
            "/api/v1/pid_def/save",
            json={"pid_def_path": device_saved_path},
        )
        assert response.status_code == 200, response.text
        assert response.json().get("status") == "success", response.text
        created_files.add(saved_path)

        response = api.request("GET", f"/api/v1/sd_card/file{saved_path}")
        assert response.status_code == 200, response.text
        persisted = json.loads(response.content)
        assert isinstance(persisted, list) and persisted
        assert all("length" in definition for definition in persisted)
        assert all("len" not in definition for definition in persisted)
        assert all(
            {"desc", "minV", "maxV", "interval"} <= definition.keys()
            for definition in persisted
        )
        assert all(
            not {"description", "minValue", "maxValue", "update_interval_ms"}
            & definition.keys()
            for definition in persisted
        )
        assert [storage_definition_to_get_shape(definition) for definition in persisted] == expected["data"]

        response = api.request(
            "POST", f"/api/v1/sd_card/file{legacy_path}", data=json_body([legacy])
        )
        assert_successful_sd_upload(response)
        created_files.add(legacy_path)
        assert "len" in legacy and "length" not in legacy
        response = api.request(
            "POST", "/api/v1/pid_def/load", json={"pid_def_path": device_legacy_path}
        )
        assert response.status_code == 200, response.text
        assert response.json().get("status") == "success", response.text
        assert get_pid_definitions(api) == expected

        response = api.request(
            "POST", f"/api/v1/sd_card/file{conflicting_path}", data=json_body([conflicting])
        )
        assert_successful_sd_upload(response)
        created_files.add(conflicting_path)
        response = api.request(
            "POST", "/api/v1/pid_def/load", json={"pid_def_path": device_conflicting_path}
        )
        assert response.status_code == 400, response.text
        assert get_pid_definitions(api) == expected

        for corrupt_path, corrupt_body in corrupt_paths.items():
            response = api.request("POST", f"/api/v1/sd_card/file{corrupt_path}", data=corrupt_body)
            assert_successful_sd_upload(response)
            created_files.add(corrupt_path)

            response = api.request(
                "POST", "/api/v1/pid_def/load", json={"pid_def_path": f"/sdcard{corrupt_path}"}
            )
            assert response.status_code == 400, response.text
            assert get_pid_definitions(api) == expected
    finally:
        try:
            response = api.request(
                "PUT",
                PID_DEF_ROUTE,
                json=[to_put_definition(definition, "length") for definition in snapshot["data"]],
                headers={"Content-Type": "application/json"},
            )
            assert_successful_put(response)
        finally:
            for file_path in created_files:
                response = api.request("DELETE", f"/api/v1/sd_card/file{file_path}")
                assert response.status_code == 200, response.text
            if directory_created:
                response = api.request("DELETE", f"/api/v1/sd_card/file{directory_path}")
                assert response.status_code == 200, response.text

    assert get_pid_definitions(api) == snapshot
