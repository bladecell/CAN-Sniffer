import json

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


@pytest.mark.parametrize("body, expected_status", INVALID_PID_DEF_PAYLOADS)
def test_invalid_pid_def_put_is_atomic(api, device_reachable, body, expected_status):
    before = get_pid_definitions(api)

    response = api.request(
        "PUT",
        PID_DEF_ROUTE,
        data=body,
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == expected_status, (response.status_code, response.text)

    after = get_pid_definitions(api)
    assert after == before


def to_put_definition(definition):
    return {
        "id": definition["id"],
        "mode": definition["mode"],
        "pid": definition["pid"],
        "len": definition["length"],
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


def assert_successful_put(response):
    assert response.status_code == 204, response.text
    assert response.content == b""


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
