"""POST /api/v1/pid_def per-item validation (string caps, types, empty formula, duplicates).

Runs against a real board with a live PID set, so it only adds derived PIDs (mode 0x45, length 0:
no CAN request) at numbers that are free at test time, never PUTs the set, and deletes every PID
it may have added in the fixture teardown.
"""
import json

import pytest


pytestmark = pytest.mark.integration

PID_DEF_ROUTE = "/api/v1/pid_def"
POST_BODY_CAP = 2048
INVALID_ARG = "ESP_ERR_INVALID_ARG"
CAPS = {"name": 64, "unit": 32, "desc": 256, "formula": 1024, "icon": 64}
PID_BASE = 61000  # derived PIDs are arbitrary numbers; scan upward for free ones
PID_POOL = 4


def definition(pid, **overrides):
    item = {
        "id": 2015,
        "mode": 0x45,
        "pid": pid,
        "length": 0,
        "name": "pytest post",
        "unit": "",
        "desc": "",
        "formula": "1",
        "interval": 1000,
    }
    item.update(overrides)
    return item


def get_pid_snapshot(api):
    response = api.request("GET", PID_DEF_ROUTE)
    assert response.status_code == 200, response.text
    snapshot = response.json()
    assert isinstance(snapshot.get("data"), list)
    assert snapshot["count"] == len(snapshot["data"])
    return snapshot


def post_items(api, items):
    body = json.dumps(items).encode()
    assert len(body) <= POST_BODY_CAP, f"test body {len(body)} bytes exceeds the {POST_BODY_CAP} byte cap"
    return api.request("POST", PID_DEF_ROUTE, data=body, headers={"Content-Type": "application/json"})


def delete_pid(api, pid):
    response = api.request("DELETE", f"{PID_DEF_ROUTE}/{pid}")
    assert response.status_code in (204, 404), (pid, response.status_code, response.text)


def free_pids(existing, count=PID_POOL, base=PID_BASE):
    taken = set(existing)
    found = []
    pid = base
    while len(found) < count and pid <= 0xFFFF:
        if pid not in taken:
            found.append(pid)
        pid += 1
    assert len(found) == count, "no free PID numbers for the test"
    return found


@pytest.fixture
def sandbox(auth_api, device_reachable, pid_def_write_enabled):
    """Yield (pids, count_before). Teardown deletes every pool PID that exists, then checks the count."""
    before = get_pid_snapshot(auth_api)
    pids = free_pids(d["pid"] for d in before["data"])
    try:
        yield pids, before["count"]
    finally:
        try:
            present = {d["pid"] for d in get_pid_snapshot(auth_api)["data"]}
            for pid in pids:
                if pid in present:
                    delete_pid(auth_api, pid)
        finally:
            after = get_pid_snapshot(auth_api)
            assert after["count"] == before["count"], "test left PIDs behind"
            assert [d["pid"] for d in after["data"]] == [d["pid"] for d in before["data"]]


def assert_all_rejected(api, items, expected_pids, count_before):
    response = post_items(api, items)
    assert response.status_code == 422, (response.status_code, response.text)
    payload = response.json()
    assert payload["status"] == "error", payload
    assert payload["added"] == [], payload
    assert [f["error"] for f in payload["failed"]] == [INVALID_ARG] * len(items), payload
    assert [f["pid"] for f in payload["failed"]] == expected_pids, payload
    assert get_pid_snapshot(api)["count"] == count_before


def over_cap(field):
    return "x" * (CAPS[field] + 1)


OVER_CAP_CASES = tuple(pytest.param(field, id=f"{field}-over-cap") for field in CAPS)

NON_STRING_CASES = tuple(
    pytest.param(field, value, id=f"{field}-{type(value).__name__}")
    for field in ("name", "unit", "desc", "formula", "icon")
    for value in (5, ["a"], {"a": 1})
) + (pytest.param("name", None, id="name-null"), pytest.param("formula", None, id="formula-null"))


@pytest.mark.pid_def_write
@pytest.mark.parametrize("field", OVER_CAP_CASES)
def test_pid_def_post_rejects_string_over_cap(auth_api, sandbox, field):
    pids, count_before = sandbox
    assert_all_rejected(auth_api, [definition(pids[0], **{field: over_cap(field)})], [pids[0]], count_before)


@pytest.mark.pid_def_write
def test_pid_def_post_rejects_empty_formula(auth_api, sandbox):
    pids, count_before = sandbox
    assert_all_rejected(auth_api, [definition(pids[0], formula="")], [pids[0]], count_before)


@pytest.mark.pid_def_write
@pytest.mark.parametrize("field, value", NON_STRING_CASES)
def test_pid_def_post_rejects_non_string_field(auth_api, sandbox, field, value):
    pids, count_before = sandbox
    assert_all_rejected(auth_api, [definition(pids[0], **{field: value})], [pids[0]], count_before)


@pytest.mark.pid_def_write
def test_pid_def_post_invalid_items_in_one_batch_all_fail_without_change(auth_api, sandbox):
    pids, count_before = sandbox
    items = [
        definition(pids[0], unit=over_cap("unit")),
        definition(pids[1], formula=""),
        definition(pids[2], desc=7),
    ]
    assert_all_rejected(auth_api, items, pids[:3], count_before)


@pytest.mark.pid_def_write
def test_pid_def_post_accepts_exact_cap_strings(auth_api, sandbox):
    pids, count_before = sandbox
    exact = {field: "x" * cap for field, cap in CAPS.items()}
    exact["formula"] = " " * (CAPS["formula"] - 1) + "1"
    response = post_items(auth_api, [definition(pids[0], **exact)])
    assert response.status_code == 200, (response.status_code, response.text)
    payload = response.json()
    assert payload["status"] == "success", payload
    assert payload["added"] == [pids[0]] and payload["failed"] == [], payload

    snapshot = get_pid_snapshot(auth_api)
    assert snapshot["count"] == count_before + 1
    stored = next(d for d in snapshot["data"] if d["pid"] == pids[0])
    assert stored["name"] == exact["name"]
    assert stored["unit"] == exact["unit"]
    assert stored["description"] == exact["desc"]
    assert stored["icon"] == exact["icon"]
    assert len(stored["formula"]) == CAPS["formula"]


@pytest.mark.pid_def_write
def test_pid_def_post_optional_strings_may_be_null_or_absent(auth_api, sandbox):
    pids, count_before = sandbox
    items = [
        definition(pids[0], unit=None, desc=None, icon=None),
        {k: v for k, v in definition(pids[1]).items() if k not in ("unit", "desc", "icon")},
    ]
    response = post_items(auth_api, items)
    assert response.status_code == 200, (response.status_code, response.text)
    payload = response.json()
    assert payload["status"] == "success", payload
    assert payload["added"] == pids[:2] and payload["failed"] == [], payload
    assert get_pid_snapshot(auth_api)["count"] == count_before + 2


@pytest.mark.pid_def_write
def test_pid_def_post_duplicate_pid_in_one_batch_adds_first_only(auth_api, sandbox):
    pids, count_before = sandbox
    items = [definition(pids[0], name="first"), definition(pids[0], name="second")]
    response = post_items(auth_api, items)
    assert response.status_code == 200, (response.status_code, response.text)
    payload = response.json()
    assert payload["status"] == "partial_success", payload
    assert payload["added"] == [pids[0]], payload
    assert len(payload["failed"]) == 1 and payload["failed"][0]["pid"] == pids[0], payload
    assert payload["failed"][0]["error"].startswith("ESP_ERR_"), payload

    snapshot = get_pid_snapshot(auth_api)
    assert snapshot["count"] == count_before + 1
    stored = [d for d in snapshot["data"] if d["pid"] == pids[0]]
    assert len(stored) == 1 and stored[0]["name"] == "first"


@pytest.mark.pid_def_write
def test_pid_def_post_mixed_batch_keeps_valid_item(auth_api, sandbox):
    pids, count_before = sandbox
    items = [definition(pids[0], name=over_cap("name")), definition(pids[1])]
    response = post_items(auth_api, items)
    assert response.status_code == 200, (response.status_code, response.text)
    payload = response.json()
    assert payload["status"] == "partial_success", payload
    assert payload["added"] == [pids[1]], payload
    assert payload["failed"] == [{"pid": pids[0], "error": INVALID_ARG}], payload
    assert get_pid_snapshot(auth_api)["count"] == count_before + 1
