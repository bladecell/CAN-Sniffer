#!/usr/bin/env python3
"""On-device check of OBD-6: add, edit and remove a PID at runtime, persist to the SD card, reload.

Usage: tools/check-pid-persistence.py [host]      (default can-sniffer.local, no auth)
Backs up /sdcard/config/pid_def.json (path from GET /obd2), always restores it byte-identical, and fails if the
restored file or the PID list differs from the original. Safe against the polling state: it does not start polling.
Uses only documented endpoints (docs/api.md); no clear_dtc, no SD format.
"""
import hashlib
import json
import sys
import urllib.error
import urllib.request

HOST = sys.argv[1] if len(sys.argv) > 1 else "can-sniffer.local"
BASE = f"http://{HOST}/api/v1"
TEST_PID = 9999


def call(method, path, body=None, raw=False, ctype="application/json"):
    data = body if raw else (None if body is None else json.dumps(body).encode())
    req = urllib.request.Request(BASE + path, data=data, method=method, headers={"Content-Type": ctype})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def jget(path):
    status, body = call("GET", path)
    assert status == 200, f"GET {path} -> {status} {body[:200]}"
    return json.loads(body)


def to_request_format(d):
    """GET /pid_def names -> PUT /pid_def names (docs/api.md, PIDDef)."""
    return {"id": d["id"], "mode": d["mode"], "pid": d["pid"], "length": d["length"], "name": d["name"],
            "unit": d["unit"], "desc": d["description"], "formula": d["formula"], "minV": d["minValue"],
            "maxV": d["maxValue"], "priority": d["priority"], "interval": d["update_interval_ms"],
            "color": d["color"], "icon": d["icon"]}


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond:
        raise SystemExit(1)


def main():
    path = jget("/obd2")["pid_def_path"]
    assert path.startswith("/sdcard/"), path
    rel = path[len("/sdcard/"):]
    status, original_file = call("GET", f"/sd_card/file/{rel}?download=true")
    check(status == 200 and len(original_file) > 0, f"backup of {path}: {len(original_file)} bytes, sha256 {hashlib.sha256(original_file).hexdigest()[:16]}")
    original = jget("/pid_def")["data"]
    check(not any(d["pid"] == TEST_PID for d in original), f"test PID {TEST_PID} is free")
    try:
        # add
        entry = {"id": 2015, "mode": 34, "pid": TEST_PID, "length": 3, "name": "ZZ_PERSIST_TEST", "unit": "x",
                 "desc": "persistence check", "formula": "A", "minV": 0, "maxV": 255, "priority": 3, "interval": 0,
                 "color": 255, "icon": "bolt"}
        status, body = call("POST", "/pid_def", [entry])
        check(status == 200 and TEST_PID in json.loads(body)["added"], "add via POST /pid_def")
        # edit (PUT replaces the whole set)
        current = [to_request_format(d) for d in jget("/pid_def")["data"]]
        for d in current:
            if d["pid"] == TEST_PID:
                d["name"], d["formula"] = "ZZ_PERSIST_TEST_EDIT", "A+1"
        status, body = call("PUT", "/pid_def", current)
        check(status == 204, f"edit via PUT /pid_def (status {status} {body[:100]})")
        # save and inspect the file
        status, _ = call("POST", "/pid_def/save")
        check(status == 200, "POST /pid_def/save")
        status, saved = call("GET", f"/sd_card/file/{rel}?download=true")
        saved_list = json.loads(saved)
        mine = [d for d in saved_list if d["pid"] == TEST_PID]
        check(len(mine) == 1 and mine[0]["name"] == "ZZ_PERSIST_TEST_EDIT" and mine[0]["formula"] == "A+1",
              "saved file contains the edited PID")
        check(len(saved_list) == len(original) + 1, f"saved file has {len(saved_list)} PIDs (original {len(original)} + 1)")
        # reload from the file
        status, _ = call("POST", "/pid_def/load")
        check(status == 200, "POST /pid_def/load")
        reloaded = {d["pid"]: d for d in jget("/pid_def")["data"]}
        check(TEST_PID in reloaded and reloaded[TEST_PID]["name"] == "ZZ_PERSIST_TEST_EDIT", "edit survived the reload")
        # remove, save
        status, _ = call("DELETE", f"/pid_def/{TEST_PID}")
        check(status == 200, "remove via DELETE /pid_def/<pid>")
        status, _ = call("POST", "/pid_def/save")
        status, saved = call("GET", f"/sd_card/file/{rel}?download=true")
        check(all(d["pid"] != TEST_PID for d in json.loads(saved)), "saved file no longer contains the test PID")
    finally:
        # restore the user's file byte for byte, then reload it
        status, _ = call("POST", f"/sd_card/file/{rel}", original_file, raw=True, ctype="application/octet-stream")
        print(f"restore upload -> {status}")
        call("POST", "/pid_def/load")
    status, after_file = call("GET", f"/sd_card/file/{rel}?download=true")
    check(after_file == original_file, "restored file is byte-identical")
    after = jget("/pid_def")["data"]
    check(after == original, f"GET /pid_def equals the original ({len(after)} PIDs)")
    print("OBD-6 persistence check passed")


if __name__ == "__main__":
    main()
