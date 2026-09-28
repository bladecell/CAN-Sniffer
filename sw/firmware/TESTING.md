# Firmware REST API tests

Install the test dependencies and run the read-only integration suite:

```sh
python -m pip install -r requirements-test.txt
pytest -m "integration and not sd_write and not sd_mutating_security and not pid_def_write"
```

The target defaults to `http://can-sniffer.local`. Override it when needed:

```sh
CAN_SNIFFER_URL=http://192.168.4.1 pytest -m "integration and not sd_write and not sd_mutating_security and not pid_def_write"
```

Tests skip clearly when the device or SD card is unavailable. Every request has
bounded connect/read timeouts, ignores proxy environment variables, does not
follow redirects, and is paced at no more than 20 requests/second. HTTP 429 is
reported immediately rather than retried in a burst. The default suite is
strictly read-only; it never invokes POST or DELETE probes, reboot, format,
settings/PID mutation, or DTC clearing endpoints.

The WebSocket PID-stream smoke test is opt-in because it may temporarily start
continuous polling. When enabled, it reads `continuous_running` from
`/api/v1/obd2`, starts polling only if it was stopped, and restores that exact
state in a `finally` block (including skips and errors). It skips without
changing polling if that state cannot be read safely:

```sh
CAN_SNIFFER_ENABLE_WEBSOCKET_SMOKE=1 pytest tests/rest_api/test_websocket.py
```

The JSON parser tests send malformed, incomplete, and concatenated/trailing
JSON bodies to a validation-only path. They also send a valid copy-file JSON
object in three deliberate TCP body writes. Its paths are outside `/sdcard`, so
middleware rejects it before any filesystem operation; the test therefore
checks successful body assembly/parsing without mutating device state.

The SD hardening suite is intentionally double-gated and requires a reachable
device with a mounted card. It creates only unique `pytest-*` paths and removes
them in `finally` blocks. It covers encoded filename upload/tree/read/delete,
explicit/missing/malformed upload lengths, header-only 16 MiB + 1 rejection,
file-serving MIME/disposition/security headers, regular-file child rejection,
and self-copy preservation. The separate GET-only path-security suite also
covers strict download-query parsing (valid literal forms and rejected ambiguous
forms):

Malformed `Content-Length` values may be rejected by the ESP-IDF HTTP parser
before the upload route runs, producing a plain-text 400; the test only checks
route-level JSON when that request reaches the route.

```sh
CAN_SNIFFER_ENABLE_SD_WRITE=1 pytest --run-sd-write -m sd_write
```

PID-definition mutation-capable tests are separately double-gated. Every test
that sends a PID-definition `PUT`, including rejected-payload atomicity checks,
requires both the `pid_def_write` marker gate and
`CAN_SNIFFER_ENABLE_PID_DEF_WRITE=1`. They snapshot the complete current
definition set and restore it in a `finally` block when needed; the replacement
test temporarily clears it. The save/load persistence test also requires the SD-write
gate: it uses a unique temporary SD directory/file through the REST API, checks
the saved JSON, loads legacy `len` data, and checks that conflicting aliases do
not change the active set. It also verifies truncated and trailing-corrupt PID
files are rejected while the active definition set is preserved. It never uses
the configured default path and removes its temporary files in a `finally`
block. These tests must only be run against a device where that interruption is
acceptable:

```sh
CAN_SNIFFER_ENABLE_PID_DEF_WRITE=1 CAN_SNIFFER_ENABLE_SD_WRITE=1 \
  pytest --run-pid-def-write --run-sd-write -m pid_def_write
```

GET-only path-security checks run by default and continue to require HTTP 400.
They will fail against older firmware, which is useful evidence that the target
must be reflashed. Potentially destructive POST/DELETE security probes include
root deletion cases and **must only be run after hardened firmware is flashed**.
They have a separate double opt-in:

```sh
CAN_SNIFFER_ENABLE_SD_WRITE=1 pytest --run-mutating-sd-security -m sd_mutating_security
```

The integration harness intentionally does **not** attempt time-based SD-lock
contention, physical card removal during an operation, or injected file-close
failures. Those require timing, hardware, or fault-injection control that the
raw HTTP/device-card tests cannot provide deterministically.
