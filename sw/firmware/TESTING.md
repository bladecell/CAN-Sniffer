# Firmware REST API tests

Install the test dependencies and run the read-only integration suite:

```sh
python -m pip install -r requirements-test.txt
pytest -m "integration and not sd_write and not sd_mutating_security"
```

The target defaults to `http://can-sniffer.local`. Override it when needed:

```sh
CAN_SNIFFER_URL=http://192.168.4.1 pytest -m "integration and not sd_write and not sd_mutating_security"
```

Tests skip clearly when the device or SD card is unavailable. Every request has
bounded connect/read timeouts, ignores proxy environment variables, does not
follow redirects, and is paced at no more than 20 requests/second. HTTP 429 is
reported immediately rather than retried in a burst. The default suite is
strictly read-only; it never invokes POST or DELETE probes, reboot, format,
settings/PID mutation, or DTC clearing endpoints.

The temporary SD roundtrip is intentionally double-gated. It creates a unique
directory, verifies one file, and removes both in a `finally` block:

```sh
CAN_SNIFFER_ENABLE_SD_WRITE=1 pytest --run-sd-write -m sd_write
```

GET-only path-security checks run by default and continue to require HTTP 400.
They will fail against older firmware, which is useful evidence that the target
must be reflashed. Potentially destructive POST/DELETE security probes include
root deletion cases and **must only be run after hardened firmware is flashed**.
They have a separate double opt-in:

```sh
CAN_SNIFFER_ENABLE_SD_WRITE=1 pytest --run-mutating-sd-security -m sd_mutating_security
```
