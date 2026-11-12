#!/usr/bin/env python3
"""Load test for the async web server (WEB-1).

Usage: tools/load-test.py [host] [legacy options]
       tools/load-test.py --mode read-only --base-url http://HOST --simulator-no-car
         --handshake-schedule compare --handshake-only --stagger-ms 250
The firmware serves ONE WebSocket session (docs/api.md, decisions.md 2026-11-11): /ws?sid=<1-8 hex, nonzero>
[&takeover=1]. A second sid while the holder is live is upgraded, then closed with code 4000; takeover=1 replaces
the holder (old one gets 4001); a holder silent for 10 s is replaced.
Read-only mode starts ONE steady WS client with its own sid (it asks for the PID stream with 0xA0 and pings every
2 s), then runs --ws handshake probes (default 2) with other sids and checks each is refused with close code 4000
(reported as refused=N). It then records the client's lifecycle and data rates while stepped HTTP levels run. The
load phase is skipped unless the steady client is live. --takeover makes the steady client replace a current
session (that disconnects any open web page); without it a busy device is reported and the run stops.
--drain-seconds inserts a bounded pause after the probes and before the HTTP load phase (default 0).
The tool never sends the 0xA1 stop command. The default legacy mode retains its historical PATCH /obd2 polling
behavior and also uses one WS client.
Per level it prints success / 429 / 503 / other-status / timeout / error counts, latency p50 and p95, the WebSocket
packet rate, and heap and the lowest task stack from GET /system (heap_free, heap_min_free, tasks).
Never calls clear_dtc, SD format or delete, OTA, or any pid_def write.
"""
import argparse
import base64
import hashlib
import hmac
import http.client
import json
import math
import os
import re
import socket
import ssl
import statistics
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any
from urllib.parse import urlsplit

ENDPOINTS = ["/api/v1/system", "/api/v1/obd2", "/api/v1/pid_def", "/api/v1/pid_data", "/api/v1/dtc",
             "/api/v1/sd_card/info"]
LARGE = "/"  # gzipped frontend, the biggest read-only response
READ_ONLY_METHODS = {"GET", "HEAD"}
MAX_READ_ONLY_TIMEOUT = 60.0
MAX_DIAGNOSTIC_BYTES = 16 * 1024
DEFAULT_DRAIN_SECONDS = 0.0
MAX_DRAIN_SECONDS = 30.0


@dataclass(frozen=True)
class HttpResult:
    status: int | str
    elapsed: float


@dataclass
class WsClientStats:
    client_id: str
    role: str
    sid: str = ""
    close_code: int | None = None
    handshake_status: str = "pending"
    handshake_valid: bool = False
    handshake_elapsed_s: float = 0.0
    exception_class: str = ""
    exception_message: str = ""
    worker_still_running: bool = False
    connected: bool = False
    connected_at: float | None = None
    connected_duration_s: float = 0.0
    disconnect_reason: str = "pending"
    pid_packets: int = 0
    pongs: int = 0
    transport: socket.socket | None = field(default=None, repr=False)
    handshake_complete: threading.Event = field(default_factory=threading.Event, repr=False)
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)


def sanitize_diagnostic_text(value: str) -> str:
    value = re.sub(r"https?://\S+", "<url>", value, flags=re.IGNORECASE)
    value = re.sub(r"(?i)\b(authorization|proxy-authorization)\s*[:=]\s*.*$",
                   r"\1=<redacted>", value)
    value = re.sub(r"(?i)\b(bearer|basic)\s+\S+", r"\1 <redacted>", value)
    value = re.sub(r"(?i)\b(password|passwd|token|pin)([=: ]+)\S+", r"\1\2<redacted>", value)
    return " ".join(value.replace("\r", " ").replace("\n", " ").split())[:160]


def handshake_failure(client_id: str, base_url: str, error: Exception) -> WsClientStats:
    del base_url  # Never include the target URL in diagnostics.
    return WsClientStats(client_id=client_id, role="handshake-probe",
                         handshake_status="exception", exception_class=type(error).__name__,
                         exception_message=sanitize_diagnostic_text(str(error)))


def validate_websocket_handshake(response: bytes, key: str) -> str:
    header, separator, _rest = response.partition(b"\r\n\r\n")
    if not separator:
        raise ValueError("incomplete WebSocket handshake headers")
    lines = header.split(b"\r\n")
    status_line = lines[0].decode("latin-1", errors="replace")
    status_parts = status_line.split(" ", 2)
    if len(status_parts) < 2 or status_parts[0] != "HTTP/1.1" or status_parts[1] != "101":
        raise ValueError("expected HTTP/1.1 status 101")

    headers: dict[str, list[str]] = {}
    for line in lines[1:]:
        name, colon, value = line.partition(b":")
        if not colon:
            raise ValueError("malformed WebSocket handshake header")
        header_name = name.decode("ascii", errors="strict").lower()
        headers.setdefault(header_name, []).append(value.decode("latin-1").strip())

    upgrades = headers.get("upgrade", [])
    connections = headers.get("connection", [])
    accepts = headers.get("sec-websocket-accept", [])
    if len(upgrades) != 1 or "websocket" not in {token.strip().lower() for token in upgrades[0].split(",")}:
        raise ValueError("missing WebSocket Upgrade token")
    if len(connections) != 1 or "upgrade" not in {token.strip().lower() for token in connections[0].split(",")}:
        raise ValueError("missing Connection Upgrade token")
    expected = base64.b64encode(hashlib.sha1(
        (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
    ).digest()).decode("ascii")
    if len(accepts) != 1 or not hmac.compare_digest(accepts[0], expected):
        raise ValueError("Sec-WebSocket-Accept does not match client key")
    return status_line


def new_sid(exclude: set[str] | None = None) -> str:
    """Random session id: 1-8 hex digits, never zero."""
    while True:
        sid = format(int.from_bytes(os.urandom(4), "big"), "x")
        if int(sid, 16) and sid not in (exclude or set()):
            return sid


def ws_path(sid: str, takeover: bool = False) -> str:
    return f"/ws?sid={sid}" + ("&takeover=1" if takeover else "")


def parse_ws_frame(buffer: bytes) -> tuple[int, bytes, bytes] | None:
    """Split one unmasked server frame off buffer: (opcode, payload, rest), or None when incomplete."""
    if len(buffer) < 2:
        return None
    size, offset = buffer[1] & 0x7f, 2
    if size == 126:
        if len(buffer) < 4:
            return None
        size, offset = struct.unpack(">H", buffer[2:4])[0], 4
    elif size == 127:
        raise ValueError("oversized WebSocket frame")
    if len(buffer) < offset + size:
        return None
    return buffer[0] & 0x0f, buffer[offset:offset + size], buffer[offset + size:]


def close_code_of(payload: bytes) -> int | None:
    return int.from_bytes(payload[:2], "big") if len(payload) >= 2 else None


def read_close_code(sock: socket.socket, deadline: float) -> int | None:
    """Wait for the server's close frame after an upgrade; None on EOF, timeout or no close code."""
    buffer = b""
    try:
        while True:
            parsed = parse_ws_frame(buffer)
            if parsed is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                sock.settimeout(remaining)
                part = sock.recv(4096)
                if not part:
                    return None
                buffer += part
                continue
            opcode, payload, buffer = parsed
            if opcode == 8:
                return close_code_of(payload)
    except (OSError, ValueError):
        return None


def record_ws_payload(client: WsClientStats, payload: bytes) -> None:
    with client.lock:
        if payload[:1] == b"\x02" and len(payload) == 20:
            client.pid_packets += 1
        elif payload == b"\xa3":
            client.pongs += 1


def ws_ready_live_counts(clients: list[WsClientStats]) -> tuple[int, int]:
    snapshots = []
    for client in clients:
        with client.lock:
            snapshots.append((client.handshake_valid, client.connected))
    ready = sum(valid for valid, _ in snapshots)
    live = sum(connected for _, connected in snapshots)
    return ready, live


def format_ws_diagnostics(
    clients: list[WsClientStats], duration: float,
    before: dict[str, tuple[int, int]] | None = None,
    counts: tuple[int, int] | None = None,
) -> str:
    elapsed = max(duration, 0.001)
    ready, live = counts if counts is not None else ws_ready_live_counts(clients)
    rows = []
    for client in clients:
        with client.lock:
            connected_duration = client.connected_duration_s
            if client.connected and client.connected_at is not None:
                connected_duration = max(0.0, time.monotonic() - client.connected_at)
            prior_packets, prior_pongs = (before or {}).get(client.client_id, (0, 0))
            packets = client.pid_packets - prior_packets
            pongs = client.pongs - prior_pongs
            details = (f"{client.role} id={client.client_id} status={client.handshake_status} "
                       f"handshake_s={client.handshake_elapsed_s:.3f} "
                       f"connected_s={connected_duration:.3f} connected={client.connected} "
                       f"disconnect={client.disconnect_reason} "
                       f"close={client.close_code if client.close_code is not None else '-'} pid={packets} pid_rate={packets / elapsed:.1f}/s "
                       f"pong={pongs} pong_rate={pongs / elapsed:.1f}/s")
            if client.exception_class:
                details += f" exception={client.exception_class}: {client.exception_message}"
            if client.worker_still_running:
                details += " cleanup=worker_still_running"
            rows.append(details)
    expected = (1 if any(client.role == "steady-data" for client in clients)
                else sum(client.role == "handshake-probe" for client in clients))
    probes = [client for client in clients if client.role == "handshake-probe"]
    refused = f" refused={sum(client.close_code == 4000 for client in probes)}/{len(probes)}" if probes else ""
    return f"ready={ready} live={live} expected={expected}{refused}; " + " | ".join(rows)


def validate_read_only_bounds(duration: int, concurrency: int) -> None:
    if not 1 <= duration <= 3600 or not 1 <= concurrency <= 64:
        raise ValueError("duration must be 1..3600 s and concurrency 1..64")


def validate_operator_confirmation(confirmed: bool, target: str) -> None:
    if not confirmed or not target:
        raise ValueError("read-only mode requires --simulator-no-car and an explicit base URL")
    try:
        parsed = urlsplit(target)
        port = parsed.port
    except ValueError as error:
        raise ValueError("base URL must be a valid http(s) origin") from error
    if (parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.path not in ("", "/")
            or parsed.username is not None or parsed.password is not None or parsed.query or parsed.fragment):
        raise ValueError("base URL must be an http(s) origin without a path")
    if port is not None and not 1 <= port <= 65535:
        raise ValueError("base URL port must be 1..65535")


def validate_read_only_timeout(timeout: float) -> None:
    if not math.isfinite(timeout) or not 0 < timeout <= MAX_READ_ONLY_TIMEOUT:
        raise ValueError(f"timeout must be finite and 0..{MAX_READ_ONLY_TIMEOUT:g} seconds")


def validate_drain_seconds(seconds: float) -> None:
    if not math.isfinite(seconds) or not 0 <= seconds <= MAX_DRAIN_SECONDS:
        raise ValueError(f"drain interval must be finite and 0..{MAX_DRAIN_SECONDS:g} seconds")


def drain_after_probes(seconds: float) -> None:
    if seconds:
        time.sleep(seconds)


def open_websocket_socket(base_url: str, timeout: float) -> socket.socket:
    validate_read_only_timeout(timeout)
    validate_operator_confirmation(True, base_url)
    url = urlsplit(base_url)
    host = url.hostname
    assert host is not None
    port = url.port or (443 if url.scheme == "https" else 80)
    sock = socket.create_connection((host, port), timeout=timeout)
    if url.scheme == "https":
        return ssl.create_default_context().wrap_socket(sock, server_hostname=host)
    return sock


def read_only_http_request(base_url: str, method: str, path: str, timeout: float = 5.0) -> HttpResult:
    validate_read_only_timeout(timeout)
    validate_operator_confirmation(True, base_url)
    if method not in READ_ONLY_METHODS:
        raise ValueError("read-only mode permits only GET and HEAD")
    url = urlsplit(base_url)
    connection_type = http.client.HTTPSConnection if url.scheme == "https" else http.client.HTTPConnection
    connection = connection_type(url.hostname, url.port, timeout=timeout)
    started = time.monotonic()
    try:
        connection.request(method, path, headers={"Host": url.netloc})
        response = connection.getresponse()
        response.read()
        return HttpResult(response.status, time.monotonic() - started)
    except socket.timeout:
        return HttpResult("timeout", time.monotonic() - started)
    except OSError:
        return HttpResult("error", time.monotonic() - started)
    finally:
        connection.close()


def format_read_only_report(
    outcomes: list[tuple[int | str, float]],
    duration: float,
    ws_packets: int,
    ws_pongs: int,
    diagnostics: dict[str, Any],
) -> str:
    statuses = {str(status): sum(1 for result, _ in outcomes if str(result) == str(status))
                for status, _ in outcomes}
    latencies = sorted(elapsed for status, elapsed in outcomes if isinstance(status, int) and 200 <= status < 300)
    p50 = latencies[len(latencies) // 2] * 1000 if latencies else float("nan")
    p95 = latencies[min(len(latencies) - 1, int(len(latencies) * .95))] * 1000 if latencies else float("nan")
    diagnostics_text = " ".join(f"{key}={value}" for key, value in diagnostics.items())
    return (f"outcomes={statuses} p50_ms={p50:.0f} p95_ms={p95:.0f} "
            f"packet_rate={ws_packets / duration:.1f}/s pong_rate={ws_pongs / duration:.1f}/s "
            f"{diagnostics_text}")


def system_diagnostics(base_url: str, timeout: float) -> dict[str, Any]:
    validate_read_only_timeout(timeout)
    validate_operator_confirmation(True, base_url)
    url = urlsplit(base_url)
    connection_type = http.client.HTTPSConnection if url.scheme == "https" else http.client.HTTPConnection
    connection = connection_type(url.hostname, url.port, timeout=timeout)
    try:
        connection.request("GET", "/api/v1/system", headers={"Host": url.netloc})
        response = connection.getresponse()
        if response.status != 200:
            return {"system_status": response.status}
        body = response.read(MAX_DIAGNOSTIC_BYTES + 1)
        if len(body) > MAX_DIAGNOSTIC_BYTES:
            return {"system_status": "too_large"}
        data = json.loads(body)
        if not isinstance(data, dict):
            return {"system_status": "invalid_response"}
        diag = {key: data[key] for key in ("uptime_s", "heap_free", "heap_min_free") if key in data}
        tasks = data.get("tasks", [])
        if not isinstance(tasks, list):
            tasks = []
        tasks = sorted((task for task in tasks if isinstance(task, dict)),
                       key=lambda task: task.get("stack_free", 0))[:3]
        diag["lowest_stacks"] = [(task.get("name"), task.get("stack_free")) for task in tasks]
        return diag
    except (OSError, ValueError, TypeError, AttributeError):
        return {"system_status": "unavailable"}
    finally:
        connection.close()


def synchronized_handshakes(
    base_url: str, count: int, timeout: float, *, schedule: str = "synchronized",
    stagger_ms: int = 250, exclude_sids: set[str] | None = None,
) -> list[WsClientStats]:
    """Run bounded handshake probes, each with its own sid, and retain upgraded sockets until every outcome is
    known. While another session is live each probe is upgraded and then closed by the server with code 4000."""
    validate_read_only_timeout(timeout)
    validate_operator_confirmation(True, base_url)
    if not 1 <= count <= 64:
        raise ValueError("handshake count must be 1..64")
    if schedule not in ("synchronized", "staggered"):
        raise ValueError("handshake schedule must be synchronized or staggered")
    if not 0 <= stagger_ms <= 2000:
        raise ValueError("handshake stagger must be 0..2000 ms")
    barrier = threading.Barrier(count) if schedule == "synchronized" else None
    sids: set[str] = set(exclude_sids or ())
    clients = [WsClientStats(f"probe-{index + 1}", "handshake-probe") for index in range(count)]
    for client in clients:
        client.sid = new_sid(sids)
        sids.add(client.sid)
    sockets: list[socket.socket | None] = [None] * count
    socket_locks = [threading.Lock() for _ in range(count)]
    abandoned = [threading.Event() for _ in range(count)]

    def connect(index: int) -> None:
        client = clients[index]
        url = urlsplit(base_url)
        sock = None
        started = time.monotonic()
        try:
            if barrier is not None:
                barrier.wait(timeout=timeout)
            elif index:
                time.sleep(index * stagger_ms / 1000)
            started = time.monotonic()
            remaining = started + timeout - time.monotonic()
            if remaining <= 0:
                raise socket.timeout
            sock = open_websocket_socket(base_url, remaining)
            with socket_locks[index]:
                late_socket = abandoned[index].is_set()
                if not late_socket:
                    sockets[index] = sock
            if late_socket:
                try:
                    sock.close()
                except OSError as error:
                    with client.lock:
                        client.exception_class = type(error).__name__
                        client.exception_message = sanitize_diagnostic_text(str(error))
                        client.disconnect_reason = "close_exception"
                sock = None
                return
            key = base64.b64encode(os.urandom(16)).decode()
            request = (f"GET {ws_path(client.sid)} HTTP/1.1\r\nHost: {url.netloc}\r\nUpgrade: websocket\r\n"
                       f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
            sock.sendall(request.encode())
            response = b""
            while b"\r\n\r\n" not in response and len(response) < 8192:
                remaining = started + timeout - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout
                sock.settimeout(remaining)
                part = sock.recv(1)
                if not part:
                    break
                response += part
            if b"\r\n\r\n" not in response:
                raise ConnectionError("incomplete WebSocket handshake headers")
            status_line = response.split(b"\r\n", 1)[0].decode("latin-1", errors="replace")
            with client.lock:
                client.handshake_status = sanitize_diagnostic_text(status_line)
                client.handshake_elapsed_s = time.monotonic() - started
            validate_websocket_handshake(response, key)
            with client.lock:
                client.handshake_valid = True
                client.connected = True
                client.connected_at = time.monotonic()
            code = read_close_code(sock, started + timeout)
            with client.lock:
                client.close_code = code
        except Exception as error:
            failure = handshake_failure(client.client_id, base_url, error)
            with client.lock:
                if client.handshake_status == "pending":
                    client.handshake_status = "exception"
                client.handshake_elapsed_s = time.monotonic() - started
                client.exception_class = failure.exception_class
                client.exception_message = failure.exception_message
                client.disconnect_reason = "handshake_failed"
        finally:
            client.handshake_complete.set()

    workers = [threading.Thread(target=connect, args=(index,), daemon=True) for index in range(count)]
    for worker in workers:
        worker.start()
    deadline = time.monotonic() + timeout + (count - 1) * stagger_ms / 1000 + 0.25
    for worker in workers:
        worker.join(timeout=max(0.0, deadline - time.monotonic()))
    lingering = [worker.is_alive() for worker in workers]
    for index, worker in enumerate(workers):
        if lingering[index]:
            abandoned[index].set()
            with socket_locks[index]:
                sock = sockets[index]
            if sock is not None:
                try:
                    sock.close()
                except OSError as error:
                    with clients[index].lock:
                        clients[index].exception_class = type(error).__name__
                        clients[index].exception_message = sanitize_diagnostic_text(str(error))
                        clients[index].disconnect_reason = "close_exception"
    for worker in workers:
        if worker.is_alive():
            worker.join(timeout=0.1)
    for index, worker in enumerate(workers):
        if worker.is_alive():
            with clients[index].lock:
                clients[index].worker_still_running = True
                clients[index].disconnect_reason = "cleanup_pending"
                if clients[index].handshake_status == "pending":
                    clients[index].handshake_status = "worker_still_running"
    primary_failure = any(client.disconnect_reason == "handshake_failed" for client in clients)
    close_errors: dict[int, OSError] = {}
    for index, sock in enumerate(sockets):
        if sock is not None and not abandoned[index].is_set() and not workers[index].is_alive():
            with clients[index].lock:
                if clients[index].connected_at is not None:
                    clients[index].connected_duration_s = time.monotonic() - clients[index].connected_at
                clients[index].connected = False
                if clients[index].disconnect_reason == "pending":
                    clients[index].disconnect_reason = "probe_complete"
            try:
                sock.close()
            except OSError as error:
                close_errors[index] = error
    reset_index = next((index for index, error in close_errors.items()
                        if isinstance(error, ConnectionResetError)), None)
    primary_close_index = reset_index if reset_index is not None else next(iter(close_errors), None)
    for index, error in close_errors.items():
        with clients[index].lock:
            if clients[index].disconnect_reason == "handshake_failed":
                continue
            if primary_failure or index != primary_close_index:
                clients[index].exception_class = ""
                clients[index].exception_message = ""
                clients[index].disconnect_reason = "controller_cancelled"
            else:
                clients[index].exception_class = type(error).__name__
                clients[index].exception_message = sanitize_diagnostic_text(str(error))
                clients[index].disconnect_reason = "close_exception"
    return clients


def stop_steady(ws_stop: threading.Event, client: WsClientStats, thread: threading.Thread,
                timeout: float) -> None:
    ws_stop.set()
    thread.join(timeout=timeout + 1)
    if thread.is_alive():
        with client.lock:
            transport = client.transport
            client.worker_still_running = True
        if transport is not None:
            try:
                transport.close()
            except OSError:
                pass


def run_read_only_mode(args: argparse.Namespace) -> None:
    validate_operator_confirmation(args.simulator_no_car, args.base_url)
    validate_drain_seconds(getattr(args, "drain_seconds", DEFAULT_DRAIN_SECONDS))
    validate_read_only_timeout(args.timeout)
    if not 1 <= args.seconds <= 3600:
        raise ValueError("duration must be 1..3600 s")
    if not 1 <= args.ws <= 8:
        raise ValueError("--ws is the number of refusal probes, 1..8")
    if len(args.concurrency) > 12 or any(n < 1 or n > 64 for n in args.concurrency):
        raise ValueError("provide 1..12 HTTP levels, each between 1 and 64")
    if not args.concurrency:
        raise ValueError("provide at least one HTTP concurrency level")
    schedule = getattr(args, "handshake_schedule", "synchronized")
    stagger_ms = getattr(args, "stagger_ms", 250)
    schedules = ("synchronized", "staggered") if schedule == "compare" else (schedule,)
    client = WsClientStats("steady-1", "steady-data", sid=new_sid())
    ws_stop = threading.Event()
    thread = threading.Thread(target=steady_ws_client,
                              args=(args.base_url, client, ws_stop, args.timeout,
                                    getattr(args, "takeover", False)), daemon=True)
    thread.start()
    client.handshake_complete.wait(args.timeout)
    thread.join(timeout=0.3)  # a busy device closes the upgraded socket with 4000 right away
    ready, live = ws_ready_live_counts([client])
    if ready != 1 or live != 1:
        failure_snapshot = (ready, live)
        with client.lock:
            if client.handshake_status == "pending":
                client.handshake_status = "startup_timeout"
                client.exception_class = "TimeoutError"
                client.exception_message = "steady WebSocket handshake startup deadline exceeded"
                client.disconnect_reason = "startup_timeout"
        stop_steady(ws_stop, client, thread, 0.6)
        print("steady WebSocket startup incomplete; probes and HTTP phase skipped:",
              format_ws_diagnostics([client], max(args.timeout, 0.001), counts=failure_snapshot))
        return
    try:
        for current_schedule in schedules:
            handshake_results = synchronized_handshakes(
                args.base_url, args.ws, args.timeout, schedule=current_schedule, stagger_ms=stagger_ms,
                exclude_sids={client.sid},
            )
            print(f"{current_schedule} refusal probes (expect close 4000):",
                  format_ws_diagnostics(handshake_results, max(args.timeout, 0.001)))
            if any(c.worker_still_running for c in handshake_results):
                print("remaining phases skipped: handshake worker cleanup is still pending")
                return
        if getattr(args, "handshake_only", False):
            return
        if schedule == "compare":
            print("handshake comparison complete; subsequent HTTP load phase skipped in compare mode")
            return
        drain_after_probes(getattr(args, "drain_seconds", DEFAULT_DRAIN_SECONDS))
        for concurrency in args.concurrency:
            if ws_ready_live_counts([client])[1] != 1:
                print("HTTP concurrency levels skipped: the steady WebSocket client is no longer live")
                print("steady WebSocket client:", format_ws_diagnostics([client], max(args.timeout, 0.001)))
                break
            outcomes = []
            end = time.monotonic() + args.seconds
            started = time.monotonic()
            with client.lock:
                before = {client.client_id: (client.pid_packets, client.pongs)}
            lock = threading.Lock()

            def worker() -> None:
                while time.monotonic() < end:
                    result = read_only_http_request(args.base_url, "GET", "/api/v1/system", args.timeout)
                    with lock:
                        outcomes.append((result.status, result.elapsed))

            workers = [threading.Thread(target=worker, daemon=True) for _ in range(concurrency)]
            for item in workers:
                item.start()
            for item in workers:
                item.join(timeout=args.seconds + args.timeout + 1)
            elapsed = max(time.monotonic() - started, 0.001)
            packets_before, pongs_before = before[client.client_id]
            print(f"HTTP concurrency={concurrency} duration={elapsed:.1f}s",
                  format_read_only_report(outcomes, elapsed, client.pid_packets - packets_before,
                                          client.pongs - pongs_before,
                                          system_diagnostics(args.base_url, args.timeout)))
            print("steady WebSocket client:", format_ws_diagnostics([client], elapsed, before))
            if ws_ready_live_counts([client])[1] != 1:
                print("remaining HTTP levels skipped: the steady WebSocket client is no longer live")
                break
    finally:
        stop_steady(ws_stop, client, thread, args.timeout)


def steady_ws_client(
    base_url: str, client: WsClientStats, stop: threading.Event, timeout: float,
    takeover: bool = False,
) -> None:
    validate_read_only_timeout(timeout)
    validate_operator_confirmation(True, base_url)
    if not client.sid:
        client.sid = new_sid()
    url = urlsplit(base_url)
    sock = None
    started = time.monotonic()
    try:
        sock = open_websocket_socket(base_url, timeout)
        with client.lock:
            client.transport = sock
        key = base64.b64encode(os.urandom(16)).decode()
        sock.sendall((f"GET {ws_path(client.sid, takeover)} HTTP/1.1\r\nHost: {url.netloc}\r\n"
                      f"Upgrade: websocket\r\nConnection: Upgrade\r\n"
                      f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        response = b""
        while b"\r\n\r\n" not in response and len(response) < 8192:
            remaining = started + timeout - time.monotonic()
            if remaining <= 0:
                raise socket.timeout("WebSocket handshake deadline exceeded")
            sock.settimeout(remaining)
            part = sock.recv(1)
            if not part:
                with client.lock:
                    client.handshake_status = "connection_closed_before_handshake"
                    client.handshake_elapsed_s = time.monotonic() - started
                    client.disconnect_reason = "peer_closed_before_handshake"
                return
            response += part
        if b"\r\n\r\n" not in response:
            with client.lock:
                client.handshake_status = "incomplete_handshake_headers"
                client.handshake_elapsed_s = time.monotonic() - started
                client.disconnect_reason = "incomplete_handshake_headers"
            client.handshake_complete.set()
            return
        status_line = response.split(b"\r\n", 1)[0].decode("latin-1", errors="replace")
        with client.lock:
            client.handshake_status = sanitize_diagnostic_text(status_line)
            client.handshake_elapsed_s = time.monotonic() - started
        validate_websocket_handshake(response, key)
        with client.lock:
            client.handshake_valid = True
            client.connected = True
            client.connected_at = time.monotonic()
        client.handshake_complete.set()

        def send(payload: bytes) -> None:
            mask = os.urandom(4)
            sock.sendall(bytes([0x82, 0x80 | len(payload)]) + mask +
                         bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload)))

        send(b"\xa0")
        sock.settimeout(0.5)
        buffer = b""
        next_ping = time.monotonic()
        while not stop.is_set():
            if time.monotonic() >= next_ping:
                send(b"\xa2")
                next_ping = time.monotonic() + 2
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                with client.lock:
                    client.disconnect_reason = "peer_closed"
                break
            buffer += data
            while (parsed := parse_ws_frame(buffer)) is not None:
                opcode, frame, buffer = parsed
                if opcode == 8:
                    with client.lock:
                        client.close_code = close_code_of(frame)
                        client.disconnect_reason = "server_close_frame"
                    return
                record_ws_payload(client, frame)
    except Exception as error:
        with client.lock:
            if client.handshake_status == "pending":
                client.handshake_status = "exception"
                client.handshake_elapsed_s = time.monotonic() - started
            client.exception_class = type(error).__name__
            client.exception_message = sanitize_diagnostic_text(str(error))
            client.disconnect_reason = "exception"
    finally:
        client.handshake_complete.set()
        with client.lock:
            if client.connected_at is not None:
                client.connected_duration_s = time.monotonic() - client.connected_at
            client.connected = False
            if client.disconnect_reason == "pending":
                client.disconnect_reason = "test_stopped" if stop.is_set() else "client_ended"
        if sock is not None:
            with client.lock:
                client.transport = None
            try:
                sock.close()
            except OSError as error:
                with client.lock:
                    client.exception_class = type(error).__name__
                    client.exception_message = sanitize_diagnostic_text(str(error))
                    client.disconnect_reason = "close_exception"


def http_get(host, path, timeout=5.0):
    t = time.time()
    try:
        c = http.client.HTTPConnection(host, 80, timeout=timeout)
        c.request("GET", path)
        r = c.getresponse()
        r.read()
        c.close()
        return r.status, time.time() - t
    except socket.timeout:
        return "timeout", time.time() - t
    except Exception:
        return "error", time.time() - t


class WsClient(threading.Thread):
    """Minimal RFC 6455 client with its own sid: sends 0xA0 (start PID stream) and a 0xA2 ping every 2 s, counts 0x02 packets."""

    def __init__(self, host):
        super().__init__(daemon=True)
        self.sid = new_sid()
        self.host, self.stop, self.packets, self.pongs, self.error, self.t0 = host, False, 0, 0, None, 0.0

    def send(self, s, payload):
        mask = os.urandom(4)
        s.sendall(bytes([0x82, 0x80 | len(payload)]) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def run(self):
        try:
            s = socket.create_connection((self.host, 80), timeout=3)
            key = base64.b64encode(os.urandom(16)).decode()
            s.sendall((f"GET {ws_path(self.sid)} HTTP/1.1\r\nHost: {self.host}\r\nUpgrade: websocket\r\n"
                       f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
            head = b""
            while b"\r\n\r\n" not in head:
                head += s.recv(1)
            if b" 101 " not in head.split(b"\r\n")[0]:
                self.error = head.split(b"\r\n")[0].decode()
                return
            self.send(s, bytes([0xA0]))
            s.settimeout(0.5)
            buf, nxt = b"", 0
            while not self.stop:
                if time.time() >= nxt:
                    self.send(s, bytes([0xA2]))
                    nxt = time.time() + 2
                try:
                    d = s.recv(4096)
                except socket.timeout:
                    continue
                if not d:
                    self.error = "closed by server"
                    return
                buf += d
                while (parsed := parse_ws_frame(buf)) is not None:
                    opcode, p, buf = parsed
                    if opcode == 8:
                        self.error = f"closed by server, code {close_code_of(p)}"
                        return
                    if p[:1] == b"\x02" and len(p) == 20:
                        self.packets += 1
                    elif p == b"\xa3":
                        self.pongs += 1
            s.close()
        except Exception as e:
            self.error = repr(e)


def slow_client(host, stop, counts):
    """Sends a GET one byte at a time (about 4 s per request), then reads the answer."""
    while not stop.is_set():
        try:
            s = socket.create_connection((host, 80), timeout=8)
            req = f"GET /api/v1/system HTTP/1.1\r\nHost: {host}\r\n\r\n".encode()
            for b in req:
                s.sendall(bytes([b]))
                time.sleep(4.0 / len(req))
            s.settimeout(8)
            counts.append("ok" if s.recv(64).startswith(b"HTTP/1.1 200") else "bad")
            s.close()
        except Exception:
            counts.append("err")


def pct(v, p):
    return statistics.quantiles(v, n=100)[p - 1] if len(v) >= 2 else (v[0] if v else float("nan"))


def diag(host):
    try:
        c = http.client.HTTPConnection(host, 80, timeout=8)
        c.request("GET", "/api/v1/system")
        import json
        d = json.loads(c.getresponse().read())
        tasks = sorted(d.get("tasks", []), key=lambda t: t["stack_free"])[:3]
        return d.get("uptime_s"), d.get("heap_free"), d.get("heap_min_free"), [(t["name"], t["stack_free"]) for t in tasks]
    except Exception as e:
        return None, None, None, repr(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", nargs="?", default="can-sniffer.local")
    ap.add_argument("--levels", default="2,4,8,16,32")
    ap.add_argument("--seconds", type=int, default=30)
    ap.add_argument("--ws", type=int, default=2,
                    help="read-only: number of refusal probes (1..8); legacy mode always runs one WS client")
    ap.add_argument("--slow", type=int, default=2)
    ap.add_argument("--no-large", action="store_true", help="skip the periodic gzipped GET /")
    ap.add_argument("--mode", choices=("legacy", "read-only"), default="legacy")
    ap.add_argument("--base-url", help="explicit http(s):// URL for read-only mode")
    ap.add_argument("--simulator-no-car", action="store_true",
                    help="confirm target is a simulator with no vehicle connected")
    ap.add_argument("--takeover", action="store_true",
                    help="steady client replaces a current WS session (disconnects an open page); default is to stop if busy")
    ap.add_argument("--handshake-schedule", choices=("synchronized", "staggered", "compare"),
                    default="synchronized", help="read-only handshake probe timing")
    ap.add_argument("--stagger-ms", type=int, default=250,
                    help="delay between staggered probe starts (0..2000 ms)")
    ap.add_argument("--handshake-only", action="store_true",
                    help="run selected handshake probes only; skip steady WebSockets and HTTP")
    ap.add_argument("--drain-seconds", type=float, default=DEFAULT_DRAIN_SECONDS,
                    help="pause after the probes and before the HTTP load phase (0..30; default 0)")
    ap.add_argument("--concurrency", default="1,2,4,8", help="HTTP concurrency levels for read-only mode")
    ap.add_argument("--timeout", type=float, default=5.0)
    a = ap.parse_args()
    if a.mode == "read-only":
        if not a.base_url:
            ap.error("--mode read-only requires --base-url")
        try:
            a.concurrency = [int(value) for value in a.concurrency.split(",")]
            validate_operator_confirmation(a.simulator_no_car, a.base_url)
            if not 0 < a.timeout <= 60 or not 1 <= a.ws <= 8:
                raise ValueError("timeout must be 0..60 s and --ws (refusal probes) 1..8")
            if not 0 <= a.stagger_ms <= 2000:
                raise ValueError("handshake stagger must be 0..2000 ms")
            run_read_only_mode(a)
        except ValueError as error:
            ap.error(str(error))
        return
    host = a.host
    http_get(host, "/api/v1/obd2")
    c = http.client.HTTPConnection(host, 80, timeout=5)
    c.request("PATCH", "/api/v1/obd2", '{"auto_poll": true}', {"Content-Type": "application/json"})
    print("poll start", c.getresponse().status)
    print("start   uptime/heap_free/heap_min/lowest stacks:", diag(host))
    ws = [WsClient(host)]  # the firmware serves one WebSocket session
    for w in ws:
        w.start()
    time.sleep(1)
    slow_stop, slow_counts = threading.Event(), []
    slow = [threading.Thread(target=slow_client, args=(host, slow_stop, slow_counts), daemon=True) for _ in range(a.slow)]
    for t in slow:
        t.start()
    print(f"{'N':>3} {'reqs':>6} {'ok':>6} {'429':>5} {'503':>5} {'other':>5} {'tmo':>4} {'err':>4} {'p50ms':>6} {'p95ms':>6} {'ws pkt/s':>8}")
    try:
        for n in [int(x) for x in a.levels.split(",")]:
            results, stop = [], threading.Event()
            big = []

            def worker(i):
                k = i
                while not stop.is_set():
                    path = ENDPOINTS[k % len(ENDPOINTS)]
                    results.append(http_get(host, path))
                    k += 1

            def large():
                while not stop.is_set():
                    big.append(http_get(host, LARGE, timeout=10))
                    time.sleep(2)

            threads = [threading.Thread(target=worker, args=(i,), daemon=True) for i in range(n)]
            if not a.no_large:
                threads.append(threading.Thread(target=large, daemon=True))
            before = [w.packets for w in ws]
            t0 = time.time()
            for t in threads:
                t.start()
            time.sleep(a.seconds)
            stop.set()
            for t in threads:
                t.join(timeout=12)
            dt = time.time() - t0
            st = [r[0] for r in results]
            ok = [r[1] for r in results if r[0] == 200]
            wsr = sum(w.packets - b for w, b in zip(ws, before)) / dt / max(1, len(ws))
            print(f"{n:>3} {len(st):>6} {st.count(200):>6} {st.count(429):>5} {st.count(503):>5} "
                  f"{sum(1 for s in st if isinstance(s, int) and s not in (200, 429, 503)):>5} {st.count('timeout'):>4} "
                  f"{st.count('error'):>4} {pct(ok, 50) * 1000:>6.0f} {pct(ok, 95) * 1000:>6.0f} {wsr:>8.1f}")
            print(f"    large GET / : {[b[0] for b in big]} latencies ms {[round(b[1] * 1000) for b in big][:6]}")
            print("    after level uptime/heap_free/heap_min/lowest stacks:", diag(host))
            time.sleep(3)
    finally:
        slow_stop.set()
        for w in ws:
            w.stop = True
        c = http.client.HTTPConnection(host, 80, timeout=5)
        c.request("PATCH", "/api/v1/obd2", '{"auto_poll": false}', {"Content-Type": "application/json"})
        print("poll stop", c.getresponse().status)
        print("slow clients:", {k: slow_counts.count(k) for k in set(slow_counts)})
        print("ws clients:", [(w.packets, w.pongs, w.error) for w in ws])
        print("end     uptime/heap_free/heap_min/lowest stacks:", diag(host))


if __name__ == "__main__":
    main()
