"""Hermetic behavior checks for the read-only load-test mode."""

from __future__ import annotations

import importlib.util
import argparse
import base64
import hashlib
from pathlib import Path
import sys
import threading
import time
from typing import Any
from unittest.mock import patch

import unittest


SPEC = importlib.util.spec_from_file_location(
    "load_test", Path(__file__).with_name("load-test.py")
)
assert SPEC is not None and SPEC.loader is not None
load_test = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = load_test
SPEC.loader.exec_module(load_test)


def fake_upgrade_response(key: str) -> bytes:
  accept = base64.b64encode(hashlib.sha1(
      (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
  ).digest())
  return (b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
          b"Connection: keep-alive, Upgrade\r\nSec-WebSocket-Accept: " + accept + b"\r\n\r\n")


class ReadOnlyLoadTest(unittest.TestCase):
  def test_websocket_handshake_requires_exact_rfc6455_upgrade(self) -> None:
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    valid = (
        b"HTTP/1.1 101 Switching Protocols\r\n"
        b"Upgrade: WebSocket\r\n"
        b"Connection: keep-alive, Upgrade\r\n"
        b"Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n\r\n"
    )
    self.assertEqual(load_test.validate_websocket_handshake(valid, key),
                     "HTTP/1.1 101 Switching Protocols")

  def test_websocket_handshake_rejects_bad_status_incomplete_headers_and_accept(self) -> None:
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    valid_headers = (
        b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
        b"Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n\r\n"
    )
    invalid_responses = (
        b"HTTP/1.1 1010 Fake\r\n" + valid_headers,
        b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n",
        b"HTTP/1.1 101 Switching Protocols\r\n"
        b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
        b"Sec-WebSocket-Accept: wrong-key\r\n\r\n",
        b"HTTP/1.1 101 Switching Protocols\r\nConnection: Upgrade\r\n"
        b"Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n\r\n",
    )
    for response in invalid_responses:
      with self.subTest(response=response):
        with self.assertRaises(ValueError):
          load_test.validate_websocket_handshake(response, key)

  def test_probe_sockets_remain_open_until_all_outcomes_are_collected(self) -> None:
    state = {"completed": 0, "closed_too_early": False}

    class FakeSocket:
      def __init__(self) -> None:
        self.response = b""

      def settimeout(self, _timeout: float) -> None:
        pass

      def sendall(self, data: bytes) -> None:
        key = next(line.split(b":", 1)[1].strip().decode()
                   for line in data.split(b"\r\n") if line.startswith(b"Sec-WebSocket-Key:"))
        self.response = fake_upgrade_response(key)

      def recv(self, _size: int) -> bytes:
        part, self.response = self.response[:1], self.response[1:]
        if self.response == b"":
          state["completed"] += 1
        return part

      def close(self) -> None:
        if state["completed"] < 3:
          state["closed_too_early"] = True

    original = load_test.socket.create_connection
    load_test.socket.create_connection = lambda *_args, **_kwargs: FakeSocket()
    try:
      results = load_test.synchronized_handshakes("http://localhost", 3, 1.0)
    finally:
      load_test.socket.create_connection = original
    self.assertEqual(sum(client.handshake_valid for client in results), 3)
    self.assertEqual(state["completed"], 3)
    self.assertFalse(state["closed_too_early"])

  def test_staggered_schedule_offsets_client_connection_starts(self) -> None:
    arrivals: list[float] = []

    class FakeSocket:
      def __init__(self) -> None:
        self.response = b""

      def settimeout(self, _timeout: float) -> None:
        pass

      def sendall(self, data: bytes) -> None:
        key = next(line.split(b":", 1)[1].strip().decode()
                   for line in data.split(b"\r\n") if line.startswith(b"Sec-WebSocket-Key:"))
        self.response = fake_upgrade_response(key)

      def recv(self, _size: int) -> bytes:
        part, self.response = self.response[:1], self.response[1:]
        return part

      def close(self) -> None:
        pass

    original = load_test.socket.create_connection

    def record_start(_address: tuple[str | None, int], timeout: float) -> FakeSocket:
      arrivals.append(time.monotonic())
      return FakeSocket()

    load_test.socket.create_connection = record_start
    try:
      results = load_test.synchronized_handshakes(
          "http://localhost", 3, 1.0, schedule="staggered", stagger_ms=30
      )
    finally:
      load_test.socket.create_connection = original
    self.assertTrue(all(client.handshake_valid for client in results))
    self.assertGreaterEqual(arrivals[1] - arrivals[0], 0.02)
    self.assertGreaterEqual(arrivals[2] - arrivals[1], 0.02)

  def test_handshake_only_compare_runs_both_schedules_without_load_phases(self) -> None:
    args = argparse.Namespace(
        simulator_no_car=True, no_other_ws_clients=True, base_url="http://localhost",
        seconds=1, ws=3, concurrency=[1], timeout=1.0,
        handshake_schedule="compare", stagger_ms=30, handshake_only=True,
    )
    observed: list[str] = []

    def probes(_url: str, count: int, _timeout: float, *, schedule: str,
                stagger_ms: int) -> list[Any]:
      observed.append(schedule)
      return [load_test.WsClientStats(f"{schedule}-{index}", "handshake-probe")
              for index in range(count)]

    with (
        patch.object(load_test, "synchronized_handshakes", side_effect=probes),
        patch.object(load_test, "steady_ws_client", side_effect=AssertionError("steady WS started")),
        patch.object(load_test, "read_only_http_request", side_effect=AssertionError("HTTP started")),
    ):
      load_test.run_read_only_mode(args)
    self.assertEqual(observed, ["synchronized", "staggered"])

  def test_failed_probe_sockets_are_closed_after_all_failure_outcomes(self) -> None:
    state = {"completed": 0, "closed_too_early": False, "closed": 0}

    class FakeSocket:
      def __init__(self) -> None:
        self.response = b"HTTP/1.1 503 Busy\r\nContent-Length: 0\r\n\r\n"

      def settimeout(self, _timeout: float) -> None:
        pass

      def sendall(self, _data: bytes) -> None:
        pass

      def recv(self, _size: int) -> bytes:
        part, self.response = self.response[:1], self.response[1:]
        if self.response == b"":
          state["completed"] += 1
        return part

      def close(self) -> None:
        state["closed"] += 1
        if state["completed"] < 3:
          state["closed_too_early"] = True

    original = load_test.socket.create_connection
    load_test.socket.create_connection = lambda *_args, **_kwargs: FakeSocket()
    try:
      results = load_test.synchronized_handshakes("http://localhost", 3, 1.0)
    finally:
      load_test.socket.create_connection = original
    self.assertEqual(sum(client.handshake_status.startswith("HTTP/1.1 503") for client in results), 3)
    self.assertEqual(state["closed"], 3)
    self.assertFalse(state["closed_too_early"])

  def test_socket_created_after_controller_returns_is_closed_by_late_worker(self) -> None:
    connect_started = threading.Event()
    release_connect = threading.Event()
    fake_sockets: list[Any] = []

    class FakeSocket:
      def __init__(self) -> None:
        self.closed = threading.Event()
        self.close_calls = 0

      def close(self) -> None:
        self.close_calls += 1
        self.closed.set()

    def delayed_connect(*_args: Any, **_kwargs: Any) -> FakeSocket:
      connect_started.set()
      self.assertTrue(release_connect.wait(2.0))
      sock = FakeSocket()
      fake_sockets.append(sock)
      return sock

    original = load_test.socket.create_connection
    load_test.socket.create_connection = delayed_connect
    try:
      results = load_test.synchronized_handshakes("http://localhost", 1, 0.03)
      self.assertTrue(connect_started.is_set())
      self.assertTrue(results[0].worker_still_running)
      release_connect.set()
      self.assertTrue(fake_sockets or release_connect.wait(0.1))
      deadline = time.monotonic() + 1.0
      while not fake_sockets and time.monotonic() < deadline:
        time.sleep(0.005)
      self.assertEqual(len(fake_sockets), 1)
      self.assertTrue(fake_sockets[0].closed.wait(1.0))
      self.assertEqual(fake_sockets[0].close_calls, 1)
    finally:
      release_connect.set()
      load_test.socket.create_connection = original

  def test_successful_handshake_racing_controller_abandonment_has_one_close_owner(self) -> None:
    fake_sockets: list[Any] = []
    validation_reached = threading.Event()
    release_validation = threading.Event()
    controller_done = threading.Event()
    results: list[Any] = []

    class FakeSocket:
      def __init__(self) -> None:
        self.closed = threading.Event()
        self.response = b""
        self.close_calls = 0

      def settimeout(self, _timeout: float) -> None:
        pass

      def sendall(self, data: bytes) -> None:
        key = next(line.split(b":", 1)[1].strip().decode()
                   for line in data.split(b"\r\n") if line.startswith(b"Sec-WebSocket-Key:"))
        self.response = fake_upgrade_response(key)

      def recv(self, _size: int) -> bytes:
        part, self.response = self.response[:1], self.response[1:]
        return part

      def close(self) -> None:
        self.close_calls += 1
        self.closed.set()

    original = load_test.socket.create_connection

    def create_socket(*_args: Any, **_kwargs: Any) -> FakeSocket:
      sock = FakeSocket()
      fake_sockets.append(sock)
      return sock

    original_validator = load_test.validate_websocket_handshake

    def delayed_validator(response: bytes, key: str) -> str:
      status = original_validator(response, key)
      validation_reached.set()
      if not release_validation.wait(2.0):
        raise TimeoutError("test validation gate timed out")
      return status

    def run_controller() -> None:
      results.extend(load_test.synchronized_handshakes("http://localhost", 1, 0.03))
      controller_done.set()

    load_test.socket.create_connection = create_socket
    load_test.validate_websocket_handshake = delayed_validator
    try:
      controller = threading.Thread(target=run_controller)
      controller.start()
      self.assertTrue(validation_reached.wait(1.0))
      self.assertTrue(controller_done.wait(1.0))
      self.assertEqual(len(fake_sockets), 1)
      self.assertTrue(fake_sockets[0].closed.wait(1.0))
      release_validation.set()
      controller.join(timeout=1.0)
      deadline = time.monotonic() + 1.0
      while not results[0].handshake_valid and time.monotonic() < deadline:
        time.sleep(0.005)
      self.assertTrue(results[0].handshake_valid)
      self.assertEqual(fake_sockets[0].close_calls, 1)
    finally:
      release_validation.set()
      load_test.validate_websocket_handshake = original_validator
      load_test.socket.create_connection = original

  def test_read_only_subscription_requires_exclusive_websocket_confirmation(self) -> None:
    with self.assertRaises(ValueError):
      load_test.validate_ws_exclusivity(False)
    load_test.validate_ws_exclusivity(True)

  def test_websocket_report_keeps_clients_separate_and_shows_rates_and_lifecycle(self) -> None:
    probe = load_test.WsClientStats(client_id="probe-2", role="handshake-probe")
    probe.handshake_status = "HTTP/1.1 503 Service Unavailable"
    probe.handshake_elapsed_s = 0.2
    stream = load_test.WsClientStats(client_id="steady-1", role="steady-data")
    stream.handshake_status = "HTTP/1.1 101 Switching Protocols"
    stream.handshake_valid = True
    stream.connected = True
    stream.pid_packets = 8
    stream.pongs = 2
    report = load_test.format_ws_diagnostics([probe, stream], 2.0)
    self.assertIn("handshake-probe id=probe-2 status=HTTP/1.1 503", report)
    self.assertIn("steady-data id=steady-1 status=HTTP/1.1 101", report)
    self.assertIn("pid=8 pid_rate=4.0/s pong=2 pong_rate=1.0/s", report)
    self.assertIn("ready=1 live=1 expected=3", report)

  def test_websocket_frame_accounting_is_client_local_and_exact(self) -> None:
    first = load_test.WsClientStats(client_id="steady-1", role="steady-data")
    second = load_test.WsClientStats(client_id="steady-2", role="steady-data")
    load_test.record_ws_payload(first, b"\x02" + bytes(19))
    load_test.record_ws_payload(first, b"\xa3")
    load_test.record_ws_payload(second, b"\x02" + bytes(18))
    self.assertEqual((first.pid_packets, first.pongs), (1, 1))
    self.assertEqual((second.pid_packets, second.pongs), (0, 0))

  def test_steady_client_records_pid_pong_and_disconnect_per_client(self) -> None:
    class FakeSocket:
      def __init__(self) -> None:
        self.header = b""
        self.stream = b"\x82\x14\x02" + bytes(19) + b"\x82\x01\xa3"
        self.frames: list[bytes] = []

      def settimeout(self, _timeout: float) -> None:
        pass

      def sendall(self, data: bytes) -> None:
        if data.startswith(b"GET "):
          key = next(line.split(b":", 1)[1].strip().decode()
                     for line in data.split(b"\r\n") if line.startswith(b"Sec-WebSocket-Key:"))
          self.header = fake_upgrade_response(key)
        else:
          self.frames.append(data)

      def recv(self, size: int) -> bytes:
        if self.header:
          part, self.header = self.header[:size], self.header[size:]
          return part
        if self.stream:
          payload, self.stream = self.stream, b""
          return payload
        return b""

      def close(self) -> None:
        pass

    client = load_test.WsClientStats("steady-2", "steady-data")
    fake_socket = FakeSocket()
    original = load_test.socket.create_connection
    load_test.socket.create_connection = lambda *_args, **_kwargs: fake_socket
    try:
      load_test.steady_ws_client("http://localhost", client, threading.Event(), 1.0, True)
    finally:
      load_test.socket.create_connection = original
    self.assertTrue(client.handshake_status.startswith("HTTP/1.1 101"))
    self.assertEqual((client.pid_packets, client.pongs), (1, 1))
    self.assertGreaterEqual(client.connected_duration_s, 0)
    self.assertEqual(client.disconnect_reason, "peer_closed")
    decoded_commands = []
    for frame in fake_socket.frames:
      mask, payload = frame[2:6], frame[6:]
      decoded_commands.append(bytes(value ^ mask[index % 4] for index, value in enumerate(payload)))
    self.assertIn(b"\xa0", decoded_commands)
    self.assertNotIn(b"\xa1", decoded_commands)

  def test_handshake_transport_exception_is_reported_with_safe_details(self) -> None:
    original = load_test.socket.create_connection
    load_test.socket.create_connection = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        TimeoutError("request https://user:secret@example.invalid/path token=abc")
    )
    try:
      client = load_test.synchronized_handshakes("http://localhost", 1, 1.0)[0]
    finally:
      load_test.socket.create_connection = original
    self.assertEqual(client.client_id, "probe-1")
    self.assertEqual(client.handshake_status, "exception")
    self.assertGreaterEqual(client.handshake_elapsed_s, 0)
    self.assertEqual(client.exception_class, "TimeoutError")
    self.assertEqual(client.disconnect_reason, "handshake_failed")
    self.assertNotIn("secret", client.exception_message)
    self.assertNotIn("abc", client.exception_message)

  def test_handshake_exception_report_is_concrete_but_redacts_credentials(self) -> None:
    result = load_test.handshake_failure(
        "probe-1", "https://user:secret@example.invalid",
        TimeoutError("request https://user:secret@example.invalid/path token=abc Authorization: Bearer other")
    )
    self.assertEqual(result.client_id, "probe-1")
    self.assertEqual(result.exception_class, "TimeoutError")
    self.assertNotIn("secret", result.exception_message)
    self.assertNotIn("user", result.exception_message)
    self.assertNotIn("abc", result.exception_message)
    self.assertNotIn("other", result.exception_message)

  def test_lingering_handshake_aborts_steady_and_http_phases(self) -> None:
    args = argparse.Namespace(
        simulator_no_car=True,
        no_other_ws_clients=True,
        base_url="http://localhost",
        seconds=1,
        ws=3,
        concurrency=[1],
        timeout=1.0,
    )
    steady_started = threading.Event()
    with (
        patch.object(load_test, "synchronized_handshakes",
                     return_value=[load_test.WsClientStats("probe-1", "handshake-probe",
                                                           worker_still_running=True)]),
        patch.object(load_test, "steady_ws_client", side_effect=lambda *_args: steady_started.set()),
        patch.object(load_test, "read_only_http_request", side_effect=AssertionError("HTTP phase started")),
        patch.object(load_test, "system_diagnostics", return_value={}),
        patch.object(load_test.time, "sleep"),
    ):
      load_test.run_read_only_mode(args)
    self.assertFalse(steady_started.is_set())

  def test_steady_phase_requires_three_live_handshakes(self) -> None:
    args = argparse.Namespace(
        simulator_no_car=True,
        no_other_ws_clients=True,
        base_url="http://localhost",
        seconds=1,
        ws=3,
        concurrency=[1],
        timeout=1.0,
    )
    probes = [load_test.WsClientStats(f"probe-{index}", "handshake-probe") for index in range(1, 4)]
    clients_started: list[str] = []

    def start_fake_client(_url: str, client: Any, _stop: threading.Event,
                          _timeout: float, _exclusive: bool) -> None:
      clients_started.append(client.client_id)
      with client.lock:
        client.handshake_status = ("HTTP/1.1 101 Switching Protocols" if client.client_id != "steady-3"
                                   else "exception")
        client.handshake_valid = client.client_id != "steady-3"
        client.connected = client.client_id != "steady-3"
      client.handshake_complete.set()

    with (
        patch.object(load_test, "synchronized_handshakes", return_value=probes),
        patch.object(load_test, "steady_ws_client", side_effect=start_fake_client),
        patch.object(load_test, "read_only_http_request", side_effect=AssertionError("HTTP phase started")),
        patch.object(load_test.time, "sleep"),
    ):
      load_test.run_read_only_mode(args)
    self.assertEqual(sorted(clients_started), ["steady-1", "steady-2", "steady-3"])

  def test_read_only_http_client_rejects_mutating_methods_before_transport(self) -> None:
    methods: list[str] = []

    class FakeConnection:
        def __init__(self, *_: Any, **__: Any) -> None:
            pass

        def request(self, method: str, _path: str, **_: Any) -> None:
            methods.append(method)

        def getresponse(self) -> Any:
            class Response:
                status = 200

                @staticmethod
                def read() -> bytes:
                    return b"{}"

            return Response()

        @staticmethod
        def close() -> None:
            pass

    original = load_test.http.client.HTTPConnection
    load_test.http.client.HTTPConnection = FakeConnection
    try:
      with self.assertRaises(ValueError):
        load_test.read_only_http_request("localhost", "PATCH", "/api/v1/obd2")
      self.assertEqual(methods, [])
      result = load_test.read_only_http_request("http://localhost", "GET", "/api/v1/system")
      self.assertEqual(result.status, 200)
    finally:
      load_test.http.client.HTTPConnection = original


  def test_handshakes_reach_fake_transport_together(self) -> None:
    arrivals: list[float] = []

    class FakeSocket:
      def __init__(self) -> None:
        self.response = b""

      def settimeout(self, _timeout: float) -> None:
        pass

      def sendall(self, data: bytes) -> None:
        key = next(line.split(b":", 1)[1].strip().decode()
                   for line in data.split(b"\r\n") if line.startswith(b"Sec-WebSocket-Key:"))
        self.response = fake_upgrade_response(key)

      def recv(self, _size: int) -> bytes:
        part, self.response = self.response[:_size], self.response[_size:]
        return part

      def close(self) -> None:
        pass

    original = load_test.socket.create_connection

    def fake_connect(_address: tuple[str | None, int], timeout: float) -> FakeSocket:
      arrivals.append(time.monotonic())
      return FakeSocket()

    load_test.socket.create_connection = fake_connect
    try:
      outcomes = load_test.synchronized_handshakes("http://example.invalid", 3, 1.0)
    finally:
      load_test.socket.create_connection = original
    self.assertEqual([outcome.handshake_status for outcome in outcomes],
                     ["HTTP/1.1 101 Switching Protocols"] * 3)
    self.assertTrue(all(outcome.handshake_elapsed_s > 0 for outcome in outcomes))
    self.assertTrue(all(outcome.disconnect_reason == "probe_complete" for outcome in outcomes))
    self.assertTrue(all(outcome.connected_duration_s >= 0 for outcome in outcomes))
    self.assertEqual(len(arrivals), 3)
    self.assertLess(max(arrivals) - min(arrivals), 0.1)


  def test_new_mode_rejects_unbounded_or_empty_load(self) -> None:
    for duration, concurrency in [(0, 1), (1, 0), (3601, 1), (1, 65)]:
      with self.subTest(duration=duration, concurrency=concurrency):
        with self.assertRaises(ValueError):
          load_test.validate_read_only_bounds(duration, concurrency)


  def test_read_only_mode_requires_operator_simulator_confirmation(self) -> None:
    with self.assertRaises(ValueError):
      load_test.validate_operator_confirmation(False, "")

  def test_read_only_mode_requires_exclusive_ws_confirmation_before_network(self) -> None:
    args = argparse.Namespace(simulator_no_car=True, base_url="http://localhost", seconds=1,
                              ws=3, concurrency=[1], timeout=1.0, no_other_ws_clients=False)
    original = load_test.socket.create_connection
    load_test.socket.create_connection = lambda *_args, **_kwargs: self.fail("network started")
    try:
      with self.assertRaisesRegex(ValueError, "no-other-ws-clients"):
        load_test.run_read_only_mode(args)
    finally:
      load_test.socket.create_connection = original

  def test_read_only_callable_rejects_nonfinite_timeout_before_starting_network(self) -> None:
    args = argparse.Namespace(
        simulator_no_car=True,
        no_other_ws_clients=True,
        base_url="http://localhost",
        seconds=1,
        ws=3,
        concurrency=[1],
        timeout=float("nan"),
    )
    original = load_test.socket.create_connection
    load_test.socket.create_connection = lambda *_args, **_kwargs: self.fail("network started")
    with self.assertRaises(ValueError):
      try:
        load_test.run_read_only_mode(args)
      finally:
        load_test.socket.create_connection = original

  def test_origin_rejects_userinfo_fragment_query_and_malformed_url(self) -> None:
    for target in (
        "http://user:pass@localhost",
        "http://localhost/#fragment",
        "http://localhost/?query=1",
        "http://localhost:bad",
    ):
      with self.subTest(target=target):
        with self.assertRaises(ValueError):
          load_test.validate_operator_confirmation(True, target)

  def test_system_diagnostics_uses_one_bounded_response(self) -> None:
    requests: list[str] = []
    read_limits: list[int] = []

    class FakeResponse:
      status = 200

      def read(self, limit: int) -> bytes:
        read_limits.append(limit)
        return b"x" * limit

    class FakeConnection:
      def __init__(self, *_: Any, **__: Any) -> None:
        pass

      def request(self, method: str, path: str, **_: Any) -> None:
        requests.append(f"{method} {path}")

      def getresponse(self) -> FakeResponse:
        return FakeResponse()

      @staticmethod
      def close() -> None:
        pass

    original = load_test.http.client.HTTPConnection
    load_test.http.client.HTTPConnection = FakeConnection
    try:
      diagnostics = load_test.system_diagnostics("http://localhost", 2.0)
    finally:
      load_test.http.client.HTTPConnection = original
    self.assertEqual(requests, ["GET /api/v1/system"])
    self.assertEqual(read_limits, [load_test.MAX_DIAGNOSTIC_BYTES + 1])
    self.assertEqual(diagnostics, {"system_status": "too_large"})

  def test_system_diagnostics_does_not_read_error_response_body(self) -> None:
    class FakeResponse:
      status = 503

      def read(self, _limit: int) -> bytes:
        raise AssertionError("error body should not be read")

    class FakeConnection:
      def __init__(self, *_: Any, **__: Any) -> None:
        pass

      @staticmethod
      def request(*_: Any, **__: Any) -> None:
        pass

      @staticmethod
      def getresponse() -> FakeResponse:
        return FakeResponse()

      @staticmethod
      def close() -> None:
        pass

    original = load_test.http.client.HTTPConnection
    load_test.http.client.HTTPConnection = FakeConnection
    try:
      diagnostics = load_test.system_diagnostics("http://localhost", 2.0)
    finally:
      load_test.http.client.HTTPConnection = original
    self.assertEqual(diagnostics, {"system_status": 503})

  def test_system_diagnostics_extracts_heap_and_lowest_stacks(self) -> None:
    class FakeResponse:
      status = 200

      def read(self, limit: int) -> bytes:
        if limit != load_test.MAX_DIAGNOSTIC_BYTES + 1:
          raise AssertionError("diagnostic read was not size-limited")
        return (b'{"uptime_s":9,"heap_free":42,"tasks":['
                b'{"name":"worker","stack_free":8},{"name":"idle","stack_free":3}]}')

    class FakeConnection:
      def __init__(self, *_: Any, **__: Any) -> None:
        pass

      @staticmethod
      def request(*_: Any, **__: Any) -> None:
        pass

      @staticmethod
      def getresponse() -> FakeResponse:
        return FakeResponse()

      @staticmethod
      def close() -> None:
        pass

    original = load_test.http.client.HTTPConnection
    load_test.http.client.HTTPConnection = FakeConnection
    try:
      diagnostics = load_test.system_diagnostics("http://localhost", 2.0)
    finally:
      load_test.http.client.HTTPConnection = original
    self.assertEqual(diagnostics["uptime_s"], 9)
    self.assertEqual(diagnostics["heap_free"], 42)
    self.assertEqual(diagnostics["lowest_stacks"], [("idle", 3), ("worker", 8)])

  def test_handshake_timeout_reports_live_worker_and_returns_boundedly(self) -> None:
    release = threading.Event()

    class BlockingSocket:
      def settimeout(self, _timeout: float) -> None:
        pass

      def sendall(self, _data: bytes) -> None:
        pass

      def recv(self, _size: int) -> bytes:
        release.wait(timeout=2)
        return b""

      def close(self) -> None:
        pass

    original = load_test.socket.create_connection
    load_test.socket.create_connection = lambda *_args, **_kwargs: BlockingSocket()
    try:
      outcomes = load_test.synchronized_handshakes("http://localhost", 1, 0.05)
      self.assertTrue(outcomes[0].worker_still_running)
    finally:
      release.set()
      load_test.socket.create_connection = original


  def test_report_includes_available_diagnostics_and_websocket_rates(self) -> None:
    report = load_test.format_read_only_report(
        outcomes=[(200, 0.1), ("timeout", 1.0)],
        duration=2.0,
        ws_packets=4,
        ws_pongs=2,
        diagnostics={"uptime_s": 12, "heap_free": 1000, "tasks": []},
    )
    self.assertIn("'timeout': 1", report)
    self.assertIn("packet_rate=2.0/s", report)
    self.assertIn("pong_rate=1.0/s", report)
    self.assertIn("uptime_s=12", report)
    self.assertIn("heap_free=1000", report)


if __name__ == "__main__":
  unittest.main()
