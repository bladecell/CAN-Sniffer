import base64
import hashlib
import os
import socket
import ssl
import struct
import time
from urllib.parse import urlsplit

import pytest
import requests


pytestmark = pytest.mark.integration


class WebSocketUnavailable(Exception):
    """The optional device WebSocket endpoint cannot be reached."""


class RawWebSocket:
    """Small dependency-free WebSocket client for the firmware's binary stream."""

    def __init__(self, base_url, timeout=1.0):
        parsed = urlsplit(base_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise WebSocketUnavailable(f"unsupported CAN_SNIFFER_URL: {base_url!r}")
        self._parsed = parsed
        self._timeout = timeout
        self._sock = None

    def __enter__(self):
        port = self._parsed.port or (443 if self._parsed.scheme == "https" else 80)
        sock = socket.create_connection((self._parsed.hostname, port), timeout=self._timeout)
        if self._parsed.scheme == "https":
            context = ssl.create_default_context()
            sock = context.wrap_socket(sock, server_hostname=self._parsed.hostname)
        sock.settimeout(self._timeout)
        self._sock = sock
        self._buffer = bytearray()

        key = base64.b64encode(os.urandom(16)).decode("ascii")
        host = self._parsed.hostname
        if ":" in host and not host.startswith("["):
            host = f"[{host}]"
        if self._parsed.port:
            host = f"{host}:{self._parsed.port}"
        request = (
            "GET /ws HTTP/1.1\r\n"
            f"Host: {host}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        ).encode("ascii")
        self._sock.sendall(request)
        headers = self._read_until(b"\r\n\r\n")
        status_line, *header_lines = headers.decode("latin1").split("\r\n")
        if " 101 " not in status_line:
            raise WebSocketUnavailable(f"WebSocket handshake failed: {status_line}")
        response_headers = {
            line.split(":", 1)[0].strip().lower(): line.split(":", 1)[1].strip()
            for line in header_lines
            if ":" in line
        }
        expected_accept = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
        ).decode("ascii")
        if response_headers.get("sec-websocket-accept") != expected_accept:
            raise WebSocketUnavailable("invalid WebSocket handshake response")
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._sock is not None:
            try:
                self._send_frame(0x8, b"")
            except (OSError, TimeoutError):
                pass
            self._sock.close()
            self._sock = None

    def _read_until(self, marker):
        data = bytearray()
        while marker not in data:
            chunk = self._sock.recv(1024)
            if not chunk:
                raise WebSocketUnavailable("WebSocket closed during handshake")
            data.extend(chunk)
            if len(data) > 16 * 1024:
                raise WebSocketUnavailable("oversized WebSocket handshake")
        header_end = data.index(marker) + len(marker)
        self._buffer.extend(data[header_end:])
        return bytes(data[:header_end])

    def _read_exact(self, count):
        data = bytearray(self._buffer[:count])
        del self._buffer[: len(data)]
        while len(data) < count:
            chunk = self._sock.recv(count - len(data))
            if not chunk:
                raise WebSocketUnavailable("WebSocket closed while reading a frame")
            data.extend(chunk)
        return bytes(data)

    def _send_frame(self, opcode, payload):
        mask = os.urandom(4)
        masked = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        length = len(payload)
        if length < 126:
            header = bytes((0x80 | opcode, 0x80 | length))
        elif length <= 0xFFFF:
            header = bytes((0x80 | opcode, 0x80 | 126)) + struct.pack(">H", length)
        else:
            header = bytes((0x80 | opcode, 0x80 | 127)) + struct.pack(">Q", length)
        self._sock.sendall(header + mask + masked)

    def send_binary(self, payload):
        self._send_frame(0x2, payload)

    def recv_frame(self):
        first, second = self._read_exact(2)
        opcode = first & 0x0F
        length = second & 0x7F
        masked = bool(second & 0x80)
        if length == 126:
            length = struct.unpack(">H", self._read_exact(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._read_exact(8))[0]
        mask = self._read_exact(4) if masked else b""
        payload = self._read_exact(length)
        if masked:
            payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        return opcode, payload


def pid_data_snapshot(api):
    response = api.request("GET", "/api/v1/pid_data")
    assert response.status_code == 200, response.text
    payload = response.json()
    assert isinstance(payload, dict)
    assert isinstance(payload.get("data"), list)
    return {int(item["pid"]): item for item in payload["data"]}


def continuous_polling_state(api):
    try:
        response = api.request("GET", "/api/v1/obd2")
    except (requests.RequestException, RuntimeError) as exc:
        pytest.skip(f"cannot safely preserve continuous polling state: {exc}")
    if response.status_code != 200:
        pytest.skip(f"cannot safely preserve continuous polling state: HTTP {response.status_code}")
    try:
        payload = response.json()
    except ValueError:
        pytest.skip("cannot safely preserve continuous polling state: invalid JSON response")
    if not isinstance(payload, dict) or type(payload.get("continuous_running")) is not bool:
        pytest.skip("cannot safely preserve continuous polling state: missing boolean continuous_running")
    return payload["continuous_running"]


def set_continuous_polling(api, running):
    response = api.request("POST", f"/api/v1/req/pid_poll?running={str(running).lower()}")
    assert response.status_code == 201, response.text


def test_pid_websocket_packets_have_rest_status_flags(api, base_url, device_reachable):
    if os.environ.get("CAN_SNIFFER_ENABLE_WEBSOCKET_SMOKE") != "1":
        pytest.skip("set CAN_SNIFFER_ENABLE_WEBSOCKET_SMOKE=1 to enable live polling/WebSocket smoke test")

    original_polling_state = continuous_polling_state(api)

    packets = []
    try:
        if not original_polling_state:
            set_continuous_polling(api, True)
        try:
            with RawWebSocket(base_url) as websocket:
                websocket.send_binary(bytes((0xA0,)))
                deadline = time.monotonic() + 7.0
                while time.monotonic() < deadline:
                    try:
                        opcode, payload = websocket.recv_frame()
                    except socket.timeout:
                        continue
                    if opcode == 0x8:
                        break
                    if opcode == 0x2 and payload and payload[0] == 0x02:
                        assert len(payload) == 20
                        assert payload[1] == 18
                        assert payload[18] in (0, 1)
                        assert payload[19] in (0, 1)
                        packets.append(payload)
                        if len(packets) >= 3:
                            break
        except (OSError, TimeoutError, WebSocketUnavailable) as exc:
            pytest.skip(f"WebSocket stream unavailable: {exc}")

        if not packets:
            pytest.skip("PID stream emitted no PID data in the allotted time")

        rest_data = pid_data_snapshot(api)
        correlated = False
        for packet in packets:
            pid = int.from_bytes(packet[2:6], "little")
            item = rest_data.get(pid)
            if item is None:
                continue
            correlated = True
            assert type(item["isSupported"]) is bool
            assert type(item["isValid"]) is bool
            assert packet[18] == int(item["isSupported"])
            assert packet[19] == int(item["isValid"])
        if not correlated:
            pytest.skip("PID stream data could not be correlated with the REST snapshot")
    finally:
        set_continuous_polling(api, original_polling_state)
