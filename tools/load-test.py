#!/usr/bin/env python3
"""Load test for the async web server (WEB-1): parallel read-only HTTP clients plus WebSocket clients.

Usage: tools/load-test.py [host] [--levels 2,4,8,16,32] [--seconds 30] [--ws 3] [--slow 2]
Stdlib only. Only GET requests on read-only endpoints, plus POST /req/auto_poll to start and stop polling.
Per level it prints success / 429 / 503 / other-status / timeout / error counts, latency p50 and p95, the WebSocket
packet rate, and heap and the lowest task stack from GET /system (heap_free, heap_min_free, tasks).
Never calls clear_dtc, SD format or delete, OTA, or any pid_def write.
"""
import argparse
import base64
import http.client
import os
import socket
import statistics
import struct
import threading
import time

ENDPOINTS = ["/api/v1/system", "/api/v1/obd2", "/api/v1/pid_def", "/api/v1/pid_data", "/api/v1/dtc",
             "/api/v1/sd_card/info"]
LARGE = "/"  # gzipped frontend, the biggest read-only response


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
    """Minimal RFC 6455 client: sends 0xA0 (start PID stream) and a 0xA2 ping every 2 s, counts 0x02 packets."""

    def __init__(self, host):
        super().__init__(daemon=True)
        self.host, self.stop, self.packets, self.pongs, self.error, self.t0 = host, False, 0, 0, None, 0.0

    def send(self, s, payload):
        mask = os.urandom(4)
        s.sendall(bytes([0x82, 0x80 | len(payload)]) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def run(self):
        try:
            s = socket.create_connection((self.host, 80), timeout=3)
            key = base64.b64encode(os.urandom(16)).decode()
            s.sendall((f"GET /ws HTTP/1.1\r\nHost: {self.host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                       f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
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
                while len(buf) >= 2:
                    n, off = buf[1] & 0x7F, 2
                    if n == 126:
                        if len(buf) < 4:
                            break
                        n, off = struct.unpack(">H", buf[2:4])[0], 4
                    if len(buf) < off + n:
                        break
                    p, buf = buf[off:off + n], buf[off + n:]
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
    ap.add_argument("--ws", type=int, default=3)
    ap.add_argument("--slow", type=int, default=2)
    ap.add_argument("--no-large", action="store_true", help="skip the periodic gzipped GET /")
    a = ap.parse_args()
    host = a.host
    http_get(host, "/api/v1/obd2")
    c = http.client.HTTPConnection(host, 80, timeout=5)
    c.request("POST", "/api/v1/req/auto_poll?auto_poll=true")
    print("poll start", c.getresponse().status)
    print("start   uptime/heap_free/heap_min/lowest stacks:", diag(host))
    ws = [WsClient(host) for _ in range(a.ws)]
    for w in ws:
        w.start()
        time.sleep(1.0)  # opening several WebSockets at the same instant stalls the whole server (see docs/architecture.md)
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
        c.request("POST", "/api/v1/req/auto_poll?auto_poll=false")
        print("poll stop", c.getresponse().status)
        print("slow clients:", {k: slow_counts.count(k) for k in set(slow_counts)})
        print("ws clients:", [(w.packets, w.pongs, w.error) for w in ws])
        print("end     uptime/heap_free/heap_min/lowest stacks:", diag(host))


if __name__ == "__main__":
    main()
