// Dev-only stand-in for the device's CAN bus page. It serves the open REST reads and a WebSocket that
// answers the raw stream (0xB0/0xB1) with synthetic 0x10 batches and 0x11 stats, like docs/api.md describes.
// Not part of the build. Run it, then the dev server against it:
//   bun mock/can-mock-device.ts            (port 8099, or MOCK_PORT=...)
//   CAN_SNIFFER_TARGET=localhost:8099 bun run dev
// Frame values are made up. Nothing here talks to a car.

const PORT = Number(process.env.MOCK_PORT ?? 8099);
const TICK_MS = 20;

type Sim = { id: number; ext: boolean; periodMs: number; make: (tick: number) => number[]; tx?: boolean; silentAfterMs?: number };

const sims: Sim[] = [
  {
    id: 0x0c9,
    ext: false,
    periodMs: 10,
    make: (n) => {
      const rpm = 800 + Math.round(600 * Math.sin(n / 90));
      const raw = rpm * 4;
      return [raw >> 8, raw & 0xff, 0, 0, 0, 0, 0, 0];
    },
  },
  { id: 0x316, ext: false, periodMs: 20, make: (n) => [0x10, Math.round(40 + 30 * Math.sin(n / 60)), 0, 0, 0, 0, 0x2a, 0] },
  { id: 0x545, ext: false, periodMs: 100, make: (n) => [0, 0, n % 40 < 20 ? 0x41 : 0x42, 0x00, 0, 0, 0, 0] },
  { id: 0x5a0, ext: false, periodMs: 50, make: (n) => [n & 0xff, 0xaa, 0, 0, 0, 0, 0, 0] },
  { id: 0x3e0, ext: false, periodMs: 40, make: (n) => [(n * 7) & 0xff, (n * 13) & 0xff, 0, 0] },
  { id: 0x7e8, ext: false, periodMs: 100, make: (n) => [0x04, 0x41, 0x0c, Math.round(20 + 10 * Math.sin(n / 30)) & 0xff, 0, 0, 0, 0] },
  { id: 0x7e9, ext: false, periodMs: 500, make: () => [0x04, 0x41, 0x0c, 0x1a, 0, 0, 0, 0] },
  { id: 0x7eb, ext: false, periodMs: 1000, make: () => [0x03, 0x41, 0x05, 0x40, 0, 0, 0, 0], silentAfterMs: 20_000 },
  { id: 0x18daf110, ext: true, periodMs: 1000, make: () => [0x02, 0x62, 0x01, 0x00, 0, 0, 0, 0] },
  { id: 0x7df, ext: false, periodMs: 1000, make: () => [0x02, 0x01, 0x0c, 0, 0, 0, 0, 0], tx: true },
];

type Socket = {
  raw: boolean;
  timer: ReturnType<typeof setInterval> | null;
  tick: ReturnType<typeof setInterval> | undefined;
  started: number;
  lastSecond: number;
  frames: number;
  dropped: number;
};

const json = (body: unknown, status = 200) =>
  new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json", "Cache-Control": "no-store" } });

function rest(url: URL): Response {
  switch (url.pathname) {
    case "/api/v1/can_bus":
      return json({
        state: "connected",
        bitrate: 500000,
        rs_mode: "high_speed",
        debug_mode: false,
        initialized: true,
        bus_connected: true,
        node_status: { twai_error_state: "error_active", tx_error_count: 0, rx_error_count: 0 },
      });
    case "/api/v1/can/mode":
      return json({ mode: "monitor", isotp_flow_control: false, obd_parsing: true, modes: ["monitor", "raw"] });
    default:
      return json({ status: "error", reason: "mock: not implemented" }, 404);
  }
}

/** One 0x10 batch: up to 16 records, times relative to the first. */
function batchBytes(frames: { t: number; id: number; ext: boolean; tx: boolean; data: number[] }[], baseTs: number, dropped: number): Uint8Array {
  const body: number[] = [];
  let prev = frames[0].t;
  for (const f of frames) {
    const dt = f.t - prev;
    prev = f.t;
    const word = (f.id >>> 0) | (f.ext ? 0x80000000 : 0);
    const ctl = f.data.length | (f.tx ? 0x80 : 0);
    body.push(dt & 0xff, (dt >> 8) & 0xff, word & 0xff, (word >> 8) & 0xff, (word >> 16) & 0xff, (word >>> 24) & 0xff, ctl, ...f.data);
  }
  const head = [0x10, 0, dropped & 0xff, (dropped >> 8) & 0xff, baseTs & 0xff, (baseTs >>> 8) & 0xff, (baseTs >>> 16) & 0xff, (baseTs >>> 24) & 0xff];
  const out = new Uint8Array([...head, ...body]);
  out[1] = out.length - 2;
  return out;
}

function statsBytes(s: Socket, now: number): Uint8Array {
  const b = new Uint8Array(25);
  const v = new DataView(b.buffer);
  b[0] = 0x11;
  b[1] = 23;
  b[2] = 0x01;
  v.setUint32(3, s.frames, true);
  v.setUint32(7, 0, true);
  v.setUint32(11, 0, true);
  v.setUint32(15, s.dropped, true);
  v.setUint16(19, 256, true);
  v.setUint16(21, 12, true);
  v.setUint16(23, 270 + Math.round(40 * Math.sin(now / 8000)), true);
  return b;
}

function statusBytes(): Uint8Array {
  const b = new Uint8Array(12);
  const v = new DataView(b.buffer);
  b[0] = 0x03;
  b[1] = 10;
  b[2] = 3;
  v.setFloat32(3, 0.27, true);
  v.setFloat32(7, 12.6, true);
  b[11] = 1;
  return b;
}

Bun.serve<Socket>({
  port: PORT,
  fetch(req, server) {
    const url = new URL(req.url);
    if (url.pathname === "/ws") {
      const ok = server.upgrade(req, { data: { raw: false, timer: null, tick: undefined, started: 0, lastSecond: 0, frames: 0, dropped: 0 } });
      return ok ? undefined : new Response("upgrade failed", { status: 400 });
    }
    return rest(url);
  },
  websocket: {
    open(ws) {
      ws.data.timer = setInterval(() => {
        const now = Date.now();
        if (now - ws.data.lastSecond >= 1000) {
          ws.data.lastSecond = now;
          ws.send(statusBytes());
          if (ws.data.raw) ws.send(statsBytes(ws.data, now));
        }
      }, 50);
    },
    message(ws, msg) {
      const b = new Uint8Array(msg as ArrayBuffer);
      if (b.length !== 1) return;
      if (b[0] === 0xa2) ws.send(new Uint8Array([0xa3]));
      if (b[0] === 0xb0 && !ws.data.raw) {
        ws.data.raw = true;
        ws.data.started = Date.now();
        ws.data.lastSecond = 0;
        ws.data.frames = 0;
        ws.data.dropped = 0;
        startRaw(ws);
      }
      if (b[0] === 0xb1) stopRaw(ws);
    },
    close(ws) {
      if (ws.data.timer) clearInterval(ws.data.timer);
      stopRaw(ws);
    },
  },
});

function startRaw(ws: { data: Socket; send(b: Uint8Array): unknown }) {
  const next = sims.map((s) => ({ s, due: 0 }));
  const tickTimer = setInterval(() => {
    const now = Date.now();
    const elapsed = now - ws.data.started;
    const due: { t: number; id: number; ext: boolean; tx: boolean; data: number[] }[] = [];
    for (const n of next) {
      if (n.s.silentAfterMs !== undefined && elapsed > n.s.silentAfterMs) continue;
      while (n.due <= elapsed) {
        const count = Math.floor(n.due / n.s.periodMs);
        due.push({ t: n.due * 1000, id: n.s.id, ext: n.s.ext, tx: n.s.tx === true, data: n.s.make(count) });
        n.due += n.s.periodMs;
        if (due.length > 400) break;
      }
    }
    due.sort((a, b) => a.t - b.t);
    const baseUs = (Date.now() * 1000) >>> 0;
    // A batch holds 16 frames and spans under 30 ms (dt is int16 µs), so a slipped tick splits into more batches.
    let i = 0;
    while (i < due.length) {
      let end = i + 1;
      while (end < due.length && end - i < 16 && due[end].t - due[i].t < 30_000) end++;
      const chunk = due.slice(i, end);
      const rebased = chunk.map((f) => ({ ...f, t: f.t - chunk[0].t }));
      ws.data.frames += chunk.length;
      ws.send(batchBytes(rebased, (baseUs + Math.round(chunk[0].t / 1000)) >>> 0, 0));
      i = end;
    }
  }, TICK_MS);
  ws.data.tick = tickTimer;
}

function stopRaw(ws: { data: Socket }) {
  ws.data.raw = false;
  if (ws.data.tick) clearInterval(ws.data.tick);
  ws.data.tick = undefined;
}

console.log(`CAN mock device on http://localhost:${PORT} (WebSocket /ws)`);
