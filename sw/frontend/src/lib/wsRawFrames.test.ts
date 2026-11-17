import { describe, expect, test } from "bun:test";
import { parseWsFrame } from "./wsFrames";

type Rec = { dt: number; id: number; ext?: boolean; rtr?: boolean; tx?: boolean; data?: number[] };

/** Builds a 0x10 batch exactly as docs/api.md describes it. */
function batch(records: Rec[], dropped = 0, baseTs = 1000) {
  const body: number[] = [];
  for (const r of records) {
    const data = r.data ?? [];
    const word = (r.id >>> 0) | (r.ext ? 0x80000000 : 0) | (r.rtr ? 0x40000000 : 0);
    const ctl = data.length | (r.tx ? 0x80 : 0);
    const dt = r.dt & 0xffff;
    body.push(dt & 0xff, dt >> 8, word & 0xff, (word >> 8) & 0xff, (word >> 16) & 0xff, (word >>> 24) & 0xff, ctl, ...data);
  }
  const head = [0x10, 0, dropped & 0xff, dropped >> 8, baseTs & 0xff, (baseTs >> 8) & 0xff, (baseTs >> 16) & 0xff, (baseTs >>> 24) & 0xff];
  const bytes = [...head, ...body];
  bytes[1] = bytes.length - 2;
  return new Uint8Array(bytes).buffer;
}

function stats(overrides: Partial<Record<string, number>> = {}, flags = 1, load = 250) {
  const b = new Uint8Array(25);
  const v = new DataView(b.buffer);
  b[0] = 0x11;
  b[1] = 23;
  b[2] = flags;
  v.setUint32(3, overrides.streamed ?? 42, true);
  v.setUint32(7, overrides.driver ?? 1, true);
  v.setUint32(11, overrides.ring ?? 2, true);
  v.setUint32(15, overrides.socket ?? 3, true);
  v.setUint16(19, 256, true);
  v.setUint16(21, 17, true);
  v.setUint16(23, load, true);
  return b.buffer;
}

describe("parseWsFrame: 0x10 raw batch", () => {
  test("reads header, records, extended flag, RTR, TX bit and data", () => {
    const frame = parseWsFrame(
      batch(
        [
          { dt: 0, id: 0x7e8, data: [0x04, 0x41, 0x0c, 0x1a] },
          { dt: -5, id: 0x18daf110, ext: true, tx: true, data: [1, 2, 3, 4, 5, 6, 7, 8] },
          { dt: 300, id: 0x123, rtr: true },
        ],
        9,
        0xfffffff0,
      ),
    );
    expect(frame).toEqual({
      kind: "rawBatch",
      dropped: 9,
      baseTs: 0xfffffff0,
      records: [
        { dt: 0, id: 0x7e8, ext: false, rtr: false, tx: false, data: new Uint8Array([4, 0x41, 0x0c, 0x1a]) },
        { dt: -5, id: 0x18daf110, ext: true, rtr: false, tx: true, data: new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8]) },
        { dt: 300, id: 0x123, ext: false, rtr: true, tx: false, data: new Uint8Array([]) },
      ],
    });
  });

  test("accepts the largest batch: 16 full frames", () => {
    const recs = Array.from({ length: 16 }, (_, i) => ({ dt: i, id: 0x100 + i, data: [1, 2, 3, 4, 5, 6, 7, 8] }));
    const buf = batch(recs);
    expect(buf.byteLength).toBe(8 + 16 * 15);
    const frame = parseWsFrame(buf);
    expect(frame?.kind === "rawBatch" && frame.records.length).toBe(16);
  });

  test("rejects a 17th record, a truncated record, a bad length byte and reserved bits", () => {
    const recs = Array.from({ length: 17 }, (_, i) => ({ dt: 0, id: i, data: [] }));
    expect(parseWsFrame(batch(recs))).toBeNull();

    const good = new Uint8Array(batch([{ dt: 0, id: 1, data: [1, 2] }]));
    expect(parseWsFrame(good.slice(0, good.length - 1).buffer)).toBeNull();

    const badLen = new Uint8Array(batch([{ dt: 0, id: 1 }]));
    badLen[1] = 99;
    expect(parseWsFrame(badLen.buffer)).toBeNull();

    const reservedCtl = new Uint8Array(batch([{ dt: 0, id: 1 }]));
    reservedCtl[8 + 6] = 0x10;
    expect(parseWsFrame(reservedCtl.buffer)).toBeNull();

    const reservedId = new Uint8Array(batch([{ dt: 0, id: 1, ext: true }]));
    reservedId[8 + 5] |= 0x20;
    expect(parseWsFrame(reservedId.buffer)).toBeNull();
  });

  test("rejects a standard id above 11 bits and a DLC above 8", () => {
    expect(parseWsFrame(batch([{ dt: 0, id: 0x800 }]))).toBeNull();
    const dlc9 = new Uint8Array(batch([{ dt: 0, id: 1 }]));
    dlc9[8 + 6] = 9;
    expect(parseWsFrame(dlc9.buffer)).toBeNull();
  });

  test("rejects an empty batch and anything over 257 bytes", () => {
    expect(parseWsFrame(new Uint8Array([0x10, 0, 0, 0, 0, 0, 0, 0]).buffer)).toBeNull();
    const big = new Uint8Array(258);
    big[0] = 0x10;
    expect(parseWsFrame(big.buffer)).toBeNull();
  });
});

describe("parseWsFrame: 0x11 raw stats", () => {
  test("reads flags, counters and bus load", () => {
    expect(parseWsFrame(stats({ streamed: 7 }, 3, 512))).toEqual({
      kind: "rawStats",
      active: true,
      filterOn: true,
      streamed: 7,
      droppedDriver: 1,
      droppedRing: 2,
      droppedSocket: 3,
      ringCapacity: 256,
      ringPeak: 17,
      busLoadPermille: 512,
    });
  });

  test("0xFFFF load is unknown, and a load above 1000 is clamped", () => {
    expect(parseWsFrame(stats({}, 1, 0xffff))).toMatchObject({ busLoadPermille: null });
    expect(parseWsFrame(stats({}, 1, 1400))).toMatchObject({ busLoadPermille: 1000 });
  });

  test("rejects the wrong size or length byte", () => {
    const short = new Uint8Array(24);
    short[0] = 0x11;
    short[1] = 22;
    expect(parseWsFrame(short.buffer)).toBeNull();
  });
});
