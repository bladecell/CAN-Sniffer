import { describe, expect, test } from "bun:test";
import { byIdRows, CanModel, changedMask, ECU_AGE_MS, ecuRows, logRows, MAX_IDS, TABLE_ROWS } from "./canModel";
import { compileRules, type Rule } from "./canRules";
import { ecuRequestId, isObdId } from "./canIds";
import type { RawBatch, RawRecord } from "../wsFrames";

const rec = (id: number, data: number[], dt = 0, opts: Partial<RawRecord> = {}): RawRecord => ({
  dt,
  id,
  ext: false,
  rtr: false,
  tx: false,
  data: new Uint8Array(data),
  ...opts,
});
const batch = (records: RawRecord[], baseTs = 0): RawBatch => ({ dropped: 0, baseTs, records });

describe("time", () => {
  test("base plus running dt gives each frame its time", () => {
    const m = new CanModel();
    m.ingest(batch([rec(1, [0]), rec(1, [0], 100), rec(1, [0], -30)], 1000), 0);
    expect(m.log.map((r) => r.t)).toEqual([1000, 1100, 1070]);
    expect(m.startUs).toBe(1000);
  });

  test("the 32-bit device clock wraps without a jump", () => {
    const m = new CanModel();
    m.ingest(batch([rec(1, [0])], 0xfffffff0), 0);
    m.ingest(batch([rec(1, [0], 0x20)], 0x10), 0);
    expect(m.log[1].t - m.log[0].t).toBe(0x20 + 0x20);
    expect(m.log[1].t).toBeGreaterThan(m.log[0].t);
  });
});

describe("changed bytes and ids", () => {
  test("changedMask marks differing bytes; first frame has none", () => {
    expect(changedMask([1, 2, 3], [1, 9, 3])).toBe(0b010);
    expect(changedMask([1], [1, 0, 0])).toBe(0b110);
  });

  test("the id entry tracks the last change, count and recent frames", () => {
    const m = new CanModel();
    m.ingest(batch([rec(0x100, [1, 2]), rec(0x100, [1, 3]), rec(0x100, [1, 3])]), 5000);
    const e = m.ids.get(0x100)!;
    expect(e.count).toBe(3);
    expect(e.changed).toBe(0);
    expect(e.recent.length).toBe(3);
    expect(e.seenMs).toBe(5000);
    expect(m.log.map((r) => r.changed)).toEqual([0, 0b10, 0]);
  });

  test("standard and extended with the same number are separate ids", () => {
    const m = new CanModel();
    m.ingest(batch([rec(0x7e8, [1]), rec(0x7e8, [1], 0, { ext: true })]), 0);
    expect(m.ids.size).toBe(2);
  });

  test("ids past the cap are counted and logged, not listed", () => {
    const m = new CanModel();
    const records = Array.from({ length: MAX_IDS + 5 }, (_, i) => rec(i, [0]));
    for (let i = 0; i < records.length; i += 16) m.ingest(batch(records.slice(i, i + 16)), 0);
    expect(m.ids.size).toBe(MAX_IDS);
    expect(m.untracked).toBe(5);
    expect(m.frames).toBe(MAX_IDS + 5);
  });
});

describe("log and table", () => {
  test("the log keeps about two tables of rows and the table shows the newest 500", () => {
    const m = new CanModel();
    for (let i = 0; i < 1500; i++) m.ingest(batch([rec(0x100 + (i % 3), [i & 0xff])]), 0);
    expect(m.log.length).toBeLessThanOrEqual(TABLE_ROWS * 2);
    const rows = logRows(m, "all");
    expect(rows.length).toBe(TABLE_ROWS);
    expect(rows[0].data[0]).toBe(1499 & 0xff);
  });

  test("hidden rows are logged but not shown; changed and obd filters", () => {
    const rules: Rule[] = [{ id: "h", name: "", match: { id: 0x5a0, ext: false, idMask: 0x7ff, bytes: [] }, formula: "" }];
    const m = new CanModel();
    m.setRules(compileRules(rules));
    m.ingest(batch([rec(0x5a0, [1]), rec(0x7e8, [1]), rec(0x7e8, [2])]), 0);
    expect(m.log.length).toBe(3);
    expect(logRows(m, "all").map((r) => r.id)).toEqual([0x7e8, 0x7e8]);
    expect(logRows(m, "changed").map((r) => r.data[0])).toEqual([2]);
    expect(logRows(m, "obd").length).toBe(2);
  });

  test("a formula rule fills the parsed column", () => {
    const m = new CanModel();
    m.setRules(compileRules([{ id: "e", name: "Engine speed", match: { id: 0xc9, ext: false, idMask: 0x7ff, bytes: [] }, formula: "(A*256+B)/4" }]));
    m.ingest(batch([rec(0xc9, [0x0b, 0xb8])]), 0);
    expect(m.log[0].parsed).toBe("Engine speed 750");
  });
});

describe("rates", () => {
  test("tick gives each id its frames per second and the frames per second history", () => {
    const m = new CanModel();
    m.ingest(batch([rec(1, [0]), rec(1, [0]), rec(2, [0])]), 0);
    m.tick(2);
    expect(m.ids.get(1)!.hz).toBe(1);
    expect(m.ids.get(2)!.hz).toBe(0.5);
    expect(m.fps).toEqual([1.5]);
    m.tick(1);
    expect(m.ids.get(1)!.hz).toBe(0);
  });
});

describe("ecu ids (passive)", () => {
  test("OBD responses map to their request ids", () => {
    expect(ecuRequestId(0x7e8, false)).toBe(0x7e0);
    expect(ecuRequestId(0x7ef, false)).toBe(0x7e7);
    expect(ecuRequestId(0x7f0, false)).toBeNull();
    expect(ecuRequestId(0x18daf110, true)).toBe(0x18da10f1);
    expect(ecuRequestId(0x18daf110, false)).toBeNull();
  });

  test("ecuRows lists only response-pattern ids, sorted", () => {
    const m = new CanModel();
    m.ingest(batch([rec(0x7e9, [1]), rec(0x7e8, [1, 2]), rec(0x3e8, [1]), rec(0x18daf110, [1], 0, { ext: true })]), 0);
    expect(ecuRows(m, 0).map((r) => r.id)).toEqual([0x7e8, 0x7e9, 0x18daf110]);
  });

  test("an ECU silent past the age limit leaves the list", () => {
    const m = new CanModel();
    m.ingest(batch([rec(0x7e8, [1])]), 1000);
    expect(ecuRows(m, 1000 + ECU_AGE_MS).length).toBe(1);
    expect(ecuRows(m, 1000 + ECU_AGE_MS + 1).length).toBe(0);
  });

  test("OBD id detection", () => {
    expect(isObdId(0x7df, false)).toBe(true);
    expect(isObdId(0x3e8, false)).toBe(false);
    expect(isObdId(0x18db33f1, true)).toBe(true);
  });
});

describe("by-id table", () => {
  test("one row per shown id, sorted, with the last frame and changed bytes", () => {
    const m = new CanModel();
    m.setRules(compileRules([{ id: "h", name: "", match: { id: 0x5a0, ext: false, idMask: 0x7ff, bytes: [] }, formula: "" }]));
    m.ingest(batch([rec(0x300, [1]), rec(0x100, [1, 2]), rec(0x100, [1, 9]), rec(0x5a0, [0])]), 0);
    const rows = byIdRows(m, "all");
    expect(rows.map((r) => r.id)).toEqual([0x100, 0x300]);
    expect(rows[0].data[1]).toBe(9);
    expect(rows[0].changed).toBe(0b10);
    expect(byIdRows(m, "changed").map((r) => r.id)).toEqual([0x100]);
  });

  test("capped at the table size", () => {
    const m = new CanModel();
    for (let i = 0; i < 700; i++) m.ingest(batch([rec(i, [0])]), 0);
    expect(byIdRows(m, "all").length).toBe(TABLE_ROWS);
  });
});
