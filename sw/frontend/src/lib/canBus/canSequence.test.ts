import { describe, expect, test } from "bun:test";
import { PERIOD_SNAP, REPEAT_SNAP, parseSequence, periodBrief, periodKey, periodToPos, posToPeriod, posToRepeat, repeatBrief, repeatKey, repeatToPos, sequenceSummary, serializeSequence } from "./canSend";
import { isSeqName, parseSeqList, seqPath } from "./canSequenceFiles";

describe("period slider", () => {
  test("ends of the track are 10 ms and 1 h", () => {
    expect(periodToPos(10)).toBe(0);
    expect(periodToPos(3_600_000)).toBe(1);
    expect(posToPeriod(0)).toBe(10);
    expect(posToPeriod(1)).toBe(3_600_000);
  });
  test("every snap value maps back to itself", () => {
    for (const s of PERIOD_SNAP) expect(posToPeriod(periodToPos(s))).toBe(s);
  });
  test("positions out of range clamp, values stay in 10..3600000", () => {
    expect(posToPeriod(-3)).toBe(10);
    expect(posToPeriod(9)).toBe(3_600_000);
    for (let x = 0; x <= 1; x += 0.01) {
      const v = posToPeriod(x);
      expect(v).toBeGreaterThanOrEqual(10);
      expect(v).toBeLessThanOrEqual(3_600_000);
    }
  });
  test("keys step, jump and clamp", () => {
    expect(periodKey(100, "ArrowUp")).toBe(150);
    expect(periodKey(10, "ArrowDown")).toBe(10);
    expect(periodKey(3_600_000, "ArrowUp")).toBe(3_600_000);
    expect(periodKey(100, "PageUp")).toBe(250);
    expect(periodKey(100, "PageDown")).toBe(50);
    expect(periodKey(100, "Home")).toBe(10);
    expect(periodKey(100, "End")).toBe(3_600_000);
    expect(periodKey(100, "a")).toBeUndefined();
  });
  test("readout", () => {
    expect([periodBrief(10), periodBrief(1500), periodBrief(90_000), periodBrief(3_600_000)]).toEqual(["10 ms", "1.5 s", "1.5 min", "1 h"]);
  });
});

describe("repeat slider", () => {
  test("Once at the left, Endless (0) at the right stop", () => {
    expect(repeatToPos(1)).toBe(0);
    expect(repeatToPos(0)).toBe(1);
    expect(posToRepeat(0)).toBe(1);
    expect(posToRepeat(1)).toBe(0);
    expect(posToRepeat(0.95)).toBe(0);
  });
  test("65535 sits just before the stop and is not Endless", () => {
    expect(repeatToPos(65535)).toBeCloseTo(0.88, 5);
    expect(posToRepeat(0.88)).toBe(65535);
  });
  test("snap values round-trip; every position gives 0..65535", () => {
    for (const s of REPEAT_SNAP) expect(posToRepeat(repeatToPos(s))).toBe(s);
    for (let x = -0.1; x <= 1.1; x += 0.01) {
      const v = posToRepeat(x);
      expect(v).toBeGreaterThanOrEqual(0);
      expect(v).toBeLessThanOrEqual(65535);
    }
  });
  test("keys wrap through Endless", () => {
    expect(repeatKey(1, "ArrowUp")).toBe(2);
    expect(repeatKey(65535, "ArrowUp")).toBe(0);
    expect(repeatKey(0, "ArrowDown")).toBe(65535);
    expect(repeatKey(1, "ArrowDown")).toBe(1);
    expect(repeatKey(5, "Home")).toBe(1);
    expect(repeatKey(5, "End")).toBe(0);
    expect(repeatKey(0, "PageUp")).toBe(0);
  });
  test("labels and summary", () => {
    expect([repeatBrief(0), repeatBrief(1), repeatBrief(7)]).toEqual(["Endless", "Once", "7 times"]);
    expect(sequenceSummary(1, 100, 1)).toBe("1 frame, every 100 ms, once");
    expect(sequenceSummary(3, 1000, 0)).toBe("3 frames, every 1 s, until stopped");
    expect(sequenceSummary(2, 50, 4)).toBe("2 frames, every 50 ms, 4 rounds");
  });
});

describe("sequence file", () => {
  const ok = { frames: [{ id: 0x7df, data: "02 01 0C" }], periodMs: 100, repeat: 0 };
  test("serialize matches the POST /can/send body and parses back", () => {
    expect(JSON.parse(serializeSequence(ok))).toEqual({ frames: [{ id: 2015, data: "02 01 0C" }], period_ms: 100, repeat: 0 });
    expect(parseSequence(serializeSequence(ok))).toEqual({ ok: true, value: ok });
  });
  test("data is normalised, repeat defaults to 1", () => {
    const r = parseSequence('{"frames":[{"id":1,"data":"0a0b"}],"period_ms":10}');
    expect(r).toEqual({ ok: true, value: { frames: [{ id: 1, data: "0A 0B" }], periodMs: 10, repeat: 1 } });
  });
  const bad = (doc: unknown) => parseSequence(typeof doc === "string" ? doc : JSON.stringify(doc)).ok;
  const f = { id: 1, data: "00" };
  test("hostile files are refused", () => {
    expect(bad("nope")).toBe(false);
    expect(bad("[]")).toBe(false);
    expect(bad("null")).toBe(false);
    expect(bad({ frames: [], period_ms: 100 })).toBe(false);
    expect(bad({ frames: Array(33).fill(f), period_ms: 100 })).toBe(false);
    expect(bad({ frames: Array(32).fill(f), period_ms: 100 })).toBe(true);
    expect(bad({ frames: [{ id: 2048, data: "00" }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [{ id: -1, data: "00" }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [{ id: 1.5, data: "00" }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [{ id: "7DF", data: "00" }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [{ id: 1, data: "" }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [{ id: 1, data: "00 11 22 33 44 55 66 77 88" }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [{ id: 1, data: "ZZ" }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [{ id: 1, data: 5 }], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [null], period_ms: 100 })).toBe(false);
    expect(bad({ frames: [f], period_ms: 9 })).toBe(false);
    expect(bad({ frames: [f], period_ms: 3_600_001 })).toBe(false);
    expect(bad({ frames: [f] })).toBe(false);
    expect(bad({ frames: [f], period_ms: 100, repeat: 65536 })).toBe(false);
    expect(bad({ frames: [f], period_ms: 100, repeat: -1 })).toBe(false);
    expect(bad({ frames: [f], period_ms: 100, repeat: "1" })).toBe(false);
    expect(bad({ frames: [f], period_ms: 100, repeat: 65535 })).toBe(true);
    expect(bad(" ".repeat(20_000))).toBe(false);
  });
});

describe("saved sequence names", () => {
  test("allow-list", () => {
    for (const n of ["a", "Test_1-b", "x".repeat(32)]) expect(isSeqName(n)).toBe(true);
    for (const n of ["", "x".repeat(33), "a b", "a.b", "../x", "a/b", "é", "a\n"]) expect(isSeqName(n)).toBe(false);
    expect(seqPath("rpm")).toBe("config/send/rpm.json");
  });
  test("list keeps json files with allowed names, sorted", () => {
    const tree = {
      path: "/config/send",
      children: [
        { name: "b.json", type: "file", size: 9 },
        { name: "a.json", type: "file", size: 9 },
        { name: "c.txt", type: "file", size: 1 },
        { name: "bad name.json", type: "file", size: 1 },
        { name: "dir.json", type: "directory" },
        { path: "/config/send/sub", children: [] },
        null,
        { name: 5, type: "file" },
      ],
    };
    expect(parseSeqList(tree)).toEqual(["a", "b"]);
  });
  test("hostile replies give an empty list", () => {
    for (const t of [null, undefined, 5, "x", {}, { children: "x" }, { children: {} }]) expect(parseSeqList(t)).toEqual([]);
  });
  test("list is capped", () => {
    const children = Array.from({ length: 300 }, (_, i) => ({ name: `s${String(i).padStart(3, "0")}.json`, type: "file" }));
    expect(parseSeqList({ children })).toHaveLength(100);
  });
});
