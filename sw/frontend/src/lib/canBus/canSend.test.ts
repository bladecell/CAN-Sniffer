import { describe, expect, test } from "bun:test";
import { moveAt, parseIntIn, parseSendData, parseSendId, planBody, reorderBy } from "./canSend";

describe("parseSendId", () => {
  test("standard ids only", () => {
    expect(parseSendId("7DF")).toEqual({ ok: true, value: 0x7df });
    expect(parseSendId("7ff")).toEqual({ ok: true, value: 0x7ff });
    expect(parseSendId("800")).toMatchObject({ ok: false });
    expect(parseSendId("18DAF110")).toMatchObject({ ok: false });
    expect(parseSendId("")).toMatchObject({ ok: false });
    expect(parseSendId("7G")).toMatchObject({ ok: false });
  });
});

describe("parseSendData", () => {
  test("accepts 1 to 8 bytes with or without spaces, normalised", () => {
    expect(parseSendData("02 01 0c")).toEqual({ ok: true, value: "02 01 0C" });
    expect(parseSendData("0201")).toEqual({ ok: true, value: "02 01" });
    expect(parseSendData("02 01 0C 00 00 00 00 00")).toMatchObject({ ok: true });
  });

  test("bad hex, odd digits, empty and a 9-byte payload send nothing", () => {
    expect(parseSendData("02 0")).toMatchObject({ ok: false });
    expect(parseSendData("zz")).toMatchObject({ ok: false });
    expect(parseSendData("")).toMatchObject({ ok: false });
    expect(parseSendData("00 01 02 03 04 05 06 07 08")).toMatchObject({ ok: false });
  });
});

describe("parseIntIn and planBody", () => {
  test("whole numbers in range", () => {
    expect(parseIntIn("10", 10, 3600000)).toBe(10);
    expect(parseIntIn("9", 10, 3600000)).toBeNull();
    expect(parseIntIn("1.5", 0, 9)).toBeNull();
    expect(parseIntIn("0", 0, 65535)).toBe(0);
  });

  test("plan body has only the documented keys", () => {
    expect(planBody([{ id: 0x7e0, data: "02 01 0C" }], 100, 0)).toEqual({
      frames: [{ id: 0x7e0, data: "02 01 0C" }],
      period_ms: 100,
      repeat: 0,
    });
  });
});

describe("sequence order", () => {
  const list = [{ k: "a" }, { k: "b" }, { k: "c" }];
  test("moveAt swaps with the neighbour and ignores out of range", () => {
    expect(moveAt(list, 0, 1).map((x) => x.k)).toEqual(["b", "a", "c"]);
    expect(moveAt(list, 0, -1)).toBe(list);
    expect(moveAt(list, 2, 1)).toBe(list);
  });
  test("reorderBy follows the ids and refuses a partial list", () => {
    expect(reorderBy(list, (x) => x.k, ["c", "a", "b"]).map((x) => x.k)).toEqual(["c", "a", "b"]);
    expect(reorderBy(list, (x) => x.k, ["c", "a"])).toBe(list);
    expect(reorderBy(list, (x) => x.k, ["c", "a", "zzz"])).toBe(list);
  });
});
