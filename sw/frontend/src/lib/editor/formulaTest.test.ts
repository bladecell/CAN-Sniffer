import { describe, expect, test } from "bun:test";
import { evaluateFormula, parseTestBytes } from "./formulaTest";

describe("parseTestBytes", () => {
  test("accepts spaced, comma or lower-case hex, up to 8 bytes", () => {
    expect(parseTestBytes("0A 3C")).toEqual({ ok: true, bytes: [10, 60] });
    expect(parseTestBytes("0a,3c")).toEqual({ ok: true, bytes: [10, 60] });
    expect(parseTestBytes("00 01 02 03 04 05 06 07")).toMatchObject({ ok: true });
  });
  test("empty input is reported as empty, not as an error", () => {
    expect(parseTestBytes("   ")).toEqual({ ok: false, reason: "empty" });
  });
  test("rejects bad digits, 3-digit groups and more than 8 bytes", () => {
    expect(parseTestBytes("0A ZZ")).toMatchObject({ ok: false });
    expect(parseTestBytes("1FF")).toMatchObject({ ok: false });
    expect(parseTestBytes("00 01 02 03 04 05 06 07 08")).toMatchObject({ ok: false });
  });
});

describe("evaluateFormula", () => {
  test("worked examples from the firmware formula rules", () => {
    expect(evaluateFormula("A - 40", [120])).toEqual({ ok: true, value: 80 });
    expect(evaluateFormula("((A*256)+B)/4", [0x0b, 0xb8])).toEqual({ ok: true, value: 750 });
    expect(evaluateFormula("A*100/255", [255])).toEqual({ ok: true, value: 100 });
  });
  test("bytes past the end of the frame read as 0", () => {
    expect(evaluateFormula("C + 1", [5])).toEqual({ ok: true, value: 1 });
  });
  test("power uses ^ as in TinyExpr", () => {
    expect(evaluateFormula("2^3", [])).toEqual({ ok: true, value: 8 });
  });
  test("getBit reads bit n, 0 above bit 31", () => {
    expect(evaluateFormula("getBit(A, 3)", [0b1000])).toEqual({ ok: true, value: 1 });
    expect(evaluateFormula("getBit(A, 2)", [0b1000])).toEqual({ ok: true, value: 0 });
    expect(evaluateFormula("getBit(A, 40)", [0xff])).toEqual({ ok: true, value: 0 });
  });
  test("getPID needs live data, so the test says so instead of guessing", () => {
    const r = evaluateFormula("getPID(0x0C) / 4", [1]);
    expect(r.ok).toBe(false);
    expect(r.ok === false && r.reason).toContain("getPID");
  });
  test("division by zero, NaN and infinity fail, as the firmware marks them invalid", () => {
    expect(evaluateFormula("A / 0", [1]).ok).toBe(false);
    expect(evaluateFormula("0 / 0", []).ok).toBe(false);
  });
  test("unknown names, member access and empty formulas are rejected", () => {
    expect(evaluateFormula("foo + A", [1]).ok).toBe(false);
    expect(evaluateFormula("A.constructor", [1]).ok).toBe(false);
    expect(evaluateFormula("A; alert(1)", [1]).ok).toBe(false);
    expect(evaluateFormula("   ", [1]).ok).toBe(false);
  });
});
