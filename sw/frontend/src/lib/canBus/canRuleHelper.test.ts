import { describe, expect, test } from "bun:test";
import { helperFormula } from "./canRuleHelper";
import { compileFormula, FRAME_LETTERS } from "../editor/formulaTest";

const run = (formula: string, bytes: number[]) => {
  const c = compileFormula(formula, FRAME_LETTERS);
  return c.ok ? c.run(bytes) : c;
};

describe("helperFormula", () => {
  test("one byte is the byte itself", () => {
    expect(helperFormula({ start: 2, length: 1, order: "be" })).toEqual({ ok: true, formula: "C" });
  });

  test("two bytes big and little endian", () => {
    expect(helperFormula({ start: 0, length: 2, order: "be" })).toEqual({ ok: true, formula: "A*256+B" });
    expect(helperFormula({ start: 0, length: 2, order: "le" })).toEqual({ ok: true, formula: "A+B*256" });
  });

  test("the written formula reads the same value as the bytes say", () => {
    const bytes = [0x0b, 0xb8, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
    const be = helperFormula({ start: 0, length: 4, order: "be" });
    expect(be.ok && run(be.formula, bytes)).toEqual({ ok: true, value: 0x0bb80102 });
    const le = helperFormula({ start: 2, length: 3, order: "le" });
    expect(le.ok && run(le.formula, bytes)).toEqual({ ok: true, value: 0x030201 });
  });

  test("refuses a start or length that does not fit", () => {
    expect(helperFormula({ start: 8, length: 1, order: "be" })).toMatchObject({ ok: false });
    expect(helperFormula({ start: 0, length: 0, order: "be" })).toMatchObject({ ok: false });
    expect(helperFormula({ start: 0, length: 5, order: "be" })).toMatchObject({ ok: false });
    expect(helperFormula({ start: 6, length: 3, order: "be" })).toMatchObject({ ok: false });
    expect(helperFormula({ start: 1.5, length: 1, order: "be" })).toMatchObject({ ok: false });
  });
});
