import { describe, expect, test } from "bun:test";
import { compileFilters, parseFilterValue, type FilterRule } from "./canFilter";

const rule = (field: FilterRule["field"], op: FilterRule["op"], value: string, exclude = false): FilterRule => ({ key: 0, field, op, value, exclude });
const f = (id: number, data: number[], tx = false) => ({ id, tx, data });

describe("canFilter", () => {
  test("parses per field", () => {
    expect(parseFilterValue("id", "7e8")).toBe(0x7e8);
    expect(parseFilterValue("id", "xyz")).toBeNull();
    expect(parseFilterValue("dlc", "8")).toBe(8);
    expect(parseFilterValue("dir", "tx")).toBe(1);
    expect(parseFilterValue("b0", "")).toBeNull();
  });

  test("operators", () => {
    const hit = (op: FilterRule["op"]) => compileFilters([rule("id", op, "100")])(f(0x100, []));
    expect([hit("="), hit("!="), hit(">"), hit("<"), hit(">="), hit("<=")]).toEqual([true, false, false, false, true, true]);
  });

  test("include rules are ANDed, exclude rules hide a match", () => {
    const p = compileFilters([rule("id", ">=", "100"), rule("dlc", "=", "8", true)]);
    expect(p(f(0x200, [1, 2]))).toBe(true);
    expect(p(f(0x200, [1, 2, 3, 4, 5, 6, 7, 8]))).toBe(false);
    expect(p(f(0x50, [1]))).toBe(false);
  });

  test("a byte the frame lacks matches nothing; bad values are ignored", () => {
    expect(compileFilters([rule("b3", "=", "0")])(f(1, [0]))).toBe(false);
    expect(compileFilters([rule("b3", "=", "0", true)])(f(1, [0]))).toBe(true);
    expect(compileFilters([rule("id", "=", "zz")])(f(1, []))).toBe(true);
  });

  test("direction", () => {
    expect(compileFilters([rule("dir", "=", "TX")])(f(1, [], true))).toBe(true);
    expect(compileFilters([rule("dir", "=", "TX")])(f(1, [], false))).toBe(false);
  });
});
