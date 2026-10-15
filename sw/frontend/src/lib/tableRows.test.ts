import { describe, expect, test } from "bun:test";
import { filterRows, formatNumber, sortRows, type FilterRule, type Row } from "./tableRows";

const rows: Row[] = [
  { name: "Bravo", rpm: 900 },
  { name: "alpha", rpm: 1200 },
  { name: "Charlie", rpm: null },
  { name: "delta", rpm: "N/A" },
  { name: "Echo", rpm: 80 },
];

const rule = (operator: FilterRule["operator"], value: string, column = "name"): FilterRule =>
  ({ id: "r", column, operator, value });

describe("filterRows", () => {
  test("no usable rule returns the same array", () => {
    expect(filterRows(rows, [])).toBe(rows);
    expect(filterRows(rows, [rule("contains", "   ")])).toBe(rows);
    expect(filterRows(rows, [{ id: "r", column: "", operator: "equals", value: "x" }])).toBe(rows);
  });
  test("text operators ignore case and surrounding spaces", () => {
    expect(filterRows(rows, [rule("contains", " HA ")]).map((r) => r.name)).toEqual(["alpha", "Charlie"]);
    expect(filterRows(rows, [rule("equals", "ECHO")]).map((r) => r.name)).toEqual(["Echo"]);
    expect(filterRows(rows, [rule("starts_with", "c")]).map((r) => r.name)).toEqual(["Charlie"]);
  });
  test("numeric operators compare as numbers", () => {
    expect(filterRows(rows, [rule("gt", "500", "rpm")]).map((r) => r.name)).toEqual(["Bravo", "alpha"]);
    expect(filterRows(rows, [rule("lt", "100", "rpm")]).map((r) => r.name)).toEqual(["Charlie", "Echo"]);
  });
  test("all rules must match", () => {
    expect(filterRows(rows, [rule("contains", "a"), rule("gt", "1000", "rpm")]).map((r) => r.name)).toEqual(["alpha"]);
  });
});

describe("sortRows", () => {
  test("text sort is case-insensitive in both directions", () => {
    expect(sortRows(rows, "name", "asc", false).map((r) => r.name)).toEqual(["alpha", "Bravo", "Charlie", "delta", "Echo"]);
    expect(sortRows(rows, "name", "desc", false).map((r) => r.name)).toEqual(["Echo", "delta", "Charlie", "Bravo", "alpha"]);
  });
  test("numeric sort compares numbers and empties always last", () => {
    expect(sortRows(rows, "rpm", "asc", true).map((r) => r.name)).toEqual(["Echo", "Bravo", "alpha", "Charlie", "delta"]);
    expect(sortRows(rows, "rpm", "desc", true).map((r) => r.name)).toEqual(["alpha", "Bravo", "Echo", "Charlie", "delta"]);
  });
  test("returns a copy and leaves the input order alone", () => {
    const input = [...rows];
    const out = sortRows(input, "name", "asc", false);
    expect(out).not.toBe(input);
    expect(input).toEqual(rows);
  });
});

describe("formatNumber", () => {
  test("hex and bin for numeric values", () => {
    expect(formatNumber(255, "hex")).toBe("0xFF");
    expect(formatNumber("5", "bin")).toBe("0b101");
  });
  test("other values pass through unchanged", () => {
    expect(formatNumber(255)).toBe(255);
    expect(formatNumber("N/A", "hex")).toBe("N/A");
    expect(formatNumber("abc", "hex")).toBe("abc");
    expect(formatNumber(null, "hex")).toBe(null);
  });
});
