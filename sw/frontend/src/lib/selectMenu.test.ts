import { describe, expect, test } from "bun:test";
import { defaultIndex, navigate, type SelectMenuOption } from "./selectMenu";

const opts: SelectMenuOption<string>[] = [
  { value: "a", label: "A" },
  { value: "b", label: "B", disabled: true },
  { value: "c", label: "C" },
  { value: "d", label: "D" },
];

describe("defaultIndex", () => {
  test("keeps the selected index when it is enabled", () => {
    expect(defaultIndex(opts, 3)).toBe(3);
  });

  test("falls back to the first enabled option for a disabled or unknown selection", () => {
    expect(defaultIndex(opts, 1)).toBe(0);
    expect(defaultIndex(opts, -1)).toBe(0);
    expect(defaultIndex(opts, 99)).toBe(0);
  });

  test("returns -1 for an empty or fully disabled list", () => {
    expect(defaultIndex([], 0)).toBe(-1);
    expect(defaultIndex([{ value: 1, label: "x", disabled: true }], -1)).toBe(-1);
  });
});

describe("navigate", () => {
  test("ArrowDown and ArrowUp skip disabled options", () => {
    expect(navigate(opts, 0, "ArrowDown")).toBe(2);
    expect(navigate(opts, 2, "ArrowUp")).toBe(0);
  });

  test("ArrowDown and ArrowUp stay put at the ends", () => {
    expect(navigate(opts, 3, "ArrowDown")).toBe(3);
    expect(navigate(opts, 0, "ArrowUp")).toBe(0);
  });

  test("Home and End go to the first and last enabled option", () => {
    expect(navigate(opts, 2, "Home")).toBe(0);
    expect(navigate(opts, 0, "End")).toBe(3);
  });

  test("returns undefined for keys that are not navigation keys", () => {
    expect(navigate(opts, 0, "Enter")).toBeUndefined();
    expect(navigate(opts, 0, "a")).toBeUndefined();
  });

  test("does not crash on an empty list or an out-of-range current index", () => {
    expect(navigate([], -1, "ArrowDown")).toBe(-1);
    expect(navigate([], 0, "End")).toBe(0);
    expect(navigate(opts, 99, "ArrowDown")).toBe(99);
  });
});
