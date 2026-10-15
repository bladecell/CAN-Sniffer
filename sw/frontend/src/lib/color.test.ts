import { describe, expect, test } from "bun:test";
import { safeHexColor } from "./color";

describe("safeHexColor", () => {
  test("accepts #rrggbb and 24-bit integers", () => {
    expect(safeHexColor("#3b82F6")).toBe("#3b82F6");
    expect(safeHexColor(0xf59f0b)).toBe("#f59f0b");
    expect(safeHexColor(5)).toBe("#000005");
  });
  test("anything else falls back", () => {
    for (const bad of ["red", "#fff", "#12345g", "red; background:url(x)", "#3b82f6;x", "var(--x)", -1, 0x1000000, 1.5, NaN, null, undefined, {}]) {
      expect(safeHexColor(bad, "#111111")).toBe("#111111");
    }
  });
});
