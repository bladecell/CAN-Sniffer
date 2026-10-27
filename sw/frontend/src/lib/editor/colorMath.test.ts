import { describe, expect, test } from "bun:test";
import { PALETTE, hexToHsl, hslToHex, normalizeHex } from "./colorMath";

describe("normalizeHex", () => {
  test("accepts 6 and 3 digit hex with or without #, returns uppercase #RRGGBB", () => {
    expect(normalizeHex("#3498db")).toBe("#3498DB");
    expect(normalizeHex("3498DB")).toBe("#3498DB");
    expect(normalizeHex("#abc")).toBe("#AABBCC");
  });
  test("rejects everything else", () => {
    for (const bad of ["", "#12345", "#1234567", "zz0000", "#3498DG", "red"]) {
      expect(normalizeHex(bad)).toBeNull();
    }
  });
});

describe("hexToHsl", () => {
  test("primaries and a known blue (hand-computed: h 204, s 70, l 53)", () => {
    expect(hexToHsl("#FF0000")).toEqual({ h: 0, s: 100, l: 50 });
    const blue = hexToHsl("#3498DB");
    expect(blue.h).toBeCloseTo(204, 0);
    expect(blue.s).toBeCloseTo(70, 0);
    expect(blue.l).toBeCloseTo(53, 0);
  });
});

describe("hslToHex", () => {
  test("known values", () => {
    expect(hslToHex({ h: 120, s: 100, l: 50 })).toBe("#00FF00");
    expect(hslToHex({ h: 0, s: 0, l: 0 })).toBe("#000000");
    expect(hslToHex({ h: 0, s: 0, l: 100 })).toBe("#FFFFFF");
  });
  test("hue 360 is hue 0", () => {
    expect(hslToHex({ h: 360, s: 100, l: 50 })).toBe("#FF0000");
  });
  test("round trip keeps every palette colour", () => {
    for (const hex of PALETTE) expect(hslToHex(hexToHsl(hex))).toBe(hex);
  });
});

describe("PALETTE", () => {
  test("has 12 distinct uppercase swatches", () => {
    expect(PALETTE).toHaveLength(12);
    expect(new Set(PALETTE).size).toBe(12);
    for (const hex of PALETTE) expect(hex).toMatch(/^#[0-9A-F]{6}$/);
  });
});
