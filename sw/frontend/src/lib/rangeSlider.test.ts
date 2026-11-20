import { describe, expect, test } from "bun:test";
import { keyTarget, moveThumb, nearestThumb, snap, tickAt } from "./rangeSlider";

const volts = { min: 9, max: 16, step: 0.1, gap: 0.1 };
const pct = { min: 0, max: 100, step: 1, gap: 1 };

describe("rangeSlider", () => {
  test("snap rounds to the step and stays in range", () => {
    expect(snap(13.24, volts)).toBe(13.2);
    expect(snap(20, volts)).toBe(16);
    expect(snap(-4, pct)).toBe(0);
  });

  test("two thumbs never cross and keep the gap", () => {
    expect(moveThumb([11.4, 13.2], 0, 14, volts)).toEqual([13.1, 13.2]);
    expect(moveThumb([11.4, 13.2], 1, 10, volts)).toEqual([11.4, 11.5]);
    expect(moveThumb([11.4, 13.2], 1, 20, volts)).toEqual([11.4, 16]);
  });

  test("a zero gap lets the thumbs meet but not cross", () => {
    const meet = { ...volts, gap: 0 };
    expect(moveThumb([11.4, 13.2], 0, 14, meet)).toEqual([13.2, 13.2]);
    expect(moveThumb([11.4, 13.2], 1, 10, meet)).toEqual([11.4, 11.4]);
  });

  test("one thumb moves freely", () => {
    expect(moveThumb([50], 0, 73.4, pct)).toEqual([73]);
  });

  test("a press takes the nearer thumb", () => {
    expect(nearestThumb([11.4, 13.2], 10)).toBe(0);
    expect(nearestThumb([11.4, 13.2], 15)).toBe(1);
    expect(nearestThumb([11.4, 13.2], 12.3)).toBe(1);
    expect(nearestThumb([50], 10)).toBe(0);
  });

  test("keys", () => {
    expect(keyTarget(13.2, "ArrowRight", volts, 0.1, 1)).toBeCloseTo(13.3);
    expect(keyTarget(50, "PageDown", pct, 5, 20)).toBe(30);
    expect(keyTarget(50, "End", pct, 5, 20)).toBe(100);
    expect(keyTarget(50, "a", pct, 5, 20)).toBeUndefined();
  });

  test("without a step a value only clamps", () => {
    expect(moveThumb([5], 0, 7.77, { min: 0, max: 10 })).toEqual([7.77]);
    expect(moveThumb([5], 0, 70, { min: 0, max: 10 })).toEqual([10]);
  });

  test("a press near a tick lands on it", () => {
    const lin = (v: number) => (v - 9) / 7;
    const ticks = [10, 11, 12, 13];
    expect(tickAt(lin(12) + 0.01, ticks, lin, 300)).toBe(12);
    expect(tickAt(lin(12) + 0.06, ticks, lin, 300)).toBeNull();
    expect(tickAt(0.5, ticks, lin, 0)).toBeNull();
  });
});
