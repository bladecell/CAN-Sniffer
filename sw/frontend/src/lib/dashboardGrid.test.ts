import { describe, expect, test } from "bun:test";
import { clampSize } from "./dashboardModel";
import { columnPitch, keyResize, resizeTo, stepIndex } from "./dashboardGrid";

// A bar card: 2x1 minimum, 6x2 maximum (docs/decisions.md, 2026-11-02).
const clampBar = (w: number, h: number) => clampSize({ type: "bar", style: "plain", pids: [1] }, w, h);

describe("columnPitch", () => {
  test("six columns with 12 px gaps", () => {
    // 6 columns and 5 gaps fill 672 px: (672 - 60) / 6 + 12 = 114.
    expect(columnPitch(672)).toBe(114);
  });

  test("never zero or negative on a collapsed grid", () => {
    expect(columnPitch(0)).toBeGreaterThan(0);
  });
});

describe("resizeTo", () => {
  const pitch = 114;

  test("snaps the width to whole columns", () => {
    // 60 px is just over half a pitch: one extra column.
    expect(resizeTo(clampBar, { w: 3, h: 1 }, 60, 0, pitch)).toEqual({ w: 4, h: 1 });
    expect(resizeTo(clampBar, { w: 3, h: 1 }, 40, 0, pitch)).toEqual({ w: 3, h: 1 });
  });

  test("snaps the height to whole rows of 56 px plus the 12 px gap", () => {
    // One row pitch is 68 px. 30 px is under half a pitch, so no row changes.
    expect(resizeTo(clampBar, { w: 3, h: 1 }, 0, 30, pitch)).toEqual({ w: 3, h: 1 });
    expect(resizeTo(clampBar, { w: 3, h: 1 }, 0, 50, pitch)).toEqual({ w: 3, h: 2 });
  });

  test("stays inside the card type's range", () => {
    expect(resizeTo(clampBar, { w: 6, h: 2 }, 900, 900, pitch)).toEqual({ w: 6, h: 2 });
    expect(resizeTo(clampBar, { w: 2, h: 1 }, -900, -900, pitch)).toEqual({ w: 2, h: 1 });
  });
});

describe("keyResize", () => {
  test("arrows change one column or row", () => {
    expect(keyResize(clampBar, { w: 3, h: 1 }, "ArrowRight")).toEqual({ w: 4, h: 1 });
    expect(keyResize(clampBar, { w: 3, h: 1 }, "ArrowDown")).toEqual({ w: 3, h: 2 });
    expect(keyResize(clampBar, { w: 3, h: 2 }, "ArrowUp")).toEqual({ w: 3, h: 1 });
  });

  test("clamps at the range and ignores other keys", () => {
    expect(keyResize(clampBar, { w: 6, h: 2 }, "ArrowRight")).toEqual({ w: 6, h: 2 });
    expect(keyResize(clampBar, { w: 3, h: 1 }, "a")).toBeNull();
  });
});

describe("stepIndex", () => {
  test("moves within a section and stops at both ends", () => {
    expect(stepIndex(1, 3, 1)).toBe(2);
    expect(stepIndex(1, 3, -1)).toBe(0);
    expect(stepIndex(0, 3, -1)).toBeNull();
    expect(stepIndex(2, 3, 1)).toBeNull();
  });
});
