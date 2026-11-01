import { describe, expect, test } from "bun:test";
import { DEFAULT_GUTTERS, GUTTER_LEFT, GUTTER_RIGHT, MEASURED_TICK_PX } from "./gutter";

describe("shared gutters", () => {
  test("left is the widest y tick label plus the gap", () => {
    expect(GUTTER_LEFT).toBe(36);
    expect(GUTTER_LEFT).toBeGreaterThanOrEqual(MEASURED_TICK_PX + 4);
  });

  test("right holds only the line end, the now label and the cursor dot (no end labels)", () => {
    expect(GUTTER_RIGHT).toBe(8);
    expect(GUTTER_RIGHT).toBeGreaterThanOrEqual(5 + 2);
  });

  test("the defaults are the constants used everywhere", () => {
    expect(DEFAULT_GUTTERS).toEqual({ left: GUTTER_LEFT, right: GUTTER_RIGHT });
  });
});
