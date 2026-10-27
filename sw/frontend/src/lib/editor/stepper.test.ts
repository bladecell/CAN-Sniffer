import { describe, expect, test } from "bun:test";
import { stepValue } from "./stepper";

describe("stepValue", () => {
  test("whole steps", () => {
    expect(stepValue(5, 1, { step: 1 })).toBe(6);
    expect(stepValue(5, -1, { step: 1 })).toBe(4);
  });
  test("decimal steps do not drift (0.1 + 0.1 + 0.1 is 0.3, not 0.30000000000000004)", () => {
    expect(stepValue(0, 1, { step: 0.1 })).toBe(0.1);
    expect(stepValue(0.2, 1, { step: 0.1 })).toBe(0.3);
    expect(stepValue(0.5, -1, { step: 0.25 })).toBe(0.25);
  });
  test("clamps to min and max", () => {
    expect(stepValue(0, -1, { step: 1, min: 0 })).toBe(0);
    expect(stepValue(119, 1, { step: 1, max: 120 })).toBe(120);
    expect(stepValue(120, 1, { step: 1, max: 120 })).toBe(120);
  });
  test("multiplier is the page step (PageUp / PageDown)", () => {
    expect(stepValue(10, 1, { step: 1, multiplier: 10 })).toBe(20);
    expect(stepValue(10, -1, { step: 1, multiplier: 10, min: 0 })).toBe(0);
  });
  test("an empty value starts from 0 and then clamps", () => {
    expect(stepValue(undefined, 1, { step: 1, min: 5 })).toBe(5);
    expect(stepValue(undefined, 1, { step: 2 })).toBe(2);
  });
});
