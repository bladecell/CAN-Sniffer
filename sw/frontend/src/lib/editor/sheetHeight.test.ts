import { describe, expect, test } from "bun:test";
import { clampSheetHeight, defaultSheetHeight, stepSheetHeight } from "./sheetHeight";

describe("sheet height", () => {
  test("default is 65% of the viewport", () => {
    expect(defaultSheetHeight(1000)).toBe(650);
    expect(defaultSheetHeight(800)).toBe(520);
  });
  test("clamps between 30% and 85% of the viewport", () => {
    expect(clampSheetHeight(100, 1000)).toBe(300);
    expect(clampSheetHeight(990, 1000)).toBe(850);
    expect(clampSheetHeight(600, 1000)).toBe(600);
  });
  test("keyboard steps move 5% of the viewport and stay clamped", () => {
    expect(stepSheetHeight(600, "up", 1000)).toBe(650);
    expect(stepSheetHeight(600, "down", 1000)).toBe(550);
    expect(stepSheetHeight(850, "up", 1000)).toBe(850);
    expect(stepSheetHeight(300, "down", 1000)).toBe(300);
  });
});
