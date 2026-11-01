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

import { clampSheetPx } from "./sheetHeight";

describe("clampSheetPx", () => {
  test("holds the height inside the pixel bounds", () => {
    expect(clampSheetPx(100, 250, 700)).toBe(250);
    expect(clampSheetPx(900, 250, 700)).toBe(700);
    expect(clampSheetPx(400.4, 250, 700)).toBe(400);
  });

  test("inverted or equal bounds collapse to a single height", () => {
    expect(clampSheetPx(300, 700, 250)).toBe(300);
    expect(clampSheetPx(900, 700, 250)).toBe(700);
    expect(clampSheetPx(300, 400, 400)).toBe(400);
  });
});
