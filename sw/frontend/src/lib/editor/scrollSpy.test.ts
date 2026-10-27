import { describe, expect, test } from "bun:test";
import { activeSectionIndex } from "./scrollSpy";

describe("activeSectionIndex", () => {
  const tops = [0, 300, 700];
  test("the last section whose top has passed the probe line is active", () => {
    expect(activeSectionIndex(tops, 0, 100)).toBe(0);
    expect(activeSectionIndex(tops, 250, 100)).toBe(1);
    expect(activeSectionIndex(tops, 600, 100)).toBe(2);
  });
  test("a section is active exactly at its probe line", () => {
    expect(activeSectionIndex(tops, 200, 100)).toBe(1);
  });
  test("at the end of the page the last section wins even if its top never reaches the probe", () => {
    expect(activeSectionIndex(tops, 400, 100, true)).toBe(2);
  });
  test("no sections gives 0", () => {
    expect(activeSectionIndex([], 50, 100)).toBe(0);
  });
});
