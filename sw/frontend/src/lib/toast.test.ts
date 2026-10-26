import { describe, expect, test } from "bun:test";
import { escapeTarget, evictOldest, remainingMs } from "./toast";

describe("evictOldest", () => {
  test("keeps room for one incoming toast when the stack is full", () => {
    expect(evictOldest([1, 2, 3], 3)).toEqual([1]);
  });

  test("evicts nothing while there is room", () => {
    expect(evictOldest([1, 2], 3)).toEqual([]);
    expect(evictOldest([], 3)).toEqual([]);
  });
});

describe("escapeTarget", () => {
  test("picks the newest toast", () => {
    expect(escapeTarget([4, 7, 9], false)).toBe(9);
  });

  test("yields to an open dialog or popover", () => {
    expect(escapeTarget([4, 7], true)).toBeNull();
  });

  test("has nothing to dismiss when no toast is shown", () => {
    expect(escapeTarget([], false)).toBeNull();
  });
});

describe("remainingMs", () => {
  test("subtracts the time that ran before a pause", () => {
    expect(remainingMs(5000, 100, 1100)).toBe(4000);
  });

  test("never goes below zero", () => {
    expect(remainingMs(500, 0, 9000)).toBe(0);
  });
});
