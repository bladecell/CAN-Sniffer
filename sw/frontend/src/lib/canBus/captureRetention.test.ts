import { describe, expect, test } from "bun:test";
import { captureActive, readKeep, storeKeep } from "./captureRetention";

describe("log in background", () => {
  test("off by default and only an explicit 1 turns it on", () => {
    expect(readKeep(null)).toBe(false);
    expect(readKeep("0")).toBe(false);
    expect(readKeep("true")).toBe(false);
    expect(readKeep("1")).toBe(true);
    expect(storeKeep(true)).toBe("1");
    expect(storeKeep(false)).toBe("0");
  });

  test("the stream runs with the page open, or with the toggle on; never with neither", () => {
    expect(captureActive(false, false)).toBe(false);
    expect(captureActive(false, true)).toBe(true);
    expect(captureActive(true, false)).toBe(true);
    expect(captureActive(true, true)).toBe(true);
  });
});
