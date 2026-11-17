import { describe, expect, test } from "bun:test";
import { captureRecords } from "./captureRetention";

describe("logging", () => {
  test("records only when logging is on and the session is active", () => {
    expect(captureRecords(false, true)).toBe(false);
    expect(captureRecords(true, true)).toBe(true);
    expect(captureRecords(true, false)).toBe(false);
    expect(captureRecords(false, false)).toBe(false);
  });
});
