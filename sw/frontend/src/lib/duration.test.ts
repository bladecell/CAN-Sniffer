import { describe, expect, test } from "bun:test";
import { parseDurationMs } from "./duration";

describe("parseDurationMs", () => {
  test("units", () => {
    expect(parseDurationMs("250")).toBe(250);
    expect(parseDurationMs("250 ms")).toBe(250);
    expect(parseDurationMs("1.5 s")).toBe(1500);
    expect(parseDurationMs("1,5s")).toBe(1500);
    expect(parseDurationMs("2 min")).toBe(120000);
    expect(parseDurationMs("1 H")).toBe(3600000);
  });
  test("rejects anything else", () => {
    for (const t of ["", "abc", "-5", "1 day", "1..5", "5 s s"]) expect(parseDurationMs(t)).toBeNull();
  });
});
