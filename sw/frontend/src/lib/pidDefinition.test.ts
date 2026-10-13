import { describe, expect, test } from "bun:test";
import { normalizePidDefinition, parsePidList } from "./pidDefinition";

const required = { id: 0x7df, mode: 1, pid: 12, length: 2, name: "RPM", formula: "A", update_interval_ms: 256 };

describe("normalizePidDefinition", () => {
  test("full definition passes through", () => {
    const d = normalizePidDefinition({ ...required, unit: "rpm", description: "d", priority: 3, color: 5, icon: "gauge", minValue: 0, maxValue: 8000 });
    expect(d.unit).toBe("rpm");
    expect(d.maxValue).toBe(8000);
  });
  test("null/undefined optional fields are unset, not an error", () => {
    const d = normalizePidDefinition({ ...required, unit: null, desc: null, priority: null, color: null, icon: null, minV: null, maxV: undefined });
    expect(d.unit).toBe("");
    expect(d.description).toBe("");
    expect(d.icon).toBe("");
    expect(d.priority).toBe(5);
    expect(d.color).toBe(0x01aaff);
    expect(Number.isNaN(d.minValue)).toBe(true);
    expect(Number.isNaN(d.maxValue)).toBe(true);
  });
  test("short key aliases are accepted", () => {
    const d = normalizePidDefinition({ ...required, len: 3, length: undefined, desc: "x", interval: 512, update_interval_ms: undefined, minV: 1, maxV: 2 });
    expect([d.length, d.description, d.update_interval_ms, d.minValue, d.maxValue]).toEqual([3, "x", 512, 1, 2]);
  });
  test("missing required field still throws", () => {
    expect(() => normalizePidDefinition({ ...required, name: undefined })).toThrow();
    expect(() => normalizePidDefinition({ ...required, formula: null })).toThrow();
    expect(() => normalizePidDefinition(null)).toThrow();
  });
});

describe("parsePidList", () => {
  test("one sparse entry does not fail the list", () => {
    const list = parsePidList({ data: [{ ...required, minValue: null }, { ...required, pid: 13, unit: null }] });
    expect(list).toHaveLength(2);
  });
  test("rejects a non-array", () => {
    expect(() => parsePidList({ data: "x" })).toThrow();
    expect(() => parsePidList(null)).toThrow();
  });
});
