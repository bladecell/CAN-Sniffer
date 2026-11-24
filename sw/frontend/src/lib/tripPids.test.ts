import { describe, expect, test } from "bun:test";
import { parseSidecarPids, tripDefs, SIDECAR_MAX_BYTES } from "./tripPids";
import type { PidDefinition } from "./types";

const live = (pid: number, name: string, unit = "", color = 0x112233): PidDefinition => ({
  id: 0x7df, mode: 1, pid, length: 1, name, unit, description: "", formula: "", minValue: 0, maxValue: 100, update_interval_ms: 200, color, priority: 1, icon: "",
});

describe("parseSidecarPids", () => {
  test("reads name and unit per pid", () => {
    const m = parseSidecarPids(JSON.stringify({ version: 1, pids: [{ pid: 12, name: "RPM", unit: "rpm" }, { pid: 13, name: "Speed", unit: "km/h" }] }));
    expect(m?.get(12)).toEqual({ name: "RPM", unit: "rpm" });
    expect(m?.get(13)).toEqual({ name: "Speed", unit: "km/h" });
  });
  test("bad json, wrong shape and oversize give null", () => {
    expect(parseSidecarPids("{")).toBeNull();
    expect(parseSidecarPids("[]")).toBeNull();
    expect(parseSidecarPids('{"pids":5}')).toBeNull();
    expect(parseSidecarPids("x".repeat(SIDECAR_MAX_BYTES + 1))).toBeNull();
  });
  test("skips invalid entries, trims and caps strings", () => {
    const m = parseSidecarPids(
      JSON.stringify({ pids: [null, 5, { pid: "x", name: "a" }, { pid: -1, name: "a" }, { pid: 1.5, name: "a" }, { pid: 7, name: 3 }, { pid: 8, name: "n".repeat(500), unit: 4 }] }),
    );
    expect(m?.size).toBe(1);
    expect(m?.get(8)?.name.length).toBe(64);
    expect(m?.get(8)?.unit).toBe("");
  });
  test("caps the number of pids and keeps the first of a duplicate", () => {
    const pids = Array.from({ length: 1000 }, (_, i) => ({ pid: i, name: `p${i}` }));
    expect(parseSidecarPids(JSON.stringify({ pids }))?.size).toBe(256);
    const d = parseSidecarPids(JSON.stringify({ pids: [{ pid: 1, name: "a" }, { pid: 1, name: "b" }] }));
    expect(d?.get(1)?.name).toBe("a");
  });
});

describe("tripDefs", () => {
  test("sidecar names win, colour and range come from the live definition", () => {
    const side = new Map([[12, { name: "Engine RPM", unit: "rpm" }]]);
    const [d] = tripDefs([12], side, [live(12, "Old name", "x", 0xabcdef)]);
    expect(d).toMatchObject({ pid: 12, name: "Engine RPM", unit: "rpm", color: 0xabcdef, minValue: 0, maxValue: 100 });
  });
  test("a pid the sidecar lacks falls back to the live definition", () => {
    const side = new Map([[12, { name: "RPM", unit: "rpm" }]]);
    const defs = tripDefs([12, 13], side, [live(13, "Speed", "km/h")]);
    expect(defs.map((d) => d.name)).toEqual(["RPM", "Speed"]);
    expect(defs[1].unit).toBe("km/h");
  });
  test("no sidecar and no live definition: hex name, a colour, every pid listed", () => {
    const defs = tripDefs([5, 300], null, []);
    expect(defs.map((d) => d.name)).toEqual(["0x05", "0x12C"]);
    expect(defs[0].color).not.toBe(defs[1].color);
    expect(Number.isFinite(defs[0].color)).toBe(true);
  });
  test("an empty sidecar name falls back", () => {
    const [d] = tripDefs([12], new Map([[12, { name: "", unit: "" }]]), [live(12, "RPM", "rpm")]);
    expect(d.name).toBe("RPM");
  });
});
