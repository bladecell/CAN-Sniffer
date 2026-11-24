import { describe, expect, test } from "bun:test";
import { buildWideCsv, parseSidecarNames, wideFileName } from "./tripWide";

describe("buildWideCsv", () => {
  const series = [
    { pid: 12, t: [1000, 2000, 3000], v: [800, null, 950.5] },
    { pid: 13, t: [1000, 3000, 4000], v: [40, 41, 42] },
  ];

  test("one row per distinct t_ms, one column per PID, empty where there is no valid sample", () => {
    expect(buildWideCsv(series)).toBe("t_ms,12,13\n1000,800,40\n2000,,\n3000,950.5,41\n4000,,42\n");
  });

  test("headers use the sidecar name and unit, escaped and neutralised like the plot CSV", () => {
    const names = new Map([[12, { name: "Engine, RPM", unit: "rpm" }], [13, { name: "=cmd", unit: "" }]]);
    expect(buildWideCsv(series, names).split("\n")[0]).toBe('t_ms,"Engine, RPM (rpm)",\'=cmd');
  });

  test("a repeated t_ms in one PID keeps the later value; values use the recorder's %.8g form", () => {
    const s = [{ pid: 1, t: [5, 5, 6], v: [1, 2, 123456789] }];
    expect(buildWideCsv(s)).toBe("t_ms,1\n5,2\n6,1.2345679e+08\n");
  });

  test("no series gives just the header", () => {
    expect(buildWideCsv([])).toBe("t_ms\n");
  });
});

describe("parseSidecarNames", () => {
  test("reads pid, name and unit and ignores anything malformed", () => {
    const text = JSON.stringify({ pids: [{ pid: 12, name: "RPM", unit: "rpm" }, { pid: "x", name: "bad" }, { pid: 13, name: 5 }, null, { pid: 14, name: "Temp" }] });
    expect([...parseSidecarNames(text)]).toEqual([[12, { name: "RPM", unit: "rpm" }], [14, { name: "Temp", unit: "" }]]);
  });

  test("bad JSON, a wrong shape and long names are safe", () => {
    expect(parseSidecarNames("{nope").size).toBe(0);
    expect(parseSidecarNames('{"pids":5}').size).toBe(0);
    expect(parseSidecarNames(JSON.stringify({ pids: [{ pid: 1, name: "n".repeat(500) }] })).get(1)?.name.length).toBe(64);
  });
});

test("wideFileName", () => {
  expect(wideFileName("trip_000007.csv")).toBe("trip_000007_wide.csv");
});
