import { describe, expect, test } from "bun:test";
import {
  applyPidView,
  chartPidsFor,
  chipAction,
  pidStatusChips,
  seenLabel,
  DEFAULT_PID_FIELDS,
} from "./telemetryView";

describe("pidStatusChips", () => {
  const all = new Set(["mode", "interval", "supported", "valid", "pending", "loaded"] as const);

  test("supported and valid PID shows ok chips", () => {
    const chips = pidStatusChips({ loaded: true, supported: true, valid: true, pending: false }, all);
    expect(chips.map((c) => c.label)).toEqual(["Supported", "Valid"]);
    expect(chips.every((c) => c.kind === "ok")).toBe(true);
  });

  test("unsupported and invalid PID shows warn and crit chips", () => {
    const chips = pidStatusChips({ loaded: true, supported: false, valid: false, pending: false }, all);
    expect(chips).toEqual([
      { kind: "warn", label: "Not supported", icon: "warn" },
      { kind: "crit", label: "Invalid", icon: "warn" },
    ]);
  });

  test("pending sync only when the field is visible and the PID is pending", () => {
    const pending = { loaded: true, supported: true, valid: true, pending: true };
    expect(pidStatusChips(pending, all).map((c) => c.label)).toContain("Pending sync");
    expect(pidStatusChips(pending, new Set(["supported"])).map((c) => c.label)).not.toContain("Pending sync");
  });

  test("hidden fields produce no supported or valid chips", () => {
    const chips = pidStatusChips({ loaded: true, supported: false, valid: false, pending: false }, new Set());
    expect(chips).toEqual([]);
  });

  test("not loaded is always shown", () => {
    const chips = pidStatusChips({ loaded: false, supported: true, valid: true, pending: false }, new Set());
    expect(chips).toEqual([{ kind: "plain", label: "Not loaded" }]);
  });

  test("default fields match the design defaults", () => {
    expect([...DEFAULT_PID_FIELDS].sort()).toEqual(["interval", "mode", "pending", "supported", "valid"]);
  });
});

describe("seenLabel", () => {
  test("never seen when there is no receipt time", () => {
    expect(seenLabel(null)).toBe("never seen");
  });

  test("seconds with one decimal under a minute", () => {
    expect(seenLabel(2400)).toBe("seen 2.4 s ago");
    expect(seenLabel(0)).toBe("seen 0.0 s ago");
  });

  test("clamps negative ages from clock jumps to zero", () => {
    expect(seenLabel(-500)).toBe("seen 0.0 s ago");
  });

  test("whole minutes from one minute", () => {
    expect(seenLabel(125_000)).toBe("seen 2 min ago");
  });
});

describe("applyPidView", () => {
  type Row = { id: string };
  const item = (id: string, cells: Record<string, string | number | null>) => ({ row: { id } as Row, cells });

  const items = [
    item("a", { name: "Speed", pid: 0x0d, value: 40, interval: 200 }),
    item("b", { name: "Engine RPM", pid: 0x0c, value: null, interval: 100 }),
    item("c", { name: "Coolant", pid: 0x05, value: 90, interval: 1000 }),
  ];

  test("returns the same rows in order with no rules and no sort", () => {
    expect(applyPidView(items, [], null).map((r) => r.id)).toEqual(["a", "b", "c"]);
  });

  test("filters by a rule and keeps original row objects", () => {
    const out = applyPidView(items, [{ id: "1", column: "name", operator: "contains", value: "eng" }], null);
    expect(out).toEqual([{ id: "b" }]);
    expect(out[0]).toBe(items[1].row);
  });

  test("sorts numeric columns numerically and puts empty values last", () => {
    expect(applyPidView(items, [], { key: "pid", dir: 1 }).map((r) => r.id)).toEqual(["c", "b", "a"]);
    expect(applyPidView(items, [], { key: "value", dir: -1 }).map((r) => r.id)).toEqual(["c", "a", "b"]);
  });
});

describe("chartPidsFor", () => {
  test("ticked PIDs win over the active row", () => {
    expect(chartPidsFor([0x0c, 0x0d], 0x05)).toEqual([0x0c, 0x0d]);
  });

  test("with nothing ticked, the active row is charted", () => {
    expect(chartPidsFor([], 0x05)).toEqual([0x05]);
  });

  test("nothing ticked and no active row charts nothing", () => {
    expect(chartPidsFor([], null)).toEqual([]);
  });
});

describe("chipAction", () => {
  test("an unpressed chip ticks the PID", () => {
    expect(chipAction(false, false)).toBe("tick");
  });

  test("a pressed, ticked PID unticks", () => {
    expect(chipAction(true, true)).toBe("untick");
  });

  test("a pressed PID that is only the active row clears the active row", () => {
    expect(chipAction(true, false)).toBe("clearActive");
  });
});
