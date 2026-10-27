import { describe, expect, test } from "bun:test";
import { chartPidsFor, chipAction, groupPids, pendingKind, type PidGroupInput } from "./telemetryView";

const base: PidGroupInput = { loaded: true, supported: true, hasValue: true, mode: 0x01, modeLabel: "Current Data" };
const item = (id: string, over: Partial<PidGroupInput> = {}) => ({ id, input: { ...base, ...over } });
const titles = (items: ReturnType<typeof item>[]) =>
  groupPids(items, (i) => i.input).map((g) => `${g.title}:${g.items.map((i) => i.id).join(",")}`);

describe("groupPids", () => {
  test("orders not supported, supported without data, then available by mode", () => {
    const out = titles([
      item("avail-rpm"),
      item("nodata", { hasValue: false }),
      item("unsupported", { supported: false, hasValue: false }),
    ]);
    expect(out).toEqual([
      "Not supported:unsupported",
      "Supported, no data:nodata",
      "Available · Current Data:avail-rpm",
    ]);
  });

  test("available PIDs are sub-grouped by mode in ascending mode order", () => {
    const out = titles([
      item("derived", { mode: 0x45, modeLabel: "Derived Data" }),
      item("rbi", { mode: 0x22, modeLabel: "Read Data By Identifier" }),
      item("cur", { mode: 0x01, modeLabel: "Current Data" }),
      item("cur2"),
    ]);
    expect(out).toEqual([
      "Available · Current Data:cur,cur2",
      "Available · Read Data By Identifier:rbi",
      "Available · Derived Data:derived",
    ]);
  });

  test("rows not on the device go last, whatever their support state", () => {
    const out = titles([item("off", { loaded: false, supported: false, hasValue: false }), item("avail")]);
    expect(out).toEqual(["Available · Current Data:avail", "Not on device:off"]);
  });

  test("empty groups are omitted and keys are unique", () => {
    const groups = groupPids([item("a"), item("b", { mode: 0x22, modeLabel: "RBI" })], (i) => i.input);
    expect(groups.map((g) => g.key)).toEqual(["available:1", "available:34"]);
    expect(groupPids([] as ReturnType<typeof item>[], (i) => i.input)).toEqual([]);
  });

  test("keeps the original item objects and their order within a group", () => {
    const a = item("a");
    const groups = groupPids([a, item("b")], (i) => i.input);
    expect(groups[0].items[0]).toBe(a);
    expect(groups[0].items.map((i) => i.id)).toEqual(["a", "b"]);
  });
});

describe("pendingKind", () => {
  test("loaded and not on the device is added", () => {
    expect(pendingKind(true, false)).toBe("added");
  });

  test("loaded and on the device is modified", () => {
    expect(pendingKind(true, true)).toBe("modified");
  });

  test("not loaded is removed", () => {
    expect(pendingKind(false, true)).toBe("removed");
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
