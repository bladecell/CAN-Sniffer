import { describe, expect, test } from "bun:test";
import {
  chartPidsFor,
  definitionsToApply,
  chipAction,
  footerActions,
  groupPids,
  pendingKind,
  formatPid,
  pidHex,
  removeRows,
  type PidGroupInput,
} from "./telemetryView";

const base: PidGroupInput = { supported: true, hasValue: true, mode: 0x01, modeLabel: "Current Data" };
const item = (id: string, over: Partial<PidGroupInput> = {}) => ({ id, input: { ...base, ...over } });
const titles = (items: ReturnType<typeof item>[]) =>
  groupPids(items, (i) => i.input).map((g) => `${g.title}:${g.items.map((i) => i.id).join(",")}`);

describe("groupPids", () => {
  test("orders available by mode, supported without data, then not supported", () => {
    const out = titles([
      item("unsupported", { supported: false, hasValue: false }),
      item("nodata", { hasValue: false }),
      item("avail-rpm"),
    ]);
    expect(out).toEqual([
      "Available by mode:avail-rpm",
      "Supported, no data:nodata",
      "Not supported:unsupported",
    ]);
  });

  test("available has one section per mode in ascending mode order", () => {
    const groups = groupPids(
      [
        item("derived", { mode: 0x45, modeLabel: "Derived Data" }),
        item("rbi", { mode: 0x22, modeLabel: "Read Data By Identifier" }),
        item("cur", { mode: 0x01, modeLabel: "Current Data" }),
        item("cur2"),
      ],
      (i) => i.input,
    );
    const sections = groups[0].sections ?? [];
    expect(sections.map((s) => `${s.title}:${s.items.map((i) => i.id).join(",")}`)).toEqual([
      "Current Data:cur,cur2",
      "Read Data By Identifier:rbi",
      "Derived Data:derived",
    ]);
  });

  test("only the available group has sections", () => {
    const groups = groupPids([item("a"), item("b", { hasValue: false })], (i) => i.input);
    expect(groups.map((g) => [g.key, g.sections === undefined])).toEqual([
      ["available", false],
      ["nodata", true],
    ]);
  });

  test("empty groups are omitted and keys are unique", () => {
    const groups = groupPids([item("a"), item("b", { mode: 0x22, modeLabel: "RBI" })], (i) => i.input);
    expect(groups.map((g) => g.key)).toEqual(["available"]);
    expect(groups[0].sections?.map((s) => s.key)).toEqual(["mode:1", "mode:34"]);
    expect(groupPids([] as ReturnType<typeof item>[], (i) => i.input)).toEqual([]);
  });

  test("keeps the original item objects and their order within a group", () => {
    const a = item("a");
    const groups = groupPids([a, item("b")], (i) => i.input);
    expect(groups[0].items[0]).toBe(a);
    expect(groups[0].items.map((i) => i.id)).toEqual(["a", "b"]);
  });
});

describe("formatPid", () => {
  test("a PID that fits one byte shows two hex digits", () => {
    expect(formatPid(0x05)).toBe("0x05");
    expect(formatPid(0x0c)).toBe("0x0C");
    expect(formatPid(0xff)).toBe("0xFF");
    expect(formatPid(0x00)).toBe("0x00");
  });

  test("a longer PID shows four uppercase hex digits", () => {
    expect(formatPid(0x0105)).toBe("0x0105");
    expect(formatPid(0xf40c)).toBe("0xF40C");
    expect(formatPid(0x1c0e)).toBe("0x1C0E");
  });
});

describe("pidHex", () => {
  test("shows the PID with the shared formatter", () => {
    expect(pidHex(0x0c, 0x01)).toBe("0x0C");
    expect(pidHex(0x1c0e, 0x22)).toBe("0x1C0E");
    expect(pidHex(0x0105, 0x01)).toBe("0x0105");
  });

  test("derived PIDs show nothing", () => {
    expect(pidHex(0x0c, 0x45)).toBe("");
  });
});

describe("removeRows", () => {
  type Row = { id: string; loaded: boolean; selected: boolean; onDevice: boolean };
  const row = (id: string, over: Partial<Row> = {}): Row => ({ id, loaded: true, selected: false, onDevice: true, ...over });
  const remove = (rows: Row[], pick: (r: Row) => boolean) => removeRows(rows, pick, (r) => r.onDevice);

  test("a selected PID on the device is kept but marked not loaded, so Changes lists it as removed", () => {
    const out = remove([row("a", { selected: true })], (r) => r.selected);
    expect(out).toHaveLength(1);
    expect(out[0]).toMatchObject({ loaded: false, selected: false });
  });

  test("a selected PID the device never had is dropped", () => {
    const out = remove([row("new", { selected: true, onDevice: false })], (r) => r.selected);
    expect(out).toEqual([]);
  });

  test("unselected rows are left as they are", () => {
    const other = row("b");
    const out = remove([row("a", { selected: true }), other], (r) => r.selected);
    expect(out[1]).toBe(other);
    expect(out[1]).toMatchObject({ loaded: true, selected: false });
  });

  test("a swipe removes only the row it is given", () => {
    const rows = [row("a"), row("b")];
    const out = remove(rows, (r) => r.id === "b");
    expect(out.map((r) => [r.id, r.loaded])).toEqual([
      ["a", true],
      ["b", false],
    ]);
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

describe("footerActions", () => {
  test("nothing selected offers New PID only", () => {
    expect(footerActions(0)).toEqual(["new"]);
  });

  test("one selected PID offers Edit and Remove", () => {
    expect(footerActions(1)).toEqual(["edit", "remove"]);
  });

  test("several selected PIDs offer Remove N only", () => {
    expect(footerActions(3)).toEqual(["remove"]);
  });
});

describe("definitionsToApply", () => {
  type Def = { name: string };
  const device = (name: string) => ({ name });
  const item = (pid: number, over: { loaded?: boolean; pending?: boolean; def?: Def; deviceDef?: Def } = {}) => ({
    pid,
    loaded: over.loaded ?? true,
    pending: over.pending ?? false,
    def: over.def ?? device(`row${pid}`),
    deviceDef: over.deviceDef,
  });
  const names = (defs: Def[]) => defs.map((d) => d.name);

  test("writes the drafts of the chosen changes and the device copy of the rest", () => {
    const items = [
      item(1, { pending: true, def: device("edited"), deviceDef: device("old") }),
      item(2, { pending: true, def: device("edited2"), deviceDef: device("old2") }),
      item(3),
    ];
    expect(names(definitionsToApply(items, new Set([1])))).toEqual(["edited", "old2", "row3"]);
  });

  test("an added PID is written when chosen and dropped when not", () => {
    const added = item(4, { pending: true, def: device("new") });
    expect(names(definitionsToApply([added], new Set([4])))).toEqual(["new"]);
    expect(names(definitionsToApply([added], new Set()))).toEqual([]);
  });

  test("a removed PID is dropped when chosen and kept as on the device when not", () => {
    const removed = item(5, { loaded: false, pending: true, deviceDef: device("kept") });
    expect(names(definitionsToApply([removed], new Set([5])))).toEqual([]);
    expect(names(definitionsToApply([removed], new Set()))).toEqual(["kept"]);
  });
});
