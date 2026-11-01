import { describe, expect, test } from "bun:test";
import { normalizePidDefinition } from "./pidDefinition";
import { chosenDefinitions, classifyPidImport, pidImportCounts } from "./pidImport";

const def = (over: Record<string, unknown> = {}) =>
  normalizePidDefinition({
    id: 0x7e0,
    mode: 1,
    pid: 0x0c,
    length: 2,
    name: "Engine speed",
    unit: "rpm",
    desc: "",
    formula: "getPID(A,B)/4",
    minV: null,
    maxV: null,
    priority: 1,
    interval: 200,
    color: 0x01aaff,
    icon: "",
    ...over,
  });

describe("classifyPidImport", () => {
  test("a PID not in the current set is new", () => {
    const items = classifyPidImport([def({ pid: 0x0d, name: "Vehicle speed" })], []);
    expect(items.map((item) => item.status)).toEqual(["new"]);
  });

  test("a PID in the set with the same properties is identical", () => {
    const items = classifyPidImport([def()], [{ def: def(), loaded: true }]);
    expect(items.map((item) => item.status)).toEqual(["identical"]);
  });

  test("unset min and max compare equal to each other", () => {
    const items = classifyPidImport([def({ minV: null, maxV: null })], [{ def: def({ minV: null, maxV: null }), loaded: true }]);
    expect(items[0].status).toBe("identical");
  });

  test("a PID in the set with any different property is changed", () => {
    const current = [{ def: def(), loaded: true }];
    expect(classifyPidImport([def({ formula: "getPID(A,B)" })], current)[0].status).toBe("changed");
    expect(classifyPidImport([def({ interval: 1000 })], current)[0].status).toBe("changed");
    expect(classifyPidImport([def({ minV: 0 })], current)[0].status).toBe("changed");
    expect(classifyPidImport([def({ color: 0xff0000 })], current)[0].status).toBe("changed");
  });

  test("a row the device no longer has (not loaded) is changed, not identical", () => {
    const items = classifyPidImport([def()], [{ def: def(), loaded: false }]);
    expect(items[0].status).toBe("changed");
  });

  test("keeps the file order and matches by PID number", () => {
    const current = [{ def: def({ pid: 0x0d }), loaded: true }];
    const items = classifyPidImport([def({ pid: 0x0c }), def({ pid: 0x0d })], current);
    expect(items.map((item) => [item.def.pid, item.status])).toEqual([
      [0x0c, "new"],
      [0x0d, "identical"],
    ]);
  });
});

describe("pidImportCounts", () => {
  test("counts new and changed, not identical", () => {
    const items = classifyPidImport(
      [def({ pid: 1 }), def({ pid: 2 }), def({ pid: 3, formula: "A" }), def({ pid: 4 })],
      [
        { def: def({ pid: 2 }), loaded: true },
        { def: def({ pid: 3 }), loaded: true },
        { def: def({ pid: 4, formula: "B" }), loaded: true },
      ],
    );
    expect(pidImportCounts(items)).toEqual({ added: 1, changed: 2 });
  });
});

describe("chosenDefinitions", () => {
  const items = classifyPidImport([def({ pid: 1 }), def({ pid: 2 }), def({ pid: 3 })], []);

  test("returns the chosen definitions in file order", () => {
    expect(chosenDefinitions(items, new Set([3, 1])).map((d) => d.pid)).toEqual([1, 3]);
  });

  test("returns nothing when no PID is chosen", () => {
    expect(chosenDefinitions(items, new Set())).toEqual([]);
  });

  test("ignores chosen PIDs that are not in the file", () => {
    expect(chosenDefinitions(items, new Set([9]))).toEqual([]);
  });
});
