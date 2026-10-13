import { describe, expect, test } from "bun:test";
import { parseObd2Status, parseCanBusStatus, parseDtcGroups, parseDtcDescriptions, buildDtcData } from "./guards";

const obd = { auto_poll: false, polling: false, pid_initialized: true, pid_def_count: 3, pid_data_count: 3, poll_task_utilization: 0.1, supported_pids: { count: 2, groups: [1, 2, 3, 4, 5, 6, 7] } };

describe("parseObd2Status", () => {
  test("accepts the documented shape", () => {
    expect(parseObd2Status(obd).supported_pids.groups).toHaveLength(7);
  });
  test("throws instead of dereferencing missing supported_pids", () => {
    const { supported_pids, ...rest } = obd;
    expect(() => parseObd2Status(rest)).toThrow();
    expect(() => parseObd2Status(null)).toThrow();
    expect(() => parseObd2Status({ ...obd, polling: "yes" })).toThrow();
  });
  test("non-numeric group entries are rejected", () => {
    expect(() => parseObd2Status({ ...obd, supported_pids: { count: 1, groups: ["x"] } })).toThrow();
  });
});

describe("parseCanBusStatus", () => {
  test("accepts and rejects", () => {
    expect(parseCanBusStatus({ state: "connected", bitrate: 500000 }).state).toBe("connected");
    expect(() => parseCanBusStatus({ bitrate: 5 })).toThrow();
    expect(() => parseCanBusStatus([])).toThrow();
  });
});

describe("parseDtcGroups", () => {
  test("accepts groups and drops non-string codes", () => {
    const g = parseDtcGroups({ dtcs: [{ mode: 3, dtc: ["P0301", 5, null], dtc_count: 3 }] });
    expect(g[0].dtc).toEqual(["P0301"]);
  });
  test("missing dtcs is an empty list, malformed throws", () => {
    expect(parseDtcGroups({})).toEqual([]);
    expect(() => parseDtcGroups({ dtcs: "x" })).toThrow();
    expect(() => parseDtcGroups({ dtcs: [null] })).toThrow();
  });
});

describe("parseDtcDescriptions", () => {
  test("builds a map and ignores bad items", () => {
    expect(parseDtcDescriptions({ status: "success", dtcs: [{ dtc: "P0301", description: "Misfire" }, { dtc: 1 }, null] })).toEqual({ P0301: "Misfire" });
    expect(parseDtcDescriptions({ status: "error" })).toEqual({});
  });
});

describe("buildDtcData", () => {
  test("applies descriptions and sums counts from the code lists", () => {
    const d = buildDtcData([{ mode: 3, dtc: ["P0301"], dtc_count: 1 }, { mode: 10, dtc: ["P0420", "P0171"], dtc_count: 2 }], { P0301: "Misfire" });
    expect(d.confirmed.dtc[0].description).toBe("Misfire");
    expect(d.permanent.dtc[0].description).toBe("Description not available");
    expect(d.pending.dtc_count).toBe(0);
  });
});

import { parseConfirmRequired } from "./guards";
import { serializePidDefinition, normalizePidDefinition } from "./pidDefinition";

describe("parseConfirmRequired", () => {
  const ok = { status: "confirm_required", token: "0123456789abcdef", expires_ms: 10000 };
  test("accepts a 409 confirm body", () => {
    expect(parseConfirmRequired(409, ok)).toEqual({ token: "0123456789abcdef", expiresMs: 10000 });
  });
  test("anything else is not a confirm request", () => {
    expect(parseConfirmRequired(200, ok)).toBeNull();
    expect(parseConfirmRequired(409, { ...ok, token: "xyz" })).toBeNull();
    expect(parseConfirmRequired(409, { ...ok, token: 5 })).toBeNull();
    expect(parseConfirmRequired(409, { ...ok, status: "error" })).toBeNull();
    expect(parseConfirmRequired(409, null)).toBeNull();
    expect(parseConfirmRequired(409, "x")).toBeNull();
  });
  test("bad expires_ms falls back to 10000", () => {
    expect(parseConfirmRequired(409, { ...ok, expires_ms: "a" })?.expiresMs).toBe(10000);
  });
});

describe("serializePidDefinition", () => {
  const base = { id: 0x7df, mode: 1, pid: 12, length: 2, name: "RPM", formula: "A", update_interval_ms: 256 };
  test("unset min/max stay unset (null), never rewritten to a default", () => {
    const out = serializePidDefinition(normalizePidDefinition({ ...base, minV: null, maxV: null }));
    expect(out.minV).toBeNull();
    expect(out.maxV).toBeNull();
    const json = JSON.parse(JSON.stringify(out));
    expect(json.minV).toBeNull();
    expect(json.maxV).toBeNull();
  });
  test("set values are kept", () => {
    const out = serializePidDefinition(normalizePidDefinition({ ...base, minV: 1, maxV: 9 }));
    expect([out.minV, out.maxV]).toEqual([1, 9]);
  });
});
