import { describe, expect, test } from "bun:test";
import { dtcRows, dtcSummary, explanation, priorityFor } from "./diagnosticsView";
import type { DtcData, DtcModeData } from "./types";

function group(mode: number, codes: string[]): DtcModeData {
  return {
    mode,
    dtc_count: codes.length,
    dtc: codes.map((dtc) => ({ dtc, description: "Description not found", mode })),
  };
}

function data(confirmed: string[], pending: string[], permanent: string[]): DtcData {
  return { confirmed: group(3, confirmed), pending: group(7, pending), permanent: group(10, permanent) };
}

describe("dtcRows", () => {
  test("lists confirmed, then pending, then permanent", () => {
    const rows = dtcRows(data(["P0301"], ["P0171"], ["P0420"]));
    expect(rows.map((r) => [r.group, r.code])).toEqual([
      ["confirmed", "P0301"],
      ["pending", "P0171"],
      ["permanent", "P0420"],
    ]);
  });

  test("labels each row with its group and OBD mode", () => {
    const [confirmed, permanent] = dtcRows(data(["P0301"], [], ["P0420"]));
    expect(confirmed).toMatchObject({ groupLabel: "Confirmed", mode: "03" });
    expect(permanent).toMatchObject({ groupLabel: "Permanent", mode: "0A" });
  });

  test("keys stay unique when a code repeats across groups", () => {
    const rows = dtcRows(data(["P0420"], [], ["P0420"]));
    expect(new Set(rows.map((r) => r.key)).size).toBe(2);
  });

  test("no codes gives no rows", () => {
    expect(dtcRows(data([], [], []))).toEqual([]);
  });
});

describe("dtcSummary", () => {
  test("active faults lead and the rest follow", () => {
    expect(dtcSummary(data(["P0301", "P0302"], ["P0171"], ["P0420"]))).toEqual({
      tone: "crit",
      text: "2 active faults",
      rest: "1 pending, 1 permanent",
    });
  });

  test("singular wording for one fault", () => {
    expect(dtcSummary(data(["P0301"], [], []))).toEqual({ tone: "crit", text: "1 active fault", rest: "" });
  });

  test("pending only is a warning, not an active fault", () => {
    expect(dtcSummary(data([], ["P0171"], []))).toEqual({ tone: "warn", text: "1 pending fault", rest: "" });
  });

  test("permanent only is a warning", () => {
    expect(dtcSummary(data([], [], ["P0420", "P0430"]))).toEqual({ tone: "warn", text: "2 permanent faults", rest: "" });
  });

  test("no codes is healthy", () => {
    expect(dtcSummary(data([], [], []))).toEqual({ tone: "ok", text: "Powertrain systems healthy", rest: "" });
  });
});

describe("explanation", () => {
  test("keeps a real description", () => {
    expect(explanation("Cylinder 1 Misfire Detected")).toBe("Cylinder 1 Misfire Detected");
  });

  test("the firmware's unknown-code text and an empty string are not explanations", () => {
    expect(explanation("Description not found")).toBe("");
    expect(explanation("")).toBe("");
    expect(explanation("   ")).toBe("");
  });
});

describe("priorityFor", () => {
  test("maps code families to the existing heuristic labels", () => {
    expect(priorityFor("P0524")).toBe("Stop safely");
    expect(priorityFor("P0217")).toBe("Stop safely");
    expect(priorityFor("P0301")).toBe("Urgent");
    expect(priorityFor("P0171")).toBe("Attend soon");
    expect(priorityFor("P0700")).toBe("Service trans");
    expect(priorityFor("U0100")).toBe("Network");
    expect(priorityFor("C0035")).toBe("Inspect");
    expect(priorityFor("B1000")).toBe("Inspect");
    expect(priorityFor("P1000")).toBe("Logged");
  });
});
