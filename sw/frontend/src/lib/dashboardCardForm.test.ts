import { describe, expect, test } from "bun:test";
import { defaultThresholds, editFaceName, pidSlots, rangeOf, thresholdsError, thresholdsApply, needsRange } from "./dashboardCardForm";

describe("rangeOf", () => {
  test("returns the PID's min and max", () => {
    expect(rangeOf({ minValue: 0, maxValue: 255 })).toEqual({ min: 0, max: 255 });
  });

  test("no range when min and max are unset or equal", () => {
    expect(rangeOf({ minValue: NaN, maxValue: NaN })).toBeNull();
    expect(rangeOf({ minValue: 5, maxValue: 5 })).toBeNull();
  });
});

describe("defaultThresholds", () => {
  test("80% warn and 90% critical of the range", () => {
    // 0..200: warn at 160, critical at 180.
    expect(defaultThresholds({ min: 0, max: 200 })).toEqual({ warn: 160, critical: 180 });
  });

  test("20% warn and 10% critical when low values are the problem", () => {
    expect(defaultThresholds({ min: 0, max: 200 }, true)).toEqual({ warn: 40, critical: 20 });
  });

  test("uses the range origin, not zero", () => {
    expect(defaultThresholds({ min: 10, max: 110 })).toEqual({ warn: 90, critical: 100 });
  });

  test("no range means no defaults", () => {
    expect(defaultThresholds(null)).toBeNull();
  });
});

describe("thresholdsError", () => {
  const range = { min: 0, max: 200 };

  test("accepts a pair inside the range, in either direction", () => {
    expect(thresholdsError(range, { warn: 160, critical: 180 })).toBeNull();
    expect(thresholdsError(range, { warn: 40, critical: 20 })).toBeNull();
  });

  test("rejects equal, out of range, and non-finite values", () => {
    expect(thresholdsError(range, { warn: 100, critical: 100 })).toMatch(/different/);
    expect(thresholdsError(range, { warn: 160, critical: 300 })).toMatch(/range/);
    expect(thresholdsError(range, { warn: NaN, critical: 10 })).toMatch(/number/);
  });

  test("without a range any finite pair is accepted", () => {
    expect(thresholdsError(null, { warn: 1, critical: 2 })).toBeNull();
  });
});

describe("pidSlots", () => {
  test("PID count limits per type and style", () => {
    expect(pidSlots("value", "big", 0)).toEqual([1, 1]);
    expect(pidSlots("gauge", "arc", 0)).toEqual([1, 3]);
    expect(pidSlots("gauge", "dial", 0)).toEqual([1, 2]);
    expect(pidSlots("chart", "line", 0)).toEqual([1, 4]);
    expect(pidSlots("dtc", "rows", 0)).toEqual([0, 0]);
  });

  test("overview items share the cap of six with the VIN and DTC count", () => {
    expect(pidSlots("overview", "grid", 0)).toEqual([0, 6]);
    expect(pidSlots("overview", "grid", 2)).toEqual([0, 4]);
  });
});

describe("thresholdsApply and needsRange", () => {
  test("thresholds apply to one-PID value, bar and gauge cards", () => {
    expect(thresholdsApply("value", 1)).toBe(true);
    expect(thresholdsApply("bar", 1)).toBe(true);
    expect(thresholdsApply("gauge", 2)).toBe(false);
    expect(thresholdsApply("chart", 1)).toBe(false);
  });

  test("only the zones bar style asks for a PID range", () => {
    expect(needsRange("bar", "zones")).toBe(true);
    expect(needsRange("bar", "plain")).toBe(false);
  });
});

describe("editFaceName", () => {
  const names: Record<number, string> = { 12: "Engine RPM", 5: "Coolant" };
  test("joins the PID names and falls back to PID n for an unknown one", () => {
    expect(editFaceName({ type: "chart", pids: [12, 5] }, (p) => names[p])).toBe("Engine RPM and Coolant");
    expect(editFaceName({ type: "value", pids: [99] }, (p) => names[p])).toBe("PID 99");
  });
  test("a card without a PID shows its type", () => {
    expect(editFaceName({ type: "dtc", pids: [] }, () => undefined)).toBe("DTC list");
    expect(editFaceName({ type: "overview", pids: [] }, () => undefined)).toBe("Overview");
  });
});
