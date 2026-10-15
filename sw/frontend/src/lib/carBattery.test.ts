import { describe, expect, test } from "bun:test";
import { createChartBuffer } from "./chartBuffer";
import {
  batteryReading,
  batteryState,
  BATTERY_STALE_MS,
  chartSampleTime,
  clampFraction,
  sanitizeVoltage,
} from "./carBattery";

describe("batteryState bands (suggested thresholds, docs/power.md for 13.2 V)", () => {
  test.each([
    [10.49, "critical"],
    [10.5, "low"],
    [11.79, "low"],
    [11.8, "ok"],
    [13.19, "ok"],
    [13.2, "charging"],
    [15.0, "charging"],
    [15.01, "high"],
  ])("%p V is %p", (volts, state) => {
    expect(batteryState(volts)).toBe(state);
  });

  test("a missing value is no data", () => {
    expect(batteryState(null)).toBe("nodata");
  });
});

describe("sanitizeVoltage", () => {
  test("returns a plausible voltage unchanged", () => {
    expect(sanitizeVoltage(12.6)).toBe(12.6);
  });

  test.each([
    ["NaN", Number.NaN],
    ["Infinity", Number.POSITIVE_INFINITY],
    ["zero (no reading)", 0],
    ["negative", -1],
    ["above 20 V (hostile)", 25],
    ["a string", "12.6"],
    ["undefined", undefined],
  ])("%s is no data", (_label, raw) => {
    expect(sanitizeVoltage(raw)).toBeNull();
  });
});

describe("clampFraction (scale 9.5 to 15.5 V)", () => {
  test("maps the scale ends and middle", () => {
    expect(clampFraction(9.5)).toBe(0);
    expect(clampFraction(12.5)).toBeCloseTo(0.5, 10);
    expect(clampFraction(15.5)).toBe(1);
  });

  test("clamps values outside the scale", () => {
    expect(clampFraction(8)).toBe(0);
    expect(clampFraction(18)).toBe(1);
  });
});

describe("batteryReading: no-data rule", () => {
  const live = { wsConnected: true, busUp: true, frameAt: 1_000_000 };

  test("a fresh valid frame gives its state and fraction", () => {
    const r = batteryReading({ volts: 12.6, now: 1_000_500, ...live });
    expect(r.state).toBe("ok");
    expect(r.volts).toBe(12.6);
    expect(r.fraction).toBeCloseTo(0.5167, 4);
  });

  test("exactly at the stale limit is still live", () => {
    const r = batteryReading({ volts: 12.6, now: 1_000_000 + BATTERY_STALE_MS, ...live });
    expect(r.state).toBe("ok");
  });

  test("one ms past the stale limit is no data", () => {
    const r = batteryReading({ volts: 12.6, now: 1_000_000 + BATTERY_STALE_MS + 1, ...live });
    expect(r.state).toBe("nodata");
    expect(r.volts).toBeNull();
  });

  test("a frame that never arrived (frameAt 0) is no data", () => {
    const r = batteryReading({ volts: 12.6, now: 1_000_000, wsConnected: true, busUp: true, frameAt: 0 });
    expect(r.state).toBe("nodata");
  });

  test("bus down is no data even with a fresh value", () => {
    const r = batteryReading({ volts: 12.6, now: 1_000_100, wsConnected: true, busUp: false, frameAt: 1_000_000 });
    expect(r.state).toBe("nodata");
  });

  test("WebSocket down is no data", () => {
    const r = batteryReading({ volts: 12.6, now: 1_000_100, wsConnected: false, busUp: true, frameAt: 1_000_000 });
    expect(r.state).toBe("nodata");
  });

  test.each([
    ["zero", 0],
    ["NaN", Number.NaN],
    ["null", null],
  ])("a %s value is no data and keeps no fraction", (_label, volts) => {
    const r = batteryReading({ volts, now: 1_000_100, ...live });
    expect(r.state).toBe("nodata");
    expect(r.fraction).toBe(0);
  });

  test("a small negative age (frame landed between two clock ticks) is still fresh", () => {
    const r = batteryReading({ volts: 12.6, now: 999_700, ...live });
    expect(r.state).toBe("ok");
  });

  test("a large negative age (clock jump, frame far in the future) is no data", () => {
    const r = batteryReading({ volts: 12.6, now: 998_000, ...live });
    expect(r.state).toBe("nodata");
  });
});

describe("chart sample time", () => {
  test("is accepted by the chart buffer (Date.now() exceeds its 2^40 bound)", () => {
    const b = createChartBuffer();
    expect(b.push(chartSampleTime(), 12.5)).toBe(true);
    expect(b.size).toBe(1);
  });
});
