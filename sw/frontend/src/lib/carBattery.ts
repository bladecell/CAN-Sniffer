import { connectionStore } from "./connectionStore.svelte";
// Car battery (12 V) status rules shared by every visualization mode. Pure, no DOM.
// Source: ESP ADC battery_voltage in WS frame 0x03 (docs/api.md), read through connectionStore.wsCanStatus.

/** Display scale for the glyph, bar, gauge and chart. */
export const BATTERY_SCALE_MIN_V = 9.5;
export const BATTERY_SCALE_MAX_V = 15.5;
/** A value older than this (local clock) is no data. The device sends 0x03 at 1 Hz. */
export const BATTERY_STALE_MS = 3000;
/** Tolerated clock skew between the tick and a frame. Anything further in the future is no data. */
const BATTERY_SKEW_MS = 1000;
/** Below this a reading is not a car battery: the ADC floats or the sensor is faulty. It is no data, not Critical. */
export const BATTERY_MIN_VALID_V = 5;
/** Above this a value is hostile or corrupt, not a 12 V battery. */
const BATTERY_MAX_VALID_V = 20;

export type BatteryState = "nodata" | "critical" | "low" | "ok" | "charging" | "high";

export const BATTERY_LABEL: Record<BatteryState, string> = {
  critical: "Critical",
  low: "Low",
  ok: "OK, resting",
  charging: "Charging",
  high: "Too high",
  nodata: "No data",
};

/** Hex colours from the mockup (#f87171 for red text, not #ef4444). */
export const BATTERY_COLOR: Record<BatteryState, string> = {
  critical: "#f87171",
  low: "#f59e0b",
  ok: "#84da89",
  charging: "#84da89",
  high: "#f59e0b",
  nodata: "#8f9aab",
};

/** Band for a voltage. Lower bounds are inclusive: 10.5 V is low, 11.8 V is ok. */
export function batteryState(volts: number | null): BatteryState {
  if (volts === null) return "nodata";
  if (volts < 10.5) return "critical";
  if (volts < 11.8) return "low";
  if (volts < 13.2) return "ok";
  if (volts <= 15) return "charging";
  return "high";
}

/** Returns the voltage, or null for anything that is not a plausible battery reading. */
export function sanitizeVoltage(raw: unknown): number | null {
  if (typeof raw !== "number" || !Number.isFinite(raw)) return null;
  if (raw < BATTERY_MIN_VALID_V || raw > BATTERY_MAX_VALID_V) return null;
  return raw;
}

/** Position on the display scale, 0 at 9.5 V and 1 at 15.5 V, clamped. */
export function clampFraction(volts: number): number {
  const f = (volts - BATTERY_SCALE_MIN_V) / (BATTERY_SCALE_MAX_V - BATTERY_SCALE_MIN_V);
  return Math.min(1, Math.max(0, f));
}

export interface BatteryInput {
  volts: unknown;
  /** Local ms when the last 0x03 frame arrived, 0 when none has. */
  frameAt: number;
  /** Local ms now. */
  now: number;
  wsConnected: boolean;
  busUp: boolean;
}

export interface BatteryReading {
  state: BatteryState;
  /** null when there is no data. */
  volts: number | null;
  /** 0 when there is no data. */
  fraction: number;
}

/** The no-data rule: WebSocket down, bus down, value invalid, or the last frame older than 3 s. */
export function batteryReading(input: BatteryInput): BatteryReading {
  const volts = sanitizeVoltage(input.volts);
  const age = input.now - input.frameAt;
  // A frame can land just after the caller's clock tick, so a small negative age is still fresh.
  const fresh = input.frameAt > 0 && age >= -BATTERY_SKEW_MS && age <= BATTERY_STALE_MS;
  if (!input.wsConnected || !input.busUp || !fresh || volts === null) {
    return { state: "nodata", volts: null, fraction: 0 };
  }
  return { state: batteryState(volts), volts, fraction: clampFraction(volts) };
}

/** Short meaning under the value, shown in every mode. */
export const BATTERY_HINT: Record<BatteryState, string> = {
  charging: "Alternator running",
  ok: "Engine off, healthy",
  low: "Charge soon",
  critical: "Engine may not start",
  high: "Check the alternator",
  nodata: "No reading from the bus",
};
/** Chart sample clock: monotonic ms since page load. Date.now() (~1.8e12) exceeds the chart buffer's 2^40 time bound, so every sample was rejected. */
export const chartSampleTime = () => performance.now();
