// Pure view rules for the dashboard cards: levels, read states, control reasons, gauge and chart geometry, labels.
// No DOM and no stores, so it runs under bun test.
import type { Thresholds } from "./dashboardModel";
import { decimate, pathOf, type Pt } from "./telemetryChart/series";
import type { PidValue } from "./types";

export type Level = "ok" | "warn" | "crit";
export type ReadState = "value" | "nodata" | "off";

const RANK: Record<Level, number> = { ok: 0, warn: 1, crit: 2 };
const finite = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);

/** Level of a reading. Warn below critical means high values are bad; warn above critical means low values are bad. */
export function levelOf(value: number | null, t: Thresholds | undefined): Level | null {
  if (!finite(value) || !t) return null;
  if (t.warn < t.critical) return value >= t.critical ? "crit" : value >= t.warn ? "warn" : "ok";
  return value <= t.critical ? "crit" : value <= t.warn ? "warn" : "ok";
}

/** The worst of several levels. Null when none of them has a level. */
export function worstLevel(levels: (Level | null)[]): Level | null {
  let worst: Level | null = null;
  for (const l of levels) if (l && (worst === null || RANK[l] > RANK[worst])) worst = l;
  return worst;
}

/** Polling off wins: a stored value is then no longer refreshed. */
export function readState(entry: PidValue | undefined, polling: boolean | null): ReadState {
  if (polling === false) return "off";
  if (!entry || !entry.isValid || !entry.isSupported) return "nodata";
  return "value";
}

export interface ControlInput {
  linked: boolean;
  busUp: boolean;
  /** null when the SD state is not known yet, which does not block the control. */
  sdPresent: boolean | null;
  needsLogin: boolean;
  loggedIn: boolean;
}

/** One line for a disabled control, or null when it can run. Checked in the order the card shows them. */
export function controlReason(i: ControlInput): string | null {
  if (!i.linked) return "Device not connected";
  if (!i.busUp) return "Bus disconnected";
  if (i.sdPresent === false) return "No SD card";
  if (i.needsLogin && !i.loggedIn) return "Log in to control";
  return null;
}

/** A reading as text. Values of 100 and up have no decimals. */
export function formatReading(v: number | null): string {
  if (!finite(v)) return "—";
  return Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1);
}

/** Font size for a number of `chars` characters inside `width` units: the base size, or smaller so it fits (bold digits are about 0.7 em wide). */
export function fitFont(base: number, chars: number, width: number): number {
  if (chars <= 0 || !(width > 0)) return base;
  return Math.min(base, width / (0.7 * chars));
}

/** The line that says why a card shows no reading. A value needs no note. */
export function emptyNote(state: ReadState): string {
  return state === "nodata" ? "No reading yet" : state === "off" ? "Polling is off" : "";
}

// Dials run clockwise on screen, angles in degrees from 3 o'clock. Gauge styles (mockup GEO): arc is the top
// half, dial is 270 degrees from bottom left, ring is the full circle.
export interface Sweep {
  start: number;
  sweep: number;
}
export const GAUGE_SWEEP = {
  arc: { start: 180, sweep: 180 },
  dial: { start: 135, sweep: 270 },
  ring: { start: -90, sweep: 360 },
} as const satisfies Record<string, Sweep>;

const clamp01 = (f: number) => Math.min(1, Math.max(0, f));
const ARC_DIAL = GAUGE_SWEEP.dial;

/** A point on a dial at a fraction of its sweep. */
export function sweepPoint(cx: number, cy: number, r: number, s: Sweep, fraction: number): { x: number; y: number } {
  const theta = ((s.start + s.sweep * clamp01(fraction)) * Math.PI) / 180;
  return { x: cx + r * Math.cos(theta), y: cy + r * Math.sin(theta) };
}

/** The arc between two fractions of the sweep. The large-arc flag is set past 180 degrees. */
export function sweepSpan(cx: number, cy: number, r: number, s: Sweep, from: number, to: number): string {
  const f0 = clamp01(from);
  const f1 = clamp01(to);
  const a = sweepPoint(cx, cy, r, s, f0);
  const b = sweepPoint(cx, cy, r, s, f1);
  const large = (f1 - f0) * s.sweep > 180 ? 1 : 0;
  return `M${a.x.toFixed(2)},${a.y.toFixed(2)} A${r},${r} 0 ${large} 1 ${b.x.toFixed(2)},${b.y.toFixed(2)}`;
}

/** The arc from the start of the sweep to a fraction of it. */
export function sweepPath(cx: number, cy: number, r: number, s: Sweep, fraction: number): string {
  return sweepSpan(cx, cy, r, s, 0, fraction);
}

export function arcPoint(cx: number, cy: number, r: number, fraction: number): { x: number; y: number } {
  return sweepPoint(cx, cy, r, ARC_DIAL, fraction);
}

/** The 270-degree dial arc from the start to the given fraction. */
export function arcPath(cx: number, cy: number, r: number, fraction: number): string {
  return sweepPath(cx, cy, r, ARC_DIAL, fraction);
}

export interface ChartBox {
  w: number;
  h: number;
  /** Time window in device ms, and the value scale. A degenerate scale is widened to one unit. */
  from: number;
  to: number;
  lo: number;
  hi: number;
}

export interface ChartPaths {
  line: string;
  area: string;
}

/**
 * Line and area paths for stored samples. Gaps break the line. Each run of samples gets its own area,
 * closed to the baseline, so a gap never draws a wedge across it. Points are decimated per pixel column.
 */
export function chartPaths(samples: { t: number[]; v: (number | null)[] }, box: ChartBox): ChartPaths {
  if (samples.t.length === 0) return { line: "", area: "" };
  const span = box.to > box.from ? box.to - box.from : 1;
  const scale = box.hi > box.lo ? box.hi - box.lo : 1;
  const pts: Pt[] = samples.t.map((t, i) => {
    const v = samples.v[i];
    if (!finite(v)) return null;
    const x = clamp01((t - box.from) / span) * box.w;
    // A flat scale (constant values) draws mid-height instead of on the baseline.
    const y = box.hi > box.lo ? box.h - clamp01((v - box.lo) / scale) * box.h : box.h / 2;
    return { x, y };
  });
  const shown = decimate(pts, Math.max(1, Math.ceil(box.w)));
  const line = pathOf(shown);

  let area = "";
  let run: { x: number; y: number }[] = [];
  const close = () => {
    if (run.length >= 2) {
      const first = run[0];
      const last = run[run.length - 1];
      area += `${pathOf(run)}L${last.x.toFixed(1)},${box.h.toFixed(1)}L${first.x.toFixed(1)},${box.h.toFixed(1)}Z`;
    }
    run = [];
  };
  for (const p of shown) {
    if (p === null) close();
    else run.push(p);
  }
  close();
  return { line, area };
}

const WIFI_MODE: Record<number, string> = {
  1: "Station",
  2: "Access point",
  3: "Access point and station",
};

export function wifiModeLabel(mode: number | null): string {
  return (mode !== null && WIFI_MODE[mode]) || "Unknown";
}

export function uptimeLabel(seconds: number): string {
  const s = Math.max(0, Math.floor(seconds));
  if (s < 60) return `${s} s`;
  if (s < 3600) return `${Math.floor(s / 60)} min`;
  return `${Math.floor(s / 3600)} h ${Math.floor((s % 3600) / 60)} min`;
}

/** The samples in the last `spanMs` before the newest one. Times are device ms, ascending. */
export function windowOf(samples: { t: number[]; v: (number | null)[] }, spanMs: number): { t: number[]; v: (number | null)[] } {
  const n = samples.t.length;
  if (n === 0) return { t: [], v: [] };
  const from = samples.t[n - 1] - spanMs;
  let i = n - 1;
  while (i > 0 && samples.t[i - 1] >= from) i--;
  return { t: samples.t.slice(i), v: samples.v.slice(i) };
}

/** Where the warning and critical thresholds sit on a PID range, as fractions 0 to 1, and which way is bad. */
export function zoneStops(range: { min: number; max: number }, t: Thresholds): { warn: number; critical: number; lowBad: boolean } {
  const at = (x: number) => clamp01((x - range.min) / (range.max - range.min));
  return { warn: at(t.warn), critical: at(t.critical), lowBad: t.warn > t.critical };
}
