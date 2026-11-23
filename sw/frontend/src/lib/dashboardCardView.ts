// Pure view rules for the dashboard cards: levels, read states, control reasons, gauge and chart geometry, labels.
// No DOM and no stores, so it runs under bun test.
import type { Thresholds } from "./dashboardModel";
import { decimate, pathOf, windowPoints, type Pt } from "./telemetryChart/series";
import { SCROLL_HEADROOM_MS } from "./telemetryChart/scroll";
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

/** Polling state is not shown on cards: a stored value stays a value. */
export function readState(entry: PidValue | undefined, _polling: boolean | null): ReadState {
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
  /** Keep points outside [from, to] at their true x (drawn beyond the box, for a layer shifted under a clip). */
  unclamped?: boolean;
}

export interface ChartPaths {
  line: string;
  area: string;
  /** The newest drawn point, for the dot at the end of a trace. */
  end: { x: number; y: number } | null;
}

/**
 * Line and area paths for stored samples. Gaps break the line. Each run of samples gets its own area,
 * closed to the baseline, so a gap never draws a wedge across it. Points are decimated per pixel column.
 */
export function chartPaths(samples: { t: number[]; v: (number | null)[] }, box: ChartBox): ChartPaths {
  if (samples.t.length === 0) return { line: "", area: "", end: null };
  const span = box.to > box.from ? box.to - box.from : 1;
  const scale = box.hi > box.lo ? box.hi - box.lo : 1;
  const pts: Pt[] = samples.t.map((t, i) => {
    const v = samples.v[i];
    if (!finite(v)) return null;
    const f = (t - box.from) / span;
    const x = (box.unclamped ? f : clamp01(f)) * box.w;
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
  let end: { x: number; y: number } | null = null;
  for (let i = shown.length - 1; i >= 0 && end === null; i--) end = shown[i];
  return { line, area, end };
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

/**
 * The buffer samples a chart window needs: read from the window start less the headroom and a second, so the
 * segment that enters the left edge (windowPoints starts the headroom before it) is always covered.
 */
export function chartWindowRead(
  read: (pid: number, fromT: number) => { t: number[]; v: (number | null)[] },
  pid: number,
  to: number,
  spanMs: number,
): { t: number[]; v: (number | null)[] } {
  return read(pid, to - spanMs - SCROLL_HEADROOM_MS - 1000);
}

/**
 * A trace for the window [anchor - spanMs, anchor], built ahead of the present (telemetryChart/scroll.ts). A line holds
 * its last value to the anchor while fresh, and the margin before the window start is kept, so a layer shifted by the
 * scroll offset still covers both edges. Points outside the window are not clamped (the card clips them).
 */
export function traceShape(
  samples: { t: number[]; v: (number | null)[] },
  anchor: number,
  spanMs: number,
  holdMs: number,
  box: { w: number; h: number; lo: number; hi: number },
): ChartPaths {
  const drawn = windowPoints(samples.t, samples.v, anchor - spanMs - SCROLL_HEADROOM_MS, anchor, anchor, holdMs);
  return chartPaths(
    { t: drawn.map((p) => p.t), v: drawn.map((p) => p.v) },
    { ...box, from: anchor - spanMs, to: anchor, unclamped: true },
  );
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

/**
 * Drops the gap markers that sit between two samples at most maxMs apart. The buffer breaks the line after
 * 2 s, which for a slow PID is its normal pace; a sparkline should only break where data is really missing.
 */
export function bridgeGaps(samples: { t: number[]; v: (number | null)[] }, maxMs: number): { t: number[]; v: (number | null)[] } {
  const t: number[] = [];
  const v: (number | null)[] = [];
  let prevReal = -1;
  for (let i = 0; i < samples.t.length; i++) {
    if (samples.v[i] === null) {
      let next = i + 1;
      while (next < samples.t.length && samples.v[next] === null) next++;
      if (prevReal >= 0 && next < samples.t.length && samples.t[next] - prevReal <= maxMs) continue;
    } else prevReal = samples.t[i];
    t.push(samples.t[i]);
    v.push(samples.v[i]);
  }
  return { t, v };
}

/**
 * The value scale of a sparkline: the data span, widened to a tenth of the PID range (or 5 % of the value)
 * so that noise does not fill the whole strip with a steep line.
 */
export function sparkScale(lo: number, hi: number, range?: { min: number; max: number } | null): { lo: number; hi: number } {
  const full = range && range.max > range.min ? range.max - range.min : 0;
  const floor = full > 0 ? full * 0.1 : Math.max(Math.abs(lo), Math.abs(hi)) * 0.05 || 1;
  if (hi - lo >= floor) return { lo, hi };
  const mid = (lo + hi) / 2;
  return { lo: mid - floor / 2, hi: mid + floor / 2 };
}

/** Where the warning and critical thresholds sit on a PID range, as fractions 0 to 1, and which way is bad. */
export function zoneStops(range: { min: number; max: number }, t: Thresholds): { warn: number; critical: number; lowBad: boolean } {
  const at = (x: number) => clamp01((x - range.min) / (range.max - range.min));
  return { warn: at(t.warn), critical: at(t.critical), lowBad: t.warn > t.critical };
}

/** PID updates per second from a count taken over an interval (the Polling card's PIDs/s). */
export function updatesPerSecond(delta: number, elapsedMs: number): number {
  if (!Number.isFinite(delta) || !Number.isFinite(elapsedMs) || delta <= 0 || elapsedMs <= 0) return 0;
  return Math.round((delta * 1000) / elapsedMs);
}
