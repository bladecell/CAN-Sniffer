// Builds everything one plot draws from its samples: axes, lines, lanes, end labels, legend, stats and cursor.
// Every number that reaches an SVG attribute is finite. Samples are drawn as stored (decimated per pixel column).

import { agoLabel, clamp, finiteOr, formatValue, niceTicks, timeStep } from "./math";
import { decimate, pathOf, statsOf, valueAt, windowPoints, type Pt } from "./series";
import { GUTTER_LEFT, GUTTER_RIGHT } from "./gutter";

export type PlotMode = "overlay" | "lanes";
export type Range = [number, number];

export interface PlotSeries {
  id: number;
  name: string;
  unit: string;
  color: string;
  /** The PID's own range from its definition, or null when it has none. */
  range: Range | null;
  /** Poll interval the hold rule is based on (ms). Defaults to 2 s. */
  holdMs?: number;
  /** Device ms, ascending. */
  t: number[];
  /** null is a gap. */
  v: (number | null)[];
}

export interface PlotInput {
  width: number;
  height: number;
  mode: PlotMode;
  fit: boolean;
  laneH: number;
  win: { len: number; endAgo: number };
  /** Device ms of the present (frozen while paused). */
  now: number;
  series: PlotSeries[];
  /** Cursor position as a fraction of the plot width, or null. */
  hoverF: number | null;
}

export interface PlotModel {
  /** The plot area; every series, area and cursor is clipped to it. */
  plot: { x: number; y: number; w: number; h: number };
  W: number;
  H: number;
  L: number;
  R: number;
  T: number;
  B: number;
  pw: number;
  bottom: number;
  hasData: boolean;
  caption: string | null;
  axisUnit: string | null;
  axis: { y: number; text: string }[];
  timeTicks: { x: number; text: string; anchor: "start" | "middle" | "end" }[];
  lines: { id: number; color: string; d: string; area: string | null }[];
  lanes: { id: number; name: string; unit: string; color: string; top: number; bottom: number; big: string; ticks: { y: number; text: string }[] }[];
  /** Swatch and name only; live values are drawn at the line ends, in the lane readouts and in the tooltip. */
  legend: { id: number; name: string; color: string }[];
  /** now: the value under the cursor, else at the window end (the latest sample while live); null when there is none. */
  stats: { id: number; name: string; unit: string; color: string; now: number | null; min: number; avg: number; max: number }[];
  cursor: { x: number; ago: number; label: string; top: number; bottom: number; chipY: number; rows: { id: number; name: string; unit: string; color: string; text: string; x: number; y: number | null }[] } | null;
}

const PERCENT_TICKS = [0, 25, 50, 75, 100];
const FRAC_LIMIT = 0.04;

/**
 * Position of v in [lo, hi] as a fraction, clamped a little past both ends. Scales by the largest magnitude first so
 * values near the float limit cannot overflow to NaN. A degenerate domain gives the middle.
 */
export function fracOf(v: number, lo: number, hi: number): number {
  const s = Math.max(Math.abs(lo), Math.abs(hi), Math.abs(v)) || 1;
  const span = hi / s - lo / s;
  const f = span > 0 ? (v / s - lo / s) / span : 0.5;
  return Number.isFinite(f) ? clamp(f, -FRAC_LIMIT, 1 + FRAC_LIMIT) : 0.5;
}

function validRange(r: Range | null): Range | null {
  return r && Number.isFinite(r[0]) && Number.isFinite(r[1]) && r[1] > r[0] ? r : null;
}

/**
 * Y domain of one series. A PID's own range unless fit is on or it has none; then the visible values.
 * pad adds 8% around the visible values (lanes and single plots); overlay fit keeps the exact min and max.
 */
export function domainFor(range: Range | null, values: number[], fit: boolean, pad: boolean): Range {
  const r = validRange(range);
  let lo = Infinity;
  let hi = -Infinity;
  for (const x of values) {
    if (x < lo) lo = x;
    if (x > hi) hi = x;
  }
  // The definition's range is the default, but a value outside it widens the domain: no line leaves the plot.
  if (!fit && r) {
    if (values.length === 0) return r;
    return [Math.min(r[0], lo), Math.max(r[1], hi)];
  }
  if (values.length === 0) return r ?? [0, 1];
  const p = pad ? (hi - lo) * 0.08 : 0;
  const a = lo - p;
  const b = hi + p;
  // A flat or huge value can leave no span at float precision: widen it relative to its size.
  if (!(b > a)) {
    const e = Math.max(1, Math.abs(lo) * 0.05);
    return [lo - e, lo + e];
  }
  return Number.isFinite(a) && Number.isFinite(b) ? [a, b] : [0, 1];
}

export function buildPlotModel(inp: PlotInput): PlotModel {
  const n = inp.series.length;
  const lanes = inp.mode === "lanes";
  const multi = !lanes && n > 1;
  const laneH = clamp(finiteOr(inp.laneH, 90), 40, 1000);
  const W = Math.max(0, finiteOr(inp.width, 0));
  const L = GUTTER_LEFT;
  const R = GUTTER_RIGHT;
  // Top band: the caption (multi-PID overlay) or the unit (single PID) sits 10 px above the plot area.
  const T = lanes ? 8 : n === 1 ? 20 : 22;
  // Bottom margin: the tick labels row, then the cursor's time chip on its own row. Reserved in every plot, so the
  // plot area does not move when the cursor appears.
  const B = 48;
  const H = lanes ? T + Math.max(n, 1) * laneH + B : clamp(finiteOr(inp.height, 240), 120, 4000);
  const pw = Math.max(1, W - L - R);
  const bottom = lanes ? T + Math.max(n, 1) * laneH : H - B;
  const len = Math.max(1, finiteOr(inp.win.len, 60));
  const endAgo = Math.max(0, finiteOr(inp.win.endAgo, 0));
  const now = finiteOr(inp.now, 0);
  const fromT = now - (endAgo + len) * 1000;
  const toT = now - endAgo * 1000;
  const X = (ago: number) => L + (1 - (ago - endAgo) / len) * pw;
  const xOfT = (t: number) => X((now - t) / 1000);
  const inWindow = (t: number) => Number.isFinite(t) && t >= fromT - 1000 && t <= toT + 1000;

  const visible = inp.series.map((s) => {
    const vals: number[] = [];
    for (let k = 0; k < s.t.length; k++) {
      const x = s.v[k];
      if (typeof x === "number" && Number.isFinite(x) && s.t[k] >= fromT && s.t[k] <= toT) vals.push(x);
    }
    return vals;
  });
  const doms = inp.series.map((s, i) => domainFor(s.range, visible[i], inp.fit, !multi));
  const top = (i: number) => (lanes ? T + i * laneH + 22 : T);
  const bot = (i: number) => (lanes ? T + (i + 1) * laneH - 10 : bottom);
  const yOf = (i: number, v: number) => {
    const [lo, hi] = doms[i];
    if (multi) return bottom - fracOf(v, lo, hi) * (bottom - T);
    return bot(i) - fracOf(v, lo, hi) * (bot(i) - top(i));
  };

  const lines: PlotModel["lines"] = [];
  inp.series.forEach((s, i) => {
    // Edge-to-edge samples: interpolated at the window edges, held to the present while fresh (series.ts).
    const pts: Pt[] = windowPoints(s.t, s.v, fromT, toT, now, s.holdMs ?? 2000).map((p) =>
      p.v === null ? null : { x: xOfT(p.t), y: yOf(i, p.v) },
    );
    const dec = decimate(pts, pw);
    const d = pathOf(dec);
    // One closed shape per run of samples: a gap must not be bridged by the fill.
    const runs: Pt[][] = [[]];
    for (const p of dec) p === null ? runs.push([]) : runs[runs.length - 1].push(p);
    const base = bot(i).toFixed(1);
    const area =
      lanes || n === 1
        ? runs
            .filter((r) => r.length > 0)
            .map((r) => {
              const f = r[0] as { x: number; y: number };
              const l = r[r.length - 1] as { x: number; y: number };
              return `${pathOf(r)}L${l.x.toFixed(1)},${base}L${f.x.toFixed(1)},${base}Z`;
            })
            .join("") || null
        : null;
    lines.push({ id: s.id, color: s.color, d, area });
  });

  const hasData = visible.some((vals) => vals.length > 0);

  const legend = inp.series.map((s) => ({ id: s.id, name: s.name, color: s.color }));

  // Now: the value under the cursor while it is on the plot, else the value at the window end.
  const nowAt = inp.hoverF !== null && Number.isFinite(inp.hoverF) ? now - (endAgo + (1 - clamp(inp.hoverF, 0, 1)) * len) * 1000 : toT;
  const stats = inp.series.flatMap((s) => {
    const st = statsOf(s.t, s.v, fromT, toT);
    return st ? [{ id: s.id, name: s.name, unit: s.unit, color: s.color, now: valueAt(s.t, s.v, nowAt), ...st }] : [];
  });

  // Cursor, shared by every plot; the values under it.
  let cursor: PlotModel["cursor"] = null;
  if (inp.hoverF !== null && Number.isFinite(inp.hoverF)) {
    const f = clamp(inp.hoverF, 0, 1);
    const ago = endAgo + (1 - f) * len;
    const at = now - ago * 1000;
    const rows = inp.series.map((s, i) => {
      const v = valueAt(s.t, s.v, at);
      return {
        id: s.id,
        name: s.name,
        unit: s.unit,
        color: s.color,
        text: v === null ? "N/A" : formatValue(v),
        x: L + f * pw,
        y: v === null ? null : yOf(i, v),
      };
    });
    cursor = { x: L + f * pw, ago, label: agoLabel(ago), top: T, bottom, chipY: bottom + 26, rows };
  }

  // Axes and lane scales.
  const axis: PlotModel["axis"] = [];
  const lanesOut: PlotModel["lanes"] = [];
  if (multi) {
    for (const p of PERCENT_TICKS) axis.push({ y: bottom - (p / 100) * (bottom - T), text: `${p}%` });
  } else if (lanes) {
    inp.series.forEach((s, i) => {
      const [lo, hi] = doms[i];
      const ticks = niceTicks(lo, hi, 2).map((v) => ({ y: yOf(i, v), text: formatValue(v) }));
      const at = cursor ? valueAt(s.t, s.v, now - cursor.ago * 1000) : valueAt(s.t, s.v, toT);
      lanesOut.push({ id: s.id, name: s.name, unit: s.unit, color: s.color, top: top(i), bottom: bot(i), big: at === null ? "N/A" : formatValue(at), ticks });
    });
  } else if (n === 1) {
    const [lo, hi] = doms[0];
    for (const v of niceTicks(lo, hi, 4)) axis.push({ y: yOf(0, v), text: formatValue(v) });
  }

  // Time axis: labels count back from the present.
  const step = timeStep(len, pw);
  const timeTicks: PlotModel["timeTicks"] = [];
  for (let k = Math.ceil(endAgo / step); k * step <= endAgo + len + 1e-9; k++) {
    const ago = k * step;
    const x = X(ago);
    if (x < L - 1 || x > W - R + 1) continue;
    timeTicks.push({ x, text: agoLabel(ago), anchor: x > W - R - 14 ? "end" : x < L + 10 ? "start" : "middle" });
  }

  return {
    plot: { x: L, y: T, w: pw, h: bottom - T },
    W,
    H,
    L,
    R,
    T,
    B,
    pw,
    bottom,
    hasData,
    caption: multi ? (inp.fit ? "% of each PID's visible range" : "% of each PID's own range") : null,
    axisUnit: !lanes && n === 1 ? inp.series[0].unit : null,
    axis,
    timeTicks,
    lines,
    lanes: lanesOut,
    legend,
    stats,
    cursor,
  };
}
