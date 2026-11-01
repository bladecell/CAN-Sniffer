// Stored samples to drawable points. Nothing is resampled on a clock: lines follow the samples in the buffer.

/** A drawn point, or null for a break in the line (a gap or a sample that is not a number). */
export type Pt = { x: number; y: number } | null;

/**
 * Min/max decimation per pixel column: each column keeps its first, last, lowest and highest point, in time order,
 * so peaks survive and the output is bounded by about four points per column. Short inputs pass through.
 */
export function decimate(pts: Pt[], columns: number): Pt[] {
  const cols = Math.max(1, Math.floor(columns));
  if (pts.length <= cols * 4) return pts;
  const out: Pt[] = [];
  let bucket: { x: number; y: number }[] = [];
  const flush = () => {
    if (bucket.length === 0) return;
    let lo = bucket[0];
    let hi = bucket[0];
    for (const p of bucket) {
      if (p.y < lo.y) lo = p;
      if (p.y > hi.y) hi = p;
    }
    const keep = new Set([bucket[0], lo, hi, bucket[bucket.length - 1]]);
    for (const p of bucket) if (keep.has(p)) out.push(p);
    bucket = [];
  };
  for (const p of pts) {
    if (p === null) {
      flush();
      out.push(null);
      continue;
    }
    if (bucket.length > 0 && Math.floor(p.x) !== Math.floor(bucket[0].x)) flush();
    bucket.push(p);
  }
  flush();
  return out;
}

/** Latest value at or before `at` (ascending times). Null before the first sample or on a gap. */
export function valueAt(t: number[], v: (number | null)[], at: number): number | null {
  let lo = 0;
  let hi = t.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (t[mid] <= at) lo = mid + 1;
    else hi = mid;
  }
  if (lo === 0) return null;
  const val = v[lo - 1];
  return typeof val === "number" && Number.isFinite(val) ? val : null;
}

/** Min, average and max of the finite samples with from <= t <= to. Null when there are none. */
export function statsOf(t: number[], v: (number | null)[], from: number, to: number) {
  let min = Infinity;
  let max = -Infinity;
  let sum = 0;
  let n = 0;
  for (let i = 0; i < t.length; i++) {
    const x = v[i];
    if (t[i] < from || t[i] > to || typeof x !== "number" || !Number.isFinite(x)) continue;
    min = Math.min(min, x);
    max = Math.max(max, x);
    sum += x;
    n++;
  }
  return n === 0 ? null : { min, avg: sum / n, max };
}

/** SVG path for points in order. A break (null) lifts the pen. A lone point gets a 0.01 px stub so it still draws. */
export function pathOf(pts: Pt[]): string {
  const parts: string[] = [];
  let cur: string | null = null;
  let count = 0;
  const end = () => {
    if (cur !== null) parts.push(count === 1 ? `${cur}l0.01,0` : cur);
    cur = null;
  };
  for (const p of pts) {
    if (p === null) {
      end();
      continue;
    }
    const xy = `${p.x.toFixed(1)},${p.y.toFixed(1)}`;
    if (cur === null) {
      cur = `M${xy}`;
      count = 1;
    } else {
      cur += `L${xy}`;
      count++;
    }
  }
  end();
  return parts.join("");
}

export interface TimedValue {
  t: number;
  v: number | null;
}

const lerp = (t0: number, v0: number, t1: number, v1: number, t: number) => v0 + ((v1 - v0) * (t - t0)) / (t1 - t0);

/**
 * The samples a plot draws for the window [fromT, toT], with the line running to both edges.
 * - The left edge is interpolated from the sample before it, so the line starts exactly at the window start.
 * - The right edge is interpolated the same way when the window ends between two samples.
 * - A live line holds its newest value to the present while that sample is no older than holdMs (about one or two
 *   poll intervals). An older sample does not extend, so a stale PID shows a gap instead of a fake flat line.
 * Null values are gaps. Output is ascending in time.
 */
export function windowPoints(
  t: number[],
  v: (number | null)[],
  fromT: number,
  toT: number,
  now: number,
  holdMs: number,
): TimedValue[] {
  const value = (k: number): number | null => {
    const x = v[k];
    return typeof x === "number" && Number.isFinite(x) ? x : null;
  };
  const ok = (k: number) => Number.isFinite(t[k]);
  let lo = 0;
  let hi = t.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (t[mid] < fromT) lo = mid + 1;
    else hi = mid;
  }
  const first = lo;
  const out: TimedValue[] = [];

  const before = first - 1;
  if (before >= 0 && first < t.length && ok(before) && ok(first)) {
    const a = value(before);
    const b = value(first);
    if (a !== null && b !== null && t[first] > t[before] && t[first] > fromT) {
      out.push({ t: fromT, v: lerp(t[before], a, t[first], b, fromT) });
    }
  }

  let k = first;
  while (k < t.length && t[k] <= toT) {
    if (ok(k)) out.push({ t: t[k], v: value(k) });
    k++;
  }

  // k is the first sample after the window. Interpolate the right edge from the last sample inside it.
  const inside = k - 1;
  if (inside >= first && k < t.length && ok(k) && ok(inside)) {
    const a = value(inside);
    const b = value(k);
    if (a !== null && b !== null && t[k] > t[inside]) out.push({ t: toT, v: lerp(t[inside], a, t[k], b, toT) });
  }

  // No sample before the window (the buffer starts inside it): a first sample less than one interval after the
  // start is held back to the start. Later starts are real missing history and stay empty.
  if (before < 0 && first + 1 < t.length && ok(first) && ok(first + 1) && t[first] > fromT) {
    const a = value(first);
    if (a !== null && t[first] - fromT <= t[first + 1] - t[first]) out.unshift({ t: fromT, v: a });
  }

  const endT = Math.min(toT, now);
  const last = out[out.length - 1];
  if (last) {
    if (last.v !== null && endT > last.t && endT - last.t <= holdMs) out.push({ t: endT, v: last.v });
  } else if (before >= 0 && ok(before) && value(before) !== null && endT - t[before] <= holdMs) {
    const held = value(before) as number;
    out.push({ t: fromT, v: held }, { t: endT, v: held });
  }
  return out;
}
