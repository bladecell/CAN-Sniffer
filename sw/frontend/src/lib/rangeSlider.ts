// Pure maths of the one- and two-thumb RangeSlider, so it runs under bun test.

export interface RangeSpec {
  min: number;
  max: number;
  /** Pointer and typed values snap to this. Omit it on a non-linear slider, whose own mapping already returns valid values. */
  step?: number;
  /** Smallest distance between two thumbs. */
  gap?: number;
}

const tidy = (n: number) => Number(n.toFixed(6));

export const snap = (v: number, { min, max, step }: RangeSpec): number => Math.min(max, Math.max(min, step ? tidy(Math.round(v / step) * step) : v));

/** Which thumb a press at `v` takes: the nearer one; at a tie the lower one when the press is left of both, else the upper. */
export function nearestThumb(values: readonly number[], v: number): number {
  if (values.length < 2) return 0;
  const dl = Math.abs(v - values[0]);
  const dh = Math.abs(v - values[1]);
  return dl < dh || (dl === dh && v < values[0]) ? 0 : 1;
}

/** `values` with thumb `i` moved to `v`: snapped, in range, never closer than `gap` to the other thumb. */
export function moveThumb(values: readonly number[], i: number, v: number, spec: RangeSpec): number[] {
  const x = snap(v, spec);
  const out = [...values];
  if (values.length === 2) {
    out[i] = i === 0 ? Math.min(x, tidy(values[1] - (spec.gap ?? 0))) : Math.max(x, tidy(values[0] + (spec.gap ?? 0)));
  } else {
    out[i] = x;
  }
  return out;
}

/** Key to target value: arrows move `keyStep`, PageUp and PageDown `page`, Home and End the ends. Undefined for other keys. */
export function keyTarget(cur: number, key: string, spec: Pick<RangeSpec, "min" | "max">, keyStep: number, page: number): number | undefined {
  const moves: Record<string, number> = {
    ArrowRight: cur + keyStep,
    ArrowUp: cur + keyStep,
    ArrowLeft: cur - keyStep,
    ArrowDown: cur - keyStep,
    PageUp: cur + page,
    PageDown: cur - page,
    Home: spec.min,
    End: spec.max,
  };
  return moves[key];
}

/** The tick a press at track position `x` (0..1) lands on when one is within `tolPx` pixels of it, else null. */
export function tickAt(x: number, ticks: readonly number[], toPos: (v: number) => number, trackPx: number, tolPx = 10): number | null {
  if (trackPx <= 0) return null;
  let best: number | null = null;
  let bestPx = tolPx;
  for (const t of ticks) {
    const px = Math.abs(toPos(t) - x) * trackPx;
    if (px <= bestPx) {
      best = t;
      bestPx = px;
    }
  }
  return best;
}
