/** Number stepper maths. Pure, so it is unit-tested (stepper.test.ts). */

export interface StepOpts {
  step: number;
  min?: number;
  max?: number;
  /** PageUp / PageDown move by step * multiplier. */
  multiplier?: number;
}

function decimalsOf(n: number): number {
  const s = String(n);
  const dot = s.indexOf(".");
  return dot < 0 ? 0 : s.length - dot - 1;
}

export function stepValue(value: number | undefined, dir: 1 | -1, opts: StepOpts): number {
  const base = value === undefined || Number.isNaN(value) ? 0 : value;
  const delta = opts.step * (opts.multiplier ?? 1) * dir;
  const decimals = Math.max(decimalsOf(opts.step), decimalsOf(base));
  let next = Number((base + delta).toFixed(decimals));
  if (opts.min !== undefined) next = Math.max(opts.min, next);
  if (opts.max !== undefined) next = Math.min(opts.max, next);
  return next;
}
