/** Poll interval slider and priority levels for the PID editor. Pure, so it is unit-tested (pollRate.test.ts). */

export const STATIC_MS = 0;
/** The slider floor, the same as MIN_PID_INTERVAL. */
export const SLIDER_MIN_MS = 16;
/** The slider ceiling. Saved values above it still load and save; the firmware allows 65535. */
export const SLIDER_MAX_MS = 10000;

/** Share of the track the Static stop takes, before the 16 ms start of the log scale. */
const STOP = 0.12;
const LOG_SPAN = Math.log(SLIDER_MAX_MS / SLIDER_MIN_MS);
const SNAP_MS = [50, 100, 250, 500, 1000, 5000, 10000];

/** Track position (0 to 1) for an interval. 0 is Static. */
export function intervalToPos(ms: number): number {
  if (ms <= 0) return 0;
  const x = STOP + ((1 - STOP) * Math.log(ms / SLIDER_MIN_MS)) / LOG_SPAN;
  return Math.min(1, Math.max(STOP, x));
}

/** Interval for a track position. Below 6% is Static, then the value is rounded and snapped to common steps. */
export function posToInterval(x: number): number {
  if (x < 0.06) return STATIC_MS;
  if (x <= STOP) return SLIDER_MIN_MS;
  let ms = SLIDER_MIN_MS * Math.exp(((x - STOP) / (1 - STOP)) * LOG_SPAN);
  const grain = ms < 100 ? 1 : ms < 1000 ? 10 : ms < 10000 ? 50 : 500;
  ms = Math.min(SLIDER_MAX_MS, Math.max(SLIDER_MIN_MS, Math.round(ms / grain) * grain));
  return SNAP_MS.find((s) => Math.abs(intervalToPos(ms) - intervalToPos(s)) < 0.025) ?? ms;
}

function stepSize(ms: number): number {
  return ms < 100 ? 10 : ms < 1000 ? 50 : ms < 5000 ? 250 : 1000;
}

/** Arrow up. From Static it returns the last polled interval, so the slider never forgets it. */
export function stepUp(ms: number, lastPolled: number): number {
  if (ms === 0) return lastPolled;
  if (ms >= SLIDER_MAX_MS) return ms;
  const s = stepSize(ms);
  return Math.min(SLIDER_MAX_MS, Math.max(SLIDER_MIN_MS, (Math.floor(ms / s) + 1) * s));
}

/** Error for the Interval field, or "" when it is valid. Loaded values of 1 to 15 ms are rejected by the store, so the field says why. */
export function intervalError(ms: number | undefined): string {
  if (ms === undefined) return "Set an interval";
  if (ms > 0 && ms < SLIDER_MIN_MS) return "Use Static, or 16 ms to 10 s";
  return "";
}

/** Arrow down. Anything that would land under 16 ms becomes Static. */
export function stepDown(ms: number): number {
  if (ms === 0) return STATIC_MS;
  const s = stepSize(ms);
  const next = (Math.ceil(ms / s) - 1) * s;
  return next < SLIDER_MIN_MS ? STATIC_MS : next;
}

/** Short readout beside the slider. */
export function brief(ms: number): string {
  if (ms === 0) return "Static";
  if (ms >= 1000) return `${+(ms / 1000).toFixed(2)} s`;
  return `${ms} ms`;
}

/** One plain sentence describing the interval. */
export function describeInterval(ms: number): string {
  if (ms === 0) return "Read once, never polled.";
  if (ms < 1000) {
    const perSecond = +(1000 / ms).toPrecision(3);
    return `${perSecond} times per second, every ${ms} ms.`;
  }
  if (ms === 1000) return "Once a second.";
  return `Every ${+(ms / 1000).toFixed(2)} seconds.`;
}

/** Five levels, highest first. The stored value is 2 to 6, so level 1 is saved as 2. */
export const PRIORITY_LABELS = ["Highest", "High", "Normal", "Low", "Lowest"] as const;
const PRIORITY_BASE = 2;
const NORMAL_LEVEL = 2;

/** Index 0 to 4 of the level, for a saved priority. A value outside 2 to 6 shows the nearest level. */
export function priorityToLevel(saved: number | undefined): number {
  if (saved === undefined) return NORMAL_LEVEL;
  return Math.min(PRIORITY_LABELS.length - 1, Math.max(0, saved - PRIORITY_BASE));
}

/** The saved priority for a level index 0 to 4. */
export function levelToPriority(level: number): number {
  return level + PRIORITY_BASE;
}

/** The value written on save: a stored 0 or 1 is reserved for commands, so it becomes 2. Other values are kept. */
export function savePriority(saved: number): number {
  return saved < PRIORITY_BASE ? PRIORITY_BASE : saved;
}

/** Static PIDs are never queued, so priority does not apply to them. */
export function priorityEnabled(ms: number): boolean {
  return ms !== STATIC_MS;
}

const LADDER_MS = [50, 100, 250, 500, 1000, 2000, 5000, 10000];

/** Page up: the next step on the ladder above the interval. */
export function pageUp(ms: number): number {
  return LADDER_MS.find((s) => s > ms) ?? SLIDER_MAX_MS;
}

/** Page down: the next step on the ladder below the interval, or Static. */
export function pageDown(ms: number): number {
  if (ms === 0) return STATIC_MS;
  return [...LADDER_MS].reverse().find((s) => s < ms) ?? STATIC_MS;
}
