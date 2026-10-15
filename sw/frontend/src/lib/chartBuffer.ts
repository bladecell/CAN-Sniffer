// Pure, DOM-free ring buffer of (time, value) samples for one PID.
// Capacity is fixed at creation (default 10 000 samples, about 160 KB), so memory per PID is bounded
// no matter how long the page stays open. The oldest sample is dropped when full.
// Push is O(1). Snapshot copies only the requested window, oldest first.

export const CHART_CAPACITY = 10_000;
/** Timestamps are ms since boot. Above this bound a value is hostile or corrupt, not a time. */
export const MAX_TIME_MS = 2 ** 40;
/** A backwards step larger than this is a device reboot, not reordering. */
export const REBOOT_JUMP_MS = 1000;

export interface ChartSnapshot {
  t: number[];
  /** null marks a gap (a break in the line) or a NaN sample. */
  v: (number | null)[];
}

export interface ChartBuffer {
  readonly size: number;
  readonly capacity: number;
  /** Timestamp of the newest sample, or undefined when empty. */
  lastTime(): number | undefined;
  /** Copy of the samples with t >= fromT (all when omitted), oldest first. */
  snapshot(fromT?: number): ChartSnapshot;
}

export interface ChartBufferWriter extends ChartBuffer {
  /** Returns false when the sample was rejected (implausible or out-of-order time). */
  push(t: number, v: number, splitAfterMs?: number): boolean;
}

export function createChartBuffer(capacity = CHART_CAPACITY): ChartBufferWriter {
  const times = new Float64Array(capacity);
  const values = new Float64Array(capacity); // NaN = gap
  let head = 0; // physical index of the oldest sample
  let count = 0;

  const physical = (logical: number) => (head + logical) % capacity;
  const newest = () => (count > 0 ? times[physical(count - 1)] : undefined);

  function append(t: number, v: number) {
    const idx = physical(count);
    times[idx] = t;
    values[idx] = v;
    if (count < capacity) count++;
    else head = (head + 1) % capacity;
  }

  // First logical index whose time is >= fromT. Times are non-decreasing.
  function lowerBound(fromT: number): number {
    let lo = 0;
    let hi = count;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (times[physical(mid)] < fromT) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  return {
    get size() {
      return count;
    },
    get capacity() {
      return capacity;
    },
    lastTime: newest,
    /**
     * Adds a sample. A gap longer than splitAfterMs first inserts a null break one ms before the sample.
     * Rejects non-finite or out-of-order timestamps (returns false) so the ordering stays valid.
     */
    push(t, v, splitAfterMs = Infinity) {
      if (!Number.isFinite(t) || t < 0 || t > MAX_TIME_MS) return false;
      const last = newest();
      if (last !== undefined) {
        if (t < last - REBOOT_JUMP_MS) {
          // lastUpdated counts from device boot: a large backwards jump means the device restarted.
          count = 0;
          head = 0;
        } else if (t < last) {
          return false;
        } else if (t - last > splitAfterMs) {
          append(Math.max(t - 1, last), Number.NaN);
        }
      }
      append(t, v);
      return true;
    },
    snapshot(fromT = -Infinity) {
      const from = lowerBound(fromT);
      const t: number[] = new Array(count - from);
      const v: (number | null)[] = new Array(count - from);
      for (let i = from; i < count; i++) {
        const idx = physical(i);
        t[i - from] = times[idx];
        const val = values[idx];
        v[i - from] = Number.isNaN(val) ? null : val;
      }
      return { t, v };
    },
  };
}
