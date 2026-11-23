// One requestAnimationFrame loop for chart work. A request runs once at the next allowed frame; a subscriber runs on
// every allowed frame. Frames are capped at maxFps. The loop stops when idle or inactive and restarts on the next call.

export type FrameFn = (time: number) => void;

/** The browser's frame source, injectable for tests. */
export interface FrameSource {
  request(cb: FrameFn): unknown;
}

export interface FrameLoop {
  /** Runs fn once at the next allowed frame. Repeated requests before that frame run it once. */
  request(fn: FrameFn): void;
  /** Runs fn on every allowed frame until the returned function is called. */
  subscribe(fn: FrameFn): () => void;
  /** Restarts the loop if it stopped for being inactive. Call when the reason for inactivity ends. */
  kick(): void;
}

/** rAF timestamps jitter by a fraction of a millisecond; without slack a 60 Hz frame can be skipped. */
const JITTER_MS = 2;

export function createFrameLoop(
  source: FrameSource,
  opts: { maxFps: () => number; active: () => boolean },
): FrameLoop {
  const once = new Set<FrameFn>();
  const subs = new Set<FrameFn>();
  let handle: unknown = null;
  let lastRun = -Infinity;

  function schedule() {
    if (handle !== null || (once.size === 0 && subs.size === 0)) return;
    handle = source.request(tick);
  }

  function tick(time: number) {
    handle = null;
    if (!opts.active()) return;
    if (time - lastRun < 1000 / opts.maxFps() - JITTER_MS) return schedule();
    lastRun = time;
    // The next frame is scheduled even if a callback throws, so one bad callback cannot stop every chart.
    try {
      const ran = [...once];
      once.clear();
      for (const fn of ran) fn(time);
      for (const fn of [...subs]) fn(time);
    } finally {
      schedule();
    }
  }

  return {
    request(fn) {
      once.add(fn);
      schedule();
    },
    subscribe(fn) {
      subs.add(fn);
      schedule();
      return () => {
        subs.delete(fn);
      };
    },
    kick() {
      schedule();
    },
  };
}
