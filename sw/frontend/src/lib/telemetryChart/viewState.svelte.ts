// The one time window every plot shares, the present clock and the cursor. Components reach it through context.
import { clamp } from "./math";
import { createPresentClock, guardWin, liveWin, winStatus, type Win, type WinStatus } from "./window";

export const TELEMETRY_VIEW = "telemetry-view";
/** The present advances at 4 Hz while live; the plots redraw from `tick`. */
const CLOCK_MS = 250;

export class TelemetryView {
  win = $state<Win>(liveWin(60));
  /** Bumped while live so every reader of presentAt() refreshes. */
  tick = $state(0);
  /** Cursor position as a fraction of a plot's width, shared by all plots. */
  hoverF = $state<number | null>(null);
  /** Which plot the pointer is over, so only that plot shows the tooltip. */
  hoverKey = $state<string | null>(null);
  #clock = createPresentClock();

  get status(): WinStatus {
    return winStatus(this.win);
  }

  /** Device ms of the present: frozen while paused, extrapolated from the newest sample while live. Null before data. */
  presentAt(): number | null {
    void this.tick;
    if (this.win.pausedAt !== null) return this.win.pausedAt;
    return this.#clock.now(performance.now());
  }

  /** Feeds the newest sample time of any plotted PID. */
  observe(deviceMs: number) {
    this.#clock.observe(deviceMs, performance.now());
  }

  /** Runs the clock while mounted. Returns the stop function for $effect. */
  start(): () => void {
    const id = setInterval(() => {
      if (this.win.pausedAt === null) this.tick++;
    }, CLOCK_MS);
    return () => clearInterval(id);
  }

  /** Applies a window change. Without data there is no present, so the window cannot leave live. */
  apply(next: (w: Win, now: number) => Win) {
    const now = this.presentAt();
    this.win = guardWin(next(this.win, now ?? 0), now);
  }

  /** Replaces the window, with the same no-data guard as apply(). */
  set(w: Win) {
    this.win = guardWin(w, this.presentAt());
  }

  reset() {
    this.win = liveWin(60);
  }

  hover(f: number | null, key: string | null) {
    this.hoverF = f === null ? null : clamp(f, 0, 1);
    this.hoverKey = f === null ? null : key;
  }
}
