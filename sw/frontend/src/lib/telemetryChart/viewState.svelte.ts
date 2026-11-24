// The one time window every plot shares, the present clock and the cursor. Components reach it through context.
import { clamp } from "./math";
import { holdPause, seekWin } from "./playback";
import { createPresentClock, guardWin, liveWin, winStatus, type Win, type WinStatus } from "./window";

export const TELEMETRY_VIEW = "telemetry-view";

export class TelemetryView {
  win = $state<Win>(liveWin(60));
  /** Cursor position as a fraction of a plot's width, shared by all plots. */
  hoverF = $state<number | null>(null);
  /** Which plot the pointer is over, so only that plot shows the tooltip. */
  hoverKey = $state<string | null>(null);
  #clock = createPresentClock();

  /** A recorded trip: the player alone starts playback, a window move on a paused trip keeps it paused. */
  constructor(readonly recorded = false) {}

  #commit(next: Win) {
    this.win = this.recorded ? holdPause(this.win, next) : next;
  }

  get status(): WinStatus {
    return winStatus(this.win);
  }

  /** Device ms of the present at a frame time (rAF timestamp): frozen while paused, extrapolated while live. Null before data. */
  presentAt(at = performance.now()): number | null {
    if (this.win.pausedAt !== null) return this.win.pausedAt;
    return this.#clock.now(at);
  }

  /** Feeds the newest sample time of any plotted PID. */
  observe(deviceMs: number) {
    this.#clock.observe(deviceMs, performance.now());
  }

  /** Applies a window change. Without data there is no present, so the window cannot leave live. */
  apply(next: (w: Win, now: number) => Win) {
    const now = this.presentAt();
    this.#commit(guardWin(next(this.win, now ?? 0), now));
  }

  /** Replaces the window, with the same no-data guard as apply(). */
  set(w: Win) {
    this.#commit(guardWin(w, this.presentAt()));
  }

  /** Moves the present to a recorded time: a fresh clock, so the present can also go back. */
  seek(deviceMs: number, playing: boolean) {
    this.#clock = createPresentClock();
    this.#clock.observe(deviceMs, performance.now());
    this.win = seekWin(this.win.len, deviceMs, playing);
  }

  reset() {
    this.#commit(liveWin(60));
  }

  hover(f: number | null, key: string | null) {
    this.hoverF = f === null ? null : clamp(f, 0, 1);
    this.hoverKey = f === null ? null : key;
  }
}
