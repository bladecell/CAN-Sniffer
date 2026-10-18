import { connectionStore } from "./connectionStore.svelte";
// Car battery history, kept outside connectionStore. Samples connectionStore.wsCanStatus once per 0x03 frame (1 Hz)
// into one bounded chartBuffer. The buffer is not a PID, so it has no PID id: it is a single shared buffer.
import { untrack } from "svelte";
import { createChartBuffer, type ChartSnapshot } from "./chartBuffer";
import { createPacer, CHART_REDRAW_MS } from "./chartPacer";
import { chartSampleTime, sanitizeVoltage } from "./carBattery";

/** Chart window, the same 60 s as the PID chart. */
export const BATTERY_HISTORY_MS = 60_000;
/** A gap longer than this breaks the line (the frame is 1 Hz). */
const GAP_BREAK_MS = 2500;
const CAN_CONNECTED = "CAN Connected";

class CarBatteryFeed {
  /** Local ms of the last 0x03 frame, 0 before any frame. The frame carries no time of its own. */
  frameAt = $state(0);
  #version = $state(0);
  #buffer = createChartBuffer();
  #pacer = createPacer<"battery">(CHART_REDRAW_MS, () => {
    this.#version++;
  });
  #users = 0;
  #stopSampling: (() => void) | null = null;

  /**
   * Samples while at least one card is mounted. Call from a component effect and return the result
   * as its cleanup: the last release stops the sampling effect.
   */
  attach(): () => void {
    if (this.#users++ === 0) this.#stopSampling = $effect.root(() => this.#sample());
    return () => {
      if (--this.#users === 0) {
        this.#stopSampling?.();
        this.#stopSampling = null;
      }
    };
  }

  #sample() {
    // The status object is replaced by every 0x03 frame and mutated in place on disconnect. Only the
    // reference is tracked; the fields are read untracked, so an in-place change adds no sample.
    // The first run sees the object that was current at attach, which is not a new frame.
    const initial = untrack(() => connectionStore.wsCanStatus);
    $effect(() => {
      const status = connectionStore.wsCanStatus;
      if (status === initial) return;
      untrack(() => {
        const now = Date.now();
        const at = chartSampleTime();
        // NaN is a gap: no voltage, or the bus is down.
        const volts = status.state === CAN_CONNECTED ? sanitizeVoltage(status.battery_voltage) : null;
        this.frameAt = now;
        this.#buffer.push(at, volts ?? Number.NaN, GAP_BREAK_MS);
        this.#pacer.mark("battery");
      });
    });
  }

  /** Samples of the last 60 s, oldest first. Reading it tracks the 5 Hz redraw. */
  snapshot(): ChartSnapshot {
    void this.#version;
    const last = this.#buffer.lastTime();
    return last === undefined ? { t: [], v: [] } : this.#buffer.snapshot(last - BATTERY_HISTORY_MS);
  }
}

export const carBatteryFeed = new CarBatteryFeed();
