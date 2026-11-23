// Reactive scroll state for a chart. The pure rules are in scroll.ts.
import { scrollFrame } from "./scroll";

/** Reactive scroll state of one chart. The chart reads anchor to build and offset per frame, and calls frame(). */
export class ChartScroll {
  /** Device ms the geometry is built for. */
  anchor = $state(0);
  /** Device ms of the present at the last frame. */
  present = $state(0);
  #presentAt: (time: number) => number | null;
  #holdEnds: () => readonly number[];

  constructor(presentAt: (time: number) => number | null, holdEnds: () => readonly number[]) {
    this.#presentAt = presentAt;
    this.#holdEnds = holdEnds;
  }

  /** Runs once per frame. Rebuilds the anchor when the geometry no longer covers the present. */
  frame(time: number) {
    const now = this.#presentAt(time);
    if (now === null) return;
    const next = scrollFrame(this.anchor, now, this.#holdEnds());
    this.anchor = next.anchor;
    this.present = next.present;
  }

  /** Shift in the chart's units (px or viewBox units) for a per-ms scale. Positive moves the layer right. */
  shift(unitsPerMs: number): number {
    return (this.anchor - this.present) * unitsPerMs;
  }
}
