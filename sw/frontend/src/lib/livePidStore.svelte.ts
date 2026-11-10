import { SvelteMap } from "svelte/reactivity";
import type { PidValue } from "./types";

/** Current in-memory values only; cardinality is bounded by firmware's 256-definition limit. */
export const livePids = new SvelteMap<number, PidValue>();

// Not reactive: the Polling card reads the difference once a second (PIDs/s), so a frame costs no re-render.
let updates = 0;

/** Counts one PID update from the WebSocket. */
export function countPidUpdate(): void {
  updates++;
}

/** Updates received since the page loaded. */
export function pidUpdateCount(): number {
  return updates;
}
