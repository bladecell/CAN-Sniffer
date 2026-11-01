import { alertStore } from "$lib/alertStore.svelte";
import { hintDue } from "./selection";

let last: number | undefined;

/** Tells the user a tick needs a selected plot. Rate limited so a run of taps does not stack toasts. */
export function noPlotHint(now = Date.now()) {
  if (!hintDue(now, last)) return;
  last = now;
  alertStore.add("Select a plot first, then pick its PIDs", "info");
}
