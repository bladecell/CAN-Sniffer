import { getContext } from "svelte";
import { vehicleStore } from "$lib/vehicleStore.svelte";
import type { PidDefinition } from "$lib/types";

/** Where the plots read PID names, units, colours and ranges. Replay mode puts the open trip's definitions here. */
export const PID_DEFS = Symbol("telemetry-pid-defs");

/** Call during component init. The returned getter is the live list unless a ContextScope replaced it. */
export function pidDefsGetter(): () => readonly PidDefinition[] {
  return getContext<(() => readonly PidDefinition[]) | undefined>(PID_DEFS) ?? (() => vehicleStore.pidDefinitions);
}
