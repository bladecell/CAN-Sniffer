import { SvelteSet } from "svelte/reactivity";
import type { FilterRule } from "$lib/tableRows";
import { DEFAULT_PID_FIELDS, type PidField } from "$lib/telemetryView";

/** PID list view options. Shared by the desktop sidebar and the phone PIDs sheet, so both show the same list. */
export const pidView = $state({
  sortKey: "",
  sortDir: 1 as 1 | -1,
  rules: [] as FilterRule[],
  fields: new SvelteSet<PidField>(DEFAULT_PID_FIELDS),
});
