// Which plot is selected. Exactly one plot, or none (-1). Pure: the page wires these to pointer, key and list events.
// Related surfaces (the PID list, the phone sheet and chips, the header controls, the plot tools, the overview and its
// buttons, the stats table, dialogs and popovers) act on the selection and do not clear it.

/** Hint repeat gap: a tick tapped with no plot selected announces the hint at most this often. */
export const HINT_GAP_MS = 3000;

/** Selects plot i, if it exists. Anything else leaves the selection as it was. */
export function selectPlot(current: number, i: number, count: number): number {
  return Number.isInteger(i) && i >= 0 && i < count ? i : current;
}

/** A pointer went down outside the plots and the related surfaces: the selection clears; a related target keeps it. */
export function deselectOnPointer(current: number, related: boolean): number {
  return related ? current : -1;
}

/** Esc clears the selection only when focus is in a plot. */
export function deselectOnEscape(current: number, focusInPlot: boolean): number {
  return focusInPlot ? -1 : current;
}

/** Ticks change only when a plot is selected. */
export function canToggleTicks(active: number): boolean {
  return active >= 0;
}

/** The PIDs the list shows as ticked: the selected plot's, or none. Other plots keep theirs. */
export function chartPidsOf(perPlot: readonly (readonly number[])[], active: number): number[] {
  return active >= 0 && active < perPlot.length ? [...perPlot[active]] : [];
}

/** Whether the no-plot hint may show now. */
export function hintDue(now: number, last: number | undefined, gap = HINT_GAP_MS): boolean {
  return last === undefined || now - last >= gap;
}
