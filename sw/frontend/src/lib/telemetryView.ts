import { filterRows, sortRows, type FilterRule } from "$lib/tableRows";

/** Row fields the user can show or hide in the PID list (View menu). */
export type PidField =
  | "mode"
  | "interval"
  | "supported"
  | "valid"
  | "pending"
  | "loaded"
  | "age"
  | "length"
  | "formula"
  | "priority"
  | "range";

export const PID_FIELDS: [PidField, string][] = [
  ["mode", "Mode"],
  ["interval", "Update interval"],
  ["supported", "Supported"],
  ["valid", "Valid"],
  ["pending", "Pending sync"],
  ["loaded", "Loaded"],
  ["age", "Value age"],
  ["length", "Length"],
  ["formula", "Formula"],
  ["priority", "Priority"],
  ["range", "Min and max"],
];

export const DEFAULT_PID_FIELDS: ReadonlySet<PidField> = new Set<PidField>([
  "mode",
  "interval",
  "supported",
  "valid",
  "pending",
]);

export interface PidStatusInput {
  loaded: boolean;
  supported: boolean;
  valid: boolean;
  pending: boolean;
}

export interface PidChip {
  kind: "ok" | "warn" | "crit" | "plain";
  label: string;
  icon?: "check" | "warn" | "clock";
}

/** Status chips for one PID row. Not-loaded is always shown; the other states follow the visible fields. */
export function pidStatusChips(s: PidStatusInput, fields: ReadonlySet<PidField>): PidChip[] {
  const chips: PidChip[] = [];
  if (fields.has("supported")) {
    chips.push(
      s.supported
        ? { kind: "ok", label: "Supported", icon: "check" }
        : { kind: "warn", label: "Not supported", icon: "warn" },
    );
  }
  if (fields.has("valid")) {
    chips.push(
      s.valid ? { kind: "ok", label: "Valid", icon: "check" } : { kind: "crit", label: "Invalid", icon: "warn" },
    );
  }
  if (fields.has("pending") && s.pending) {
    chips.push({ kind: "warn", label: "Pending sync", icon: "clock" });
  }
  if (!s.loaded) chips.push({ kind: "plain", label: "Not loaded" });
  return chips;
}

/** "seen N s ago" from the frontend receipt time. Null means no frame has arrived since the page loaded. */
export function seenLabel(ageMs: number | null): string {
  if (ageMs === null) return "never seen";
  const s = Math.max(0, ageMs) / 1000;
  if (s < 60) return `seen ${s.toFixed(1)} s ago`;
  return `seen ${Math.floor(s / 60)} min ago`;
}

export const PID_SORT_KEYS: [string, string][] = [
  ["name", "Name"],
  ["pid", "PID"],
  ["mode", "Mode"],
  ["value", "Value"],
  ["interval", "Update interval"],
  ["priority", "Priority"],
  ["supported", "Supported"],
  ["valid", "Valid"],
];

const NUMERIC_KEYS = new Set(["pid", "value", "interval", "priority"]);

export interface PidViewItem<T> {
  row: T;
  cells: Record<string, string | number | null>;
}

/** Filters, then sorts, view items by their cells. Returns the original row objects. */
export function applyPidView<T>(
  items: PidViewItem<T>[],
  rules: FilterRule[],
  sort: { key: string; dir: 1 | -1 } | null,
): T[] {
  let view: Record<string, unknown>[] = items.map((i) => ({ ...i.cells, __row: i.row }));
  view = filterRows(view, rules) as Record<string, unknown>[];
  if (sort) {
    view = sortRows(view, sort.key, sort.dir === 1 ? "asc" : "desc", NUMERIC_KEYS.has(sort.key));
  }
  return view.map((v) => v.__row as T);
}

/** PIDs the active chart shows: the ticked PIDs, or the active row when nothing is ticked. */
export function chartPidsFor(tickedPids: number[], activeRowPid: number | null): number[] {
  if (tickedPids.length > 0) return tickedPids;
  return activeRowPid === null ? [] : [activeRowPid];
}

/**
 * What a status chip press does. A pressed chip that is not ticked is only on the chart
 * because its row is active, so the active row is cleared.
 */
export function chipAction(pressed: boolean, ticked: boolean): "tick" | "untick" | "clearActive" {
  if (!pressed) return "tick";
  return ticked ? "untick" : "clearActive";
}
