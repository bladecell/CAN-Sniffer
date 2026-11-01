/** What the PID list needs to know about one row to place it in a group. */
export interface PidGroupInput {
  /** The device reports the PID as supported. */
  supported: boolean;
  /** A numeric value has arrived. N/A is not a value. */
  hasValue: boolean;
  mode: number;
  modeLabel: string;
}

export interface PidSection<T> {
  key: string;
  title: string;
  items: T[];
}

export interface PidGroup<T> {
  key: string;
  title: string;
  /** Every item in the group. */
  items: T[];
  /** Only the available group: one section per mode, ascending. */
  sections?: PidSection<T>[];
}

const DERIVED_MODE = 0x45;

/**
 * Groups loaded PID rows by state, in display order: available (sections per mode), supported
 * without data, not supported. Empty groups are left out. Item order inside a group is the input order.
 */
export function groupPids<T>(items: T[], describe: (item: T) => PidGroupInput): PidGroup<T>[] {
  const available: T[] = [];
  const noData: T[] = [];
  const unsupported: T[] = [];
  const modes = new Map<number, PidSection<T>>();

  for (const item of items) {
    const s = describe(item);
    if (!s.supported) unsupported.push(item);
    else if (!s.hasValue) noData.push(item);
    else {
      available.push(item);
      let section = modes.get(s.mode);
      if (!section) {
        section = { key: `mode:${s.mode}`, title: s.modeLabel, items: [] };
        modes.set(s.mode, section);
      }
      section.items.push(item);
    }
  }

  const sections = [...modes.entries()].sort(([a], [b]) => a - b).map(([, section]) => section);
  const groups: PidGroup<T>[] = [
    { key: "available", title: "Available by mode", items: available, sections },
    { key: "nodata", title: "Supported, no data", items: noData },
    { key: "unsupported", title: "Not supported", items: unsupported },
  ];
  return groups.filter((g) => g.items.length > 0);
}

/** A PID as text: 0x, two hex digits when it fits one byte (0x05), otherwise four (0x0105). Uppercase. */
export function formatPid(pid: number): string {
  return "0x" + pid.toString(16).toUpperCase().padStart(pid > 0xff ? 4 : 2, "0");
}

/** The PID as shown in the Telemetry lists. Derived PIDs have no PID of their own, so nothing. */
export function pidHex(pid: number, mode: number): string {
  if (mode === DERIVED_MODE) return "";
  return formatPid(pid);
}

export interface RemovableRow {
  loaded: boolean;
  selected: boolean;
}

/**
 * Removes the rows `matches` picks. A row the device has is kept but not loaded, so Changes lists it
 * as removed until Apply. A row the device never had is dropped outright. Removed rows are unselected.
 */
export function removeRows<T extends RemovableRow>(rows: T[], matches: (row: T) => boolean, onDevice: (row: T) => boolean): T[] {
  return rows.filter((row) => {
    if (!matches(row)) return true;
    row.selected = false;
    if (!onDevice(row)) return false;
    row.loaded = false;
    return true;
  });
}

export type PendingKind = "added" | "modified" | "removed";

/** How a row with pending changes differs from the device. Callers pass only rows that have changes. */
export function pendingKind(loaded: boolean, onDevice: boolean): PendingKind {
  if (!loaded) return "removed";
  return onDevice ? "modified" : "added";
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

export type FooterAction = "new" | "edit" | "remove";

/** The bottom bar: New PID with no selection, Edit and Remove for one selected PID, Remove N for several. */
export function footerActions(selectedCount: number): FooterAction[] {
  if (selectedCount === 0) return ["new"];
  if (selectedCount === 1) return ["edit", "remove"];
  return ["remove"];
}

/** One row as the apply step sees it. `deviceDef` is the device's copy of the PID, if it has one. */
export interface ApplyItem<D> {
  pid: number;
  loaded: boolean;
  pending: boolean;
  def: D;
  deviceDef?: D;
}

/**
 * The definition set to write when the `chosen` PIDs are applied. A chosen row goes as the draft it holds (dropped
 * when it is removed). An unchosen pending row keeps the device's copy, so only the chosen changes reach the device.
 */
export function definitionsToApply<D>(items: ApplyItem<D>[], chosen: ReadonlySet<number>): D[] {
  const out: D[] = [];
  for (const item of items) {
    if (!item.pending || chosen.has(item.pid)) {
      if (item.loaded) out.push(item.def);
    } else if (item.deviceDef !== undefined) {
      out.push(item.deviceDef);
    }
  }
  return out;
}
