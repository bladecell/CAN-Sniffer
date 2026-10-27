/** What the PID list needs to know about one row to place it in a group. */
export interface PidGroupInput {
  loaded: boolean;
  supported: boolean;
  /** A numeric value has arrived. N/A is not a value. */
  hasValue: boolean;
  mode: number;
  modeLabel: string;
}

export interface PidGroup<T> {
  key: string;
  title: string;
  items: T[];
}

/**
 * Groups PID rows by state, in display order: not supported, supported without data,
 * available (one group per mode, ascending), then rows that are not on the device.
 * Empty groups are left out. Item order inside a group is the input order.
 */
export function groupPids<T>(items: T[], describe: (item: T) => PidGroupInput): PidGroup<T>[] {
  const unsupported: PidGroup<T> = { key: "unsupported", title: "Not supported", items: [] };
  const noData: PidGroup<T> = { key: "nodata", title: "Supported, no data", items: [] };
  const notOnDevice: PidGroup<T> = { key: "notondevice", title: "Not on device", items: [] };
  const available = new Map<number, PidGroup<T>>();

  for (const item of items) {
    const s = describe(item);
    if (!s.loaded) notOnDevice.items.push(item);
    else if (!s.supported) unsupported.items.push(item);
    else if (!s.hasValue) noData.items.push(item);
    else {
      let group = available.get(s.mode);
      if (!group) {
        group = { key: `available:${s.mode}`, title: `Available · ${s.modeLabel}`, items: [] };
        available.set(s.mode, group);
      }
      group.items.push(item);
    }
  }

  const modes = [...available.entries()].sort(([a], [b]) => a - b).map(([, g]) => g);
  return [unsupported, noData, ...modes, notOnDevice].filter((g) => g.items.length > 0);
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
