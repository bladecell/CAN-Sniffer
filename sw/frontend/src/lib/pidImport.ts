// Classifies the PIDs of a file against the PID set, and picks the ones to import. Pure, so it is unit-tested
// (pidImport.test.ts). Nothing here touches the device or the store.
import type { PidDefinition } from "./types";
import { serializePidDefinition } from "./pidDefinition";

/** new: not in the set. changed: in the set and differs (or not loaded). identical: in the set, same properties. */
export type PidImportStatus = "new" | "changed" | "identical";

export interface PidImportItem {
  def: PidDefinition;
  status: PidImportStatus;
}

/** The part of a PID row the classification reads. */
export interface CurrentPid {
  def: PidDefinition;
  loaded: boolean;
}

const fingerprint = (def: PidDefinition) => JSON.stringify(serializePidDefinition(def));

/** Each file PID in file order, with its status against the current set. Matches by PID number. */
export function classifyPidImport(incoming: PidDefinition[], current: CurrentPid[]): PidImportItem[] {
  const byPid = new Map(current.map((row) => [row.def.pid, row]));
  return incoming.map((def) => {
    const row = byPid.get(def.pid);
    if (!row) return { def, status: "new" };
    const same = row.loaded && fingerprint(row.def) === fingerprint(def);
    return { def, status: same ? "identical" : "changed" };
  });
}

/** The import summary: how many PIDs are added to the set and how many replace one in it. */
export function pidImportCounts(items: PidImportItem[]): { added: number; changed: number } {
  return {
    added: items.filter((item) => item.status === "new").length,
    changed: items.filter((item) => item.status === "changed").length,
  };
}

/** The definitions whose PID is chosen, in file order. */
export function chosenDefinitions(items: PidImportItem[], chosen: ReadonlySet<number>): PidDefinition[] {
  return items.filter((item) => chosen.has(item.def.pid)).map((item) => item.def);
}
