import type { DtcData, DtcModeData } from "./types";

export type DtcGroup = "confirmed" | "pending" | "permanent";

export interface DtcRow {
  key: string;
  code: string;
  group: DtcGroup;
  groupLabel: string;
  /** OBD mode the code was read from, as shown to the user ("03", "07", "0A"). */
  mode: string;
  description: string;
  priority: string;
}

export const GROUPS: { group: DtcGroup; label: string; mode: string }[] = [
  { group: "confirmed", label: "Confirmed", mode: "03" },
  { group: "pending", label: "Pending", mode: "07" },
  { group: "permanent", label: "Permanent", mode: "0A" },
];

/** The firmware's text for a code missing from the DTC description file. */
const UNKNOWN_DESCRIPTION = "Description not found";

/** Text shown as the explanation, or "" when the API gave none. */
export function explanation(description: string): string {
  const text = description.trim();
  return text === UNKNOWN_DESCRIPTION ? "" : text;
}

/** Heuristic urgency label from the code family (carried over from the previous Diagnostics view). */
export function priorityFor(code: string): string {
  const c = code.trim().toUpperCase();
  if (c === "P0524" || c === "P0217") return "Stop safely";
  if (c.startsWith("P030")) return "Urgent";
  if (c.startsWith("P07") || c.startsWith("P08")) return "Service trans";
  if (c.startsWith("P01") || c.startsWith("P04") || c.startsWith("P02")) return "Attend soon";
  if (c.startsWith("U")) return "Network";
  if (c.startsWith("B") || c.startsWith("C")) return "Inspect";
  return "Logged";
}

/** Rows in severity order: confirmed, pending, permanent. */
export function dtcRows(data: DtcData): DtcRow[] {
  return GROUPS.flatMap(({ group, label, mode }) => {
    const section: DtcModeData | undefined = data[group];
    return (section?.dtc ?? []).map((entry, i) => ({
      key: `${group}:${i}:${entry.dtc}`,
      code: entry.dtc,
      group,
      groupLabel: label,
      mode,
      description: explanation(entry.description ?? ""),
      priority: priorityFor(entry.dtc),
    }));
  });
}

export interface DtcSummary {
  tone: "ok" | "warn" | "crit";
  text: string;
  rest: string;
}

/** Headline for the status line: the most severe group first, the other non-empty groups in `rest`. */
export function dtcSummary(data: DtcData): DtcSummary {
  const counts = {
    confirmed: data.confirmed?.dtc?.length ?? 0,
    pending: data.pending?.dtc?.length ?? 0,
    permanent: data.permanent?.dtc?.length ?? 0,
  };
  const plural = (n: number, word: string) => `${n} ${word}${n === 1 ? "" : "s"}`;
  const parts = [
    counts.pending > 0 ? `${counts.pending} pending` : "",
    counts.permanent > 0 ? `${counts.permanent} permanent` : "",
  ].filter(Boolean);

  if (counts.confirmed > 0) {
    return { tone: "crit", text: plural(counts.confirmed, "active fault"), rest: parts.join(", ") };
  }
  if (counts.pending > 0) {
    return { tone: "warn", text: plural(counts.pending, "pending fault"), rest: counts.permanent > 0 ? `${counts.permanent} permanent` : "" };
  }
  if (counts.permanent > 0) {
    return { tone: "warn", text: plural(counts.permanent, "permanent fault"), rest: "" };
  }
  return { tone: "ok", text: "Powertrain systems healthy", rest: "" };
}
