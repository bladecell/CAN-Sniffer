// Pure rules for the add and edit card sheet: PID counts, PID ranges, default and checked thresholds.
import type { CardType, Thresholds } from "./dashboardModel";

export interface PidRange { min: number; max: number }

export const TYPE_LABELS: Record<CardType, string> = {
  value: "PID value",
  bar: "Bar",
  gauge: "Gauge",
  chart: "Chart",
  battery: "Battery",
  overview: "Overview",
  dtc: "DTC list",
  status: "Status",
  polling: "Polling",
  recorder: "Recorder",
};

export const STYLE_LABELS: Record<string, string> = {
  big: "Big number",
  spark: "Number and sparkline",
  minmax: "Number and min and max",
  plain: "Plain",
  ticks: "Ticks and range",
  zones: "Threshold zones",
  arc: "Arc",
  dial: "Needle dial",
  ring: "Ring",
  line: "Line",
  area: "Area",
  lanes: "Lanes",
  glyph: "Glyph",
  bar: "Bar",
  grid: "Grid",
  list: "List",
  hero: "Hero",
  rows: "Plain rows",
  count: "Count only",
  chips: "Chip grid",
  switch: "Switch",
  button: "Button and state",
};

/** The PIDs a card takes: [minimum, maximum]. Overview items share the cap with the VIN and DTC count (`extras`). */
export function pidSlots(type: CardType, style: string, extras: number): [number, number] {
  switch (type) {
    case "value":
    case "bar":
      return [1, 1];
    case "gauge":
      return style === "dial" ? [1, 2] : [1, 3];
    case "chart":
      return [1, 4];
    case "overview":
      return [0, Math.max(0, 6 - extras)];
    default:
      return [0, 0];
  }
}

/** A PID's range, or null when it has none (unset or empty). */
export function rangeOf(def: { minValue: number; maxValue: number }): PidRange | null {
  const { minValue: min, maxValue: max } = def;
  if (!Number.isFinite(min) || !Number.isFinite(max) || min >= max) return null;
  return { min, max };
}

const round2 = (v: number) => Math.round(v * 100) / 100;

/** Warn and critical at 80% and 90% of the range; 20% and 10% when low values are the problem (warn above critical). */
export function defaultThresholds(range: PidRange | null, lowWarns = false): Thresholds | null {
  if (!range) return null;
  const span = range.max - range.min;
  const [warnAt, critAt] = lowWarns ? [0.2, 0.1] : [0.8, 0.9];
  return { warn: round2(range.min + span * warnAt), critical: round2(range.min + span * critAt) };
}

/** Null when the pair can be saved. Without a PID range only the numbers are checked. */
export function thresholdsError(range: PidRange | null, t: Thresholds): string | null {
  if (!Number.isFinite(t.warn) || !Number.isFinite(t.critical)) return "Enter a number for both thresholds.";
  if (t.warn === t.critical) return "The warning and critical values must be different.";
  if (range && [t.warn, t.critical].some((v) => v < range.min || v > range.max)) return "Thresholds must be inside the PID range.";
  return null;
}

/** Thresholds are stored for single-PID value, bar and gauge cards. */
export function thresholdsApply(type: CardType, pidCount: number): boolean {
  return (type === "value" || type === "bar" || type === "gauge") && pidCount === 1;
}

/** Only the zones bar style needs a PID range; the others simply hide range-based parts. */
export function needsRange(type: CardType, style: string): boolean {
  return type === "bar" && style === "zones";
}

/** The name an edit-mode card shows: its PID names, or the card type when it has no PID. */
export function editFaceName(card: { type: CardType; pids: number[] }, pidName: (pid: number) => string | undefined): string {
  if (card.pids.length === 0) return TYPE_LABELS[card.type];
  return card.pids.map((pid) => pidName(pid) ?? `PID ${pid}`).join(" and ");
}
