// Layout rules for the dashboard cards that list items: how many fit a card, the overview fault chip and the battery bands.
// Pure, no DOM, so it runs under bun test. Row unit is 56 px (docs/decisions.md 2026-11-02).
import { BATTERY_SCALE_MAX_V, BATTERY_SCALE_MIN_V } from "./carBattery";

export interface Visible { shown: number; more: number }

const ROW_PX = 56;

/** Items that fit: when some do not, the last visible cell becomes the "+N more" line. */
function fit(count: number, cap: number): Visible {
  if (count <= cap) return { shown: count, more: 0 };
  const shown = Math.max(0, cap - 1);
  return { shown, more: count - shown };
}

/** Status items: chip grid is two columns (three from 6 wide) by the card's rows; list is one row per item. */
export function statusVisible(count: number, w: number, h: number, style: string): Visible {
  const cap = style === "list" ? Math.floor((h * ROW_PX - 40) / 28) : (w >= 6 ? 3 : 2) * h;
  return fit(count, Math.max(1, cap));
}

/** DTC rows at 26 px each, after the title and the group line. */
export function dtcVisible(count: number, h: number): Visible {
  return fit(count, Math.max(1, Math.floor((h * ROW_PX - 70) / 26)));
}

export type ChipTone = "ok" | "warn" | "muted";

/** The overview header chip: the fault count, or No data before the first read. */
export function faultChip(faults: number | null): { text: string; tone: ChipTone } {
  if (faults === null) return { text: "No data", tone: "muted" };
  if (faults === 0) return { text: "No issues", tone: "ok" };
  return { text: `${faults} ${faults === 1 ? "fault" : "faults"}`, tone: "warn" };
}

export interface BatteryBand { from: number; to: number; tone: "crit" | "warn" | "ok" }

/** The five voltage bands on the 9.5 to 15.5 V display scale, as fractions 0 to 1 (carBattery.ts states). */
export function batteryBands(): BatteryBand[] {
  const span = BATTERY_SCALE_MAX_V - BATTERY_SCALE_MIN_V;
  const at = (v: number) => (v - BATTERY_SCALE_MIN_V) / span;
  const cuts: [number, BatteryBand["tone"]][] = [[10.5, "crit"], [11.8, "warn"], [13.2, "ok"], [15, "ok"], [BATTERY_SCALE_MAX_V, "warn"]];
  let from = 0;
  return cuts.map(([v, tone]) => {
    const band = { from, to: v === BATTERY_SCALE_MAX_V ? 1 : at(v), tone };
    from = band.to;
    return band;
  });
}
