// Layout rules for the dashboard cards that list items: how many fit a card, the overview fault chip and the battery bands.
// Pure, no DOM, so it runs under bun test. Row unit is 56 px (docs/decisions.md 2026-11-02).
import { OVERVIEW_MAX_ITEMS } from "./dashboardModel";
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

/** Fallbacks used before a row has been rendered and measured. */
export const DTC_ROW_PX = 27;
export const DTC_MORE_PX = 24;
export const DTC_GAP_PX = 3;

const pos = (v: number, fallback: number) => (Number.isFinite(v) && v > 0 ? v : fallback);

/**
 * DTC rows that fit the measured list height. `rowPx` is the pitch of a code row (height plus gap), `moreH` the height of
 * the "+N more" line and `gap` the gap between rows, all read from the DOM. When some codes do not fit, the rows shown
 * leave room for the whole "+N more" line, so no row and not the line is cut. Bad measurements fall back to the defaults.
 */
export function dtcVisible(count: number, availPx: number, rowPx = DTC_ROW_PX, moreH = DTC_MORE_PX, gap = DTC_GAP_PX): Visible {
  const pitch = pos(rowPx, DTC_ROW_PX);
  if (count * pitch - pos(gap, DTC_GAP_PX) <= availPx) return { shown: count, more: 0 };
  const shown = Math.min(count, Math.max(0, Math.floor((availPx - pos(moreH, DTC_MORE_PX)) / pitch)));
  return { shown, more: count - shown };
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

export type OverviewKind = "pid" | "vin" | "dtc";
export interface OverviewPlan {
  style: "grid" | "list" | "hero";
  cols: number;
  /** Indices of the items drawn, in order: whole items only, the rest are left out. */
  shown: number[];
  /** Hero only: the headline item, the sub-values and the VIN row (all indices), and the wide split. */
  hero?: { head: number; subs: number[]; vin: number | null; wide: boolean };
}

/**
 * Which overview items a card shows at its size (mockup `ovPlan`, docs/decisions.md 2026-11-02). Row pitch is 56 px plus a
 * 12 px gap. Items that do not fit whole are left out instead of being cut. A hero card with no PID or DTC item is a grid.
 */
export function overviewPlan(style: string, w: number, h: number, kinds: readonly OverviewKind[]): OverviewPlan {
  const first = kinds.findIndex((k) => k === "pid");
  const head = first >= 0 ? first : kinds.findIndex((k) => k === "dtc");
  let s: OverviewPlan["style"] = style === "list" || style === "hero" ? style : "grid";
  if (s === "hero" && head < 0) s = "grid";
  if (s === "hero") {
    const vin = kinds.findIndex((k) => k === "vin");
    const wide = w >= 5;
    const cols = wide ? (h >= 3 ? 2 : 3) : 2;
    const rows = vin >= 0 ? (wide ? (h >= 3 ? 2 : 1) : 2) : h >= 3 ? 3 : wide ? 2 : 1;
    const cap = Math.min(vin >= 0 ? 4 : 5, cols * rows);
    const subs = kinds.map((_, i) => i).filter((i) => i !== head && i !== vin).slice(0, cap);
    return { style: s, cols, shown: [head, ...subs, ...(vin >= 0 ? [vin] : [])], hero: { head, subs, vin: vin >= 0 ? vin : null, wide } };
  }
  const grid = s === "grid";
  const cols = grid ? (w >= 5 ? 3 : 2) : w >= 5 ? 2 : 1;
  const rows = grid ? (h === 2 ? 2 : h === 3 ? 3 : 4) : Math.max(1, Math.floor((h * ROW_PX + (h - 1) * 12 - 46) / 28));
  const cells = cols * rows;
  const shown: number[] = [];
  let used = 0;
  for (let i = 0; i < kinds.length; i++) {
    const span = kinds[i] === "vin" ? (grid ? Math.min(2, cols) : cols) : 1;
    if (shown.length >= OVERVIEW_MAX_ITEMS || used + span > cells) break;
    used += span;
    shown.push(i);
  }
  return { style: s, cols, shown };
}
