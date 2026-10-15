import type { BatteryGridItem, DashboardItem } from "./types";
import { BATTERY_DISPLAY_MODES, BAR_STYLES, GAUGE_STYLES } from "./types";
import { safeHexColor } from "./color";

const CARD_TYPES = ["pid", "battery", "dtcs", "overview", "control"];
const PID_MODES = ["card", "chart", "gauge", "bar"];
const SPECIAL_MODES = ["default", "compact", "detailed"];
const MAX_ITEMS = 200;

const isPosNum = (v: unknown) => typeof v === "number" && Number.isFinite(v) && v > 0;
const isNum = (v: unknown) => typeof v === "number" && Number.isFinite(v);

function isValidItem(raw: unknown): raw is DashboardItem {
  if (!raw || typeof raw !== "object") return false;
  const item = raw as Record<string, unknown>;
  if (typeof item.id !== "string" || !item.id) return false;
  if (!CARD_TYPES.includes(item.cardType as string)) return false;
  if (!isPosNum(item.w) || !isPosNum(item.h)) return false;
  if (item.cardType === "pid") return isNum(item.pid) && PID_MODES.includes(item.displayMode as string);
  // Battery modes are normalised in normalizeBattery, so older placeholder modes still load.
  if (item.cardType === "battery") return true;
  if (!SPECIAL_MODES.includes(item.displayMode as string)) return false;
  if (item.cardType === "overview") {
    return Array.isArray(item.pids) && item.pids.length === 3 && item.pids.every(isNum);
  }
  return true;
}

/** Unknown battery modes become the glyph, unknown styles are dropped. The card itself is kept. */
function normalizeBattery(item: BatteryGridItem): void {
  if (!BATTERY_DISPLAY_MODES.includes(item.displayMode)) item.displayMode = "glyph";
  if (!GAUGE_STYLES.includes(item.gaugeStyle as never)) delete item.gaugeStyle;
  if (!BAR_STYLES.includes(item.barStyle as never)) delete item.barStyle;
}

/** Parses the saved layout; anything unusable is dropped so a bad entry cannot break the dashboard. */
export function parseStoredLayout(raw: string | null): DashboardItem[] {
  if (!raw) return [];
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch {
    return [];
  }
  if (!Array.isArray(parsed)) return [];
  const seen = new Set<string>();
  const out: DashboardItem[] = [];
  for (const item of parsed.slice(0, MAX_ITEMS)) {
    if (!isValidItem(item) || seen.has(item.id)) continue;
    seen.add(item.id);
    if (item.cardType === "overview") item.color = safeHexColor(item.color, "#3b82f6");
    if (item.cardType === "battery") normalizeBattery(item);
    if (item.cardType === "pid" && !BAR_STYLES.includes(item.barStyle as never)) delete item.barStyle;
    out.push(item);
  }
  return out;
}
