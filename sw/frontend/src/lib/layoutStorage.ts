import type { DashboardItem } from "./types";
import { safeHexColor } from "./color";

const CARD_TYPES = ["pid", "battery", "dtcs", "overview", "control"];
const PID_MODES = ["card", "chart", "gauge", "bar"];
const SPECIAL_MODES = ["default", "compact", "detailed"];
const MAX_ITEMS = 200;

const isPosNum = (v: unknown) => typeof v === "number" && Number.isFinite(v) && v > 0;
const isNum = (v: unknown) => typeof v === "number" && Number.isFinite(v);

function isValidItem(item: any): item is DashboardItem {
  if (!item || typeof item !== "object") return false;
  if (typeof item.id !== "string" || !item.id) return false;
  if (!CARD_TYPES.includes(item.cardType)) return false;
  if (!isPosNum(item.w) || !isPosNum(item.h)) return false;
  if (item.cardType === "pid") return isNum(item.pid) && PID_MODES.includes(item.displayMode);
  if (!SPECIAL_MODES.includes(item.displayMode)) return false;
  if (item.cardType === "overview") {
    return Array.isArray(item.pids) && item.pids.length === 3 && item.pids.every(isNum);
  }
  return true;
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
    out.push(item);
  }
  return out;
}
