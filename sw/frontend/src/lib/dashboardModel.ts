// Dashboards document: dashboards -> sections (6 columns) -> cards. Pure data and validation, no I/O.
// Sizes are columns x rows (56 px rows); ranges come from docs/decisions.md (2026-11-02).
import { templateById, type DashboardTemplate } from "./dashboardTemplates";

export const CAPS = { dashboards: 8, sectionsPerDashboard: 12, cardsPerSection: 24, cardsTotal: 150 } as const;
export const COLUMNS = 6;
export const MAX_DOC_CHARS = 256 * 1024;
const NAME_MAX = 40;
const TITLE_MAX = 60;
const OVERVIEW_MAX_ITEMS = 6;
const LABEL_MAX = 24;
/** Largest magnitude a card range may hold (PID values are 32-bit floats; this is a sanity cap). */
const RANGE_LIMIT = 1e7;

export const CARD_STYLES = {
  value: ["big", "spark", "minmax"],
  bar: ["plain", "ticks", "zones"],
  gauge: ["arc", "dial", "ring"],
  chart: ["line", "area", "lanes"],
  battery: ["glyph", "bar"],
  overview: ["grid", "list", "hero"],
  dtc: ["rows", "count"],
  status: ["chips", "list"],
  polling: ["switch", "button"],
  recorder: ["switch", "button"],
} as const;

export type CardType = keyof typeof CARD_STYLES;
export const CARD_TYPES = Object.keys(CARD_STYLES) as CardType[];

export interface Thresholds { warn: number; critical: number }

/** One overview row: a PID value, the VIN (once) or the DTC count (once). The label replaces the PID name. */
export type OverviewItem = { t: "pid"; pid: number; label?: string } | { t: "vin"; label?: string } | { t: "dtc"; label?: string };
const hasVin = (c: { items?: OverviewItem[] }) => (c.items ?? []).some((i) => i.t === "vin");

export interface Card {
  id: string;
  type: CardType;
  w: number;
  h: number;
  /** PIDs shown: 1 for value and bar, 1-3 gauge, 1-4 chart, up to 6 overview items, none otherwise. */
  pids: number[];
  style: string;
  thresholds?: Thresholds;
  /** Overview only: the items in display order, at most six. Replaces the old PIDs plus VIN and DTC flags. */
  items?: OverviewItem[];
  /** Gauge only: the scale for this card, instead of the PID's range. Optional; old cards have none. */
  range?: { min: number; max: number };
}

export interface Section { id: string; title: string; cards: Card[] }
export interface Dashboard { id: string; name: string; sections: Section[] }
export interface DashboardsDoc { version: 1; updatedAt: number; dashboards: Dashboard[] }

type Size = [number, number];
export interface SizeLimits { min: Size; max: Size }

const BASE: Record<CardType, { min: Size; def: Size; max: Size }> = {
  value: { min: [2, 1], def: [3, 2], max: [6, 3] },
  bar: { min: [2, 1], def: [3, 1], max: [6, 2] },
  gauge: { min: [2, 2], def: [3, 3], max: [6, 4] },
  chart: { min: [2, 2], def: [6, 3], max: [6, 4] },
  battery: { min: [2, 1], def: [3, 2], max: [6, 3] },
  overview: { min: [3, 2], def: [6, 2], max: [6, 4] },
  dtc: { min: [3, 3], def: [6, 3], max: [6, 6] },
  status: { min: [4, 3], def: [6, 3], max: [6, 4] },
  polling: { min: [3, 2], def: [3, 2], max: [6, 3] },
  recorder: { min: [3, 2], def: [3, 2], max: [6, 3] },
};

const GAUGE_MIN_COLS = [2, 2, 3, 4];
const CHART_MIN_COLS = [2, 2, 3, 4, 6];

export function sizeLimits(card: Pick<Card, "type" | "style" | "pids" | "items">): SizeLimits {
  const b = BASE[card.type];
  const min: Size = [...b.min];
  const max: Size = [...b.max];
  const n = card.pids.length;
  if (card.type === "gauge" && n > 1) { min[0] = GAUGE_MIN_COLS[Math.min(n, 3)]; min[1] = 3; }
  if (card.type === "chart") {
    min[0] = CHART_MIN_COLS[Math.min(n, 4)] ?? 2;
    if (card.style === "lanes") min[1] = Math.max(3, n);
  }
  if (card.type === "battery" && card.style === "glyph") min[0] = 3;
  if (card.type === "overview" && hasVin(card) && card.style === "list") min[0] = 4;
  if (card.type === "dtc" && card.style === "count") { min[1] = 2; max[1] = 2; }
  if (card.type === "status" && card.style === "list") max[1] = 5;
  return { min, max };
}

export function defaultSize(card: Pick<Card, "type" | "style" | "pids" | "items">): { w: number; h: number } {
  const [w, h] = BASE[card.type].def;
  if (card.type === "gauge" && card.pids.length > 1) return { w: 4, h: 3 };
  if (card.type === "dtc" && card.style === "count") return { w: 6, h: 2 };
  if (card.type === "status" && card.style === "list") return { w: 4, h: 5 };
  return clampSize(card, w, h);
}

const isNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

/** Snaps to whole columns and rows inside the card's range. Non-numbers take the type default. */
export function clampSize(card: Pick<Card, "type" | "style" | "pids" | "items">, w: unknown, h: unknown): { w: number; h: number } {
  const { min, max } = sizeLimits(card);
  const [dw, dh] = BASE[card.type].def;
  let cw = clamp(Math.round(isNum(w) ? w : dw), min[0], max[0]);
  let ch = clamp(Math.round(isNum(h) ? h : dh), min[1], max[1]);
  // Overview hero with a VIN fits at 5x2 or 3x4 only.
  if (card.type === "overview" && card.style === "hero" && hasVin(card) && cw < 5 && ch < 4) ch = 4;
  cw = clamp(cw, min[0], max[0]);
  ch = clamp(ch, min[1], max[1]);
  return { w: cw, h: ch };
}

let idCounter = 0;
export function newId(): string {
  idCounter += 1;
  return `${Date.now().toString(36)}${Math.random().toString(36).slice(2, 8)}${idCounter.toString(36)}`;
}

function pidLimit(type: CardType): [number, number] {
  switch (type) {
    case "value": case "bar": return [1, 1];
    case "gauge": return [1, 3];
    case "chart": return [1, 4];
    case "overview": return [0, OVERVIEW_MAX_ITEMS];
    default: return [0, 0];
  }
}

const isRecord = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);
const text = (v: unknown, max: number) => (typeof v === "string" ? v.trim().slice(0, max) : "");

/** Takes a card from untrusted data, or null if it cannot be one. Own-field rebuild: unknown keys are dropped. */
export function normalizeCard(raw: unknown, takenIds: Set<string>): Card | null {
  if (!isRecord(raw)) return null;
  const type = raw.type;
  if (typeof type !== "string" || !(type in CARD_STYLES)) return null;
  const t = type as CardType;
  const styles: readonly string[] = CARD_STYLES[t];
  const style = typeof raw.style === "string" && styles.includes(raw.style) ? raw.style : styles[0];
  const [lo, hi] = pidLimit(t);
  const rawPids = Array.isArray(raw.pids) ? raw.pids : [];
  if (!rawPids.every((p) => Number.isInteger(p) && (p as number) >= 0)) return null;
  let pids = (rawPids as number[]).slice(0, hi);
  if (t === "gauge" && style === "dial") pids = pids.slice(0, 2);
  const card: Card = { id: "", type: t, w: 0, h: 0, pids, style };
  if (t === "overview") {
    card.items = parseItems(raw, pids);
    card.pids = [];
  }
  if (card.pids.length < lo) return null;
  if (t === "gauge" && isRecord(raw.range) && isNum(raw.range.min) && isNum(raw.range.max)) {
    const { min, max } = raw.range;
    if (min < max && Math.abs(min) <= RANGE_LIMIT && Math.abs(max) <= RANGE_LIMIT) card.range = { min, max };
  }
  if (isRecord(raw.thresholds) && isNum(raw.thresholds.warn) && isNum(raw.thresholds.critical) && raw.thresholds.warn !== raw.thresholds.critical) {
    card.thresholds = { warn: raw.thresholds.warn, critical: raw.thresholds.critical };
  }
  Object.assign(card, clampSize(card, raw.w, raw.h));
  card.id = freshId(raw.id, takenIds);
  return card;
}

/** Overview items from a card. Cards saved before the item list have PIDs and VIN and DTC flags: those become items in that order. */
function parseItems(raw: Record<string, unknown>, legacyPids: number[]): OverviewItem[] {
  const src: unknown[] = Array.isArray(raw.items)
    ? raw.items
    : [...legacyPids.map((pid) => ({ t: "pid", pid })), ...(raw.showVin === true ? [{ t: "vin" }] : []), ...(raw.showDtc === true ? [{ t: "dtc" }] : [])];
  const out: OverviewItem[] = [];
  let vin = false;
  let dtc = false;
  const pids = new Set<number>();
  for (const r of src) {
    if (out.length >= OVERVIEW_MAX_ITEMS) break;
    if (!isRecord(r)) continue;
    const label = text(r.label, LABEL_MAX);
    const extra = label ? { label } : {};
    if (r.t === "vin" && !vin) {
      vin = true;
      out.push({ t: "vin", ...extra });
    } else if (r.t === "dtc" && !dtc) {
      dtc = true;
      out.push({ t: "dtc", ...extra });
    } else if (r.t === "pid" && Number.isInteger(r.pid) && (r.pid as number) >= 0 && !pids.has(r.pid as number)) {
      pids.add(r.pid as number);
      out.push({ t: "pid", pid: r.pid as number, ...extra });
    }
  }
  return out;
}

function freshId(raw: unknown, taken: Set<string>): string {
  let id = typeof raw === "string" && raw.length > 0 && raw.length <= 64 ? raw : newId();
  while (taken.has(id)) id = newId();
  taken.add(id);
  return id;
}

/** Normalises untrusted data (JSON text or an object) into a document. Never throws; null when unusable. */
export function parseDashboards(input: unknown): DashboardsDoc | null {
  try {
    let raw = input;
    if (typeof raw === "string") {
      if (raw.length === 0 || raw.length > MAX_DOC_CHARS) return null;
      raw = JSON.parse(raw);
    }
    if (!isRecord(raw) || !Array.isArray(raw.dashboards)) return null;
    const ids = new Set<string>();
    let cardsLeft: number = CAPS.cardsTotal;
    const dashboards: Dashboard[] = [];
    for (const rd of raw.dashboards.slice(0, CAPS.dashboards)) {
      if (!isRecord(rd)) continue;
      const sections: Section[] = [];
      const rawSections = Array.isArray(rd.sections) ? rd.sections : [];
      for (const rs of rawSections.slice(0, CAPS.sectionsPerDashboard)) {
        if (!isRecord(rs)) continue;
        const cards: Card[] = [];
        const rawCards = Array.isArray(rs.cards) ? rs.cards : [];
        for (const rc of rawCards.slice(0, CAPS.cardsPerSection)) {
          if (cardsLeft <= 0) break;
          const card = normalizeCard(rc, ids);
          if (!card) continue;
          cards.push(card);
          cardsLeft -= 1;
        }
        sections.push({ id: freshId(rs.id, ids), title: text(rs.title, TITLE_MAX), cards });
      }
      dashboards.push({ id: freshId(rd.id, ids), name: text(rd.name, NAME_MAX) || "Dashboard", sections });
    }
    if (dashboards.length === 0) return null;
    return { version: 1, updatedAt: isNum(raw.updatedAt) && raw.updatedAt >= 0 ? raw.updatedAt : 0, dashboards };
  } catch {
    return null;
  }
}

/** Fresh sections and card ids from a template. Cards go through normalizeCard, so a bad template card is dropped, not stored. */
export function sectionsFromTemplate(t: DashboardTemplate): Section[] {
  return t.sections.map((s) => ({
    id: newId(),
    title: s.title,
    cards: s.cards.flatMap((c) => normalizeCard(c, new Set()) ?? []),
  }));
}

/** Fallback when nothing usable is stored: the Driving template on one dashboard. */
export function defaultDashboards(now: number): DashboardsDoc {
  const driving = templateById("driving");
  return { version: 1, updatedAt: now, dashboards: [{ id: newId(), name: driving.name, sections: sectionsFromTemplate(driving) }] };
}
