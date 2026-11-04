import { describe, expect, test } from "bun:test";
import { CAPS, defaultDashboards, parseDashboards, sizeLimits, clampSize, newId, normalizeCard, type Card } from "./dashboardModel";

const card = (over: Partial<Card> = {}): Card => ({ id: "c1", type: "value", w: 3, h: 2, pids: [12], style: "big", ...over });
const doc = (cards: unknown[] = [card()]) => ({
  version: 1,
  updatedAt: 5,
  dashboards: [{ id: "d1", name: "Main", sections: [{ id: "s1", title: "Engine", cards }] }],
});

describe("sizeLimits and clampSize", () => {
  test("per-type ranges from decisions.md", () => {
    expect(sizeLimits(card({ type: "value" }))).toEqual({ min: [2, 1], max: [6, 3] });
    expect(sizeLimits(card({ type: "bar", style: "plain" }))).toEqual({ min: [2, 1], max: [6, 2] });
    expect(sizeLimits(card({ type: "gauge", style: "arc" }))).toEqual({ min: [2, 2], max: [6, 4] });
    expect(sizeLimits(card({ type: "gauge", style: "ring", pids: [1, 2] })).min).toEqual([3, 3]);
    expect(sizeLimits(card({ type: "gauge", style: "ring", pids: [1, 2, 3] })).min).toEqual([4, 3]);
    expect(sizeLimits(card({ type: "chart", style: "line", pids: [1] })).min).toEqual([2, 2]);
    expect(sizeLimits(card({ type: "chart", style: "line", pids: [1, 2] })).min).toEqual([3, 2]);
    expect(sizeLimits(card({ type: "chart", style: "lanes", pids: [1, 2, 3, 4] })).min).toEqual([6, 4]);
    expect(sizeLimits(card({ type: "dtc", style: "rows", pids: [] }))).toEqual({ min: [3, 3], max: [6, 6] });
    expect(sizeLimits(card({ type: "dtc", style: "count", pids: [] }))).toEqual({ min: [3, 2], max: [6, 2] });
    expect(sizeLimits(card({ type: "polling", style: "switch", pids: [] }))).toEqual({ min: [3, 2], max: [6, 3] });
  });
  test("clamps to range and rounds; garbage falls back to the default", () => {
    expect(clampSize(card({ type: "value" }), 99, 0)).toEqual({ w: 6, h: 1 });
    expect(clampSize(card({ type: "value" }), 2.6, 1.2)).toEqual({ w: 3, h: 1 });
    expect(clampSize(card({ type: "value" }), NaN, "x" as never)).toEqual({ w: 3, h: 2 });
  });
  test("overview with a VIN: hero needs 5x2 or 3x4", () => {
    const o = card({ type: "overview", style: "hero", pids: [], items: [{ t: "pid", pid: 1 }, { t: "vin" }] });
    expect(clampSize(o, 3, 2)).toEqual({ w: 3, h: 4 });
    expect(clampSize(o, 5, 2)).toEqual({ w: 5, h: 2 });
    expect(clampSize({ ...o, style: "list" }, 3, 2)).toEqual({ w: 4, h: 2 });
  });
});

describe("parseDashboards", () => {
  test("valid document round-trips", () => {
    const d = parseDashboards(JSON.stringify(doc()));
    expect(d?.dashboards[0].sections[0].cards[0]).toMatchObject({ id: "c1", type: "value", w: 3, h: 2, pids: [12], style: "big" });
    expect(d?.updatedAt).toBe(5);
  });
  test("hostile input never throws and yields null", () => {
    for (const raw of [null, undefined, "", "{", "[]", "null", "123", '"x"', "{}", '{"dashboards":5}', '{"dashboards":[]}', '{"dashboards":[null,1,"a"]}', "\u0000", "x".repeat(300_000), {}, 5, []]) {
      expect(() => parseDashboards(raw as never)).not.toThrow();
      expect(parseDashboards(raw as never)).toBeNull();
    }
  });
  test("proto pollution keys are ignored", () => {
    const d = parseDashboards('{"__proto__":{"x":1},"version":1,"updatedAt":1,"dashboards":[{"id":"d","name":"n","sections":[{"id":"s","cards":[{"id":"c","type":"value","w":3,"h":2,"pids":[1],"__proto__":{"y":1}}]}]}]}');
    expect(d).not.toBeNull();
    expect(({} as Record<string, unknown>).x).toBeUndefined();
    expect(({} as Record<string, unknown>).y).toBeUndefined();
  });
  test("bad cards are dropped, good ones kept", () => {
    const d = parseDashboards(JSON.stringify(doc([card(), null, { id: "x", type: "nope" }, card({ id: "c3", type: "value", pids: [] }), card({ id: "c4", pids: ["a"] as never })])));
    expect(d?.dashboards[0].sections[0].cards.map((c) => c.id)).toEqual(["c1"]);
  });
  test("sizes are clamped, unknown style falls back, duplicate ids get fresh ids", () => {
    const d = parseDashboards(JSON.stringify(doc([card({ w: 99, h: 99, style: "weird" }), card()])));
    const [a, b] = d!.dashboards[0].sections[0].cards;
    expect(a).toMatchObject({ w: 6, h: 3, style: "big" });
    expect(b.id).not.toBe("c1");
    expect(b.id.length).toBeGreaterThan(0);
  });
  test("thresholds must be finite with warn != critical, otherwise dropped", () => {
    const ok = parseDashboards(JSON.stringify(doc([card({ type: "bar", style: "zones", thresholds: { warn: 80, critical: 90 } })])));
    expect(ok!.dashboards[0].sections[0].cards[0].thresholds).toEqual({ warn: 80, critical: 90 });
    const bad = parseDashboards(JSON.stringify(doc([card({ type: "bar", style: "zones", thresholds: { warn: "a", critical: null } as never })])));
    expect(bad!.dashboards[0].sections[0].cards[0].thresholds).toBeUndefined();
  });
  test("caps are enforced on load by truncation", () => {
    const many = Array.from({ length: 30 }, (_, i) => card({ id: `c${i}` }));
    const d = parseDashboards(JSON.stringify(doc(many)));
    expect(d!.dashboards[0].sections[0].cards.length).toBe(CAPS.cardsPerSection);
    const dashboards = Array.from({ length: 10 }, (_, i) => ({ id: `d${i}`, name: "n", sections: Array.from({ length: 14 }, (_, j) => ({ id: `s${i}${j}`, cards: Array.from({ length: 24 }, (_, k) => card({ id: `c${i}${j}${k}` })) })) }));
    const big = parseDashboards(JSON.stringify({ version: 1, updatedAt: 1, dashboards }));
    expect(big!.dashboards.length).toBe(CAPS.dashboards);
    expect(big!.dashboards[0].sections.length).toBe(CAPS.sectionsPerDashboard);
    const total = big!.dashboards.flatMap((d) => d.sections).reduce((n, s) => n + s.cards.length, 0);
    expect(total).toBe(CAPS.cardsTotal);
  });
  test("names and titles are trimmed and bounded; missing updatedAt becomes 0", () => {
    const raw = { version: 1, dashboards: [{ id: "d", name: "  " + "n".repeat(100), sections: [{ id: "s", title: 5, cards: [] }] }] };
    const d = parseDashboards(JSON.stringify(raw));
    expect(d!.dashboards[0].name.length).toBe(40);
    expect(d!.dashboards[0].sections[0].title).toBe("");
    expect(d!.updatedAt).toBe(0);
  });
  test("a dashboard with no valid sections is kept with no sections", () => {
    const d = parseDashboards(JSON.stringify({ version: 1, updatedAt: 1, dashboards: [{ id: "d", name: "x", sections: "bad" }] }));
    expect(d!.dashboards[0].sections).toEqual([]);
  });
});

describe("defaults", () => {
  test("default template is valid and survives parse", () => {
    const d = defaultDashboards(1);
    expect(d.dashboards[0].name).toBe("Driving");
    expect(parseDashboards(JSON.stringify(d))).toEqual(d);
  });
  test("newId is unique", () => {
    expect(new Set(Array.from({ length: 500 }, () => newId())).size).toBe(500);
  });
});

describe("gauge range", () => {
  test("a gauge keeps a valid per-card range", () => {
    const c = normalizeCard({ type: "gauge", style: "arc", pids: [12], w: 3, h: 3, range: { min: 0, max: 6000 } }, new Set());
    expect(c?.range).toEqual({ min: 0, max: 6000 });
  });
  test("a range with min not below max, or non-numbers, is dropped", () => {
    expect(normalizeCard({ type: "gauge", style: "arc", pids: [12], w: 3, h: 3, range: { min: 5, max: 5 } }, new Set())?.range).toBeUndefined();
    expect(normalizeCard({ type: "gauge", style: "arc", pids: [12], w: 3, h: 3, range: { min: "a", max: 9 } }, new Set())?.range).toBeUndefined();
  });
  test("a huge range is dropped, not stored", () => {
    expect(normalizeCard({ type: "gauge", style: "arc", pids: [12], w: 3, h: 3, range: { min: 0, max: 1e12 } }, new Set())?.range).toBeUndefined();
  });
  test("other card types never keep a range", () => {
    expect(normalizeCard({ type: "value", style: "big", pids: [12], w: 3, h: 2, range: { min: 0, max: 9 } }, new Set())?.range).toBeUndefined();
  });
});

describe("overview items", () => {
  const ov = (extra: Record<string, unknown>) => normalizeCard({ type: "overview", style: "grid", pids: [], w: 6, h: 3, ...extra }, new Set());
  test("old cards migrate: PIDs first, then VIN, then DTC", () => {
    expect(ov({ pids: [5, 12], showVin: true, showDtc: true })?.items).toEqual([{ t: "pid", pid: 5 }, { t: "pid", pid: 12 }, { t: "vin" }, { t: "dtc" }]);
  });
  test("items keep their order; labels are trimmed and capped at 24 characters", () => {
    expect(ov({ items: [{ t: "dtc" }, { t: "pid", pid: 5, label: "  Coolant  " }, { t: "pid", pid: 6, label: "x".repeat(40) }] })?.items).toEqual([
      { t: "dtc" },
      { t: "pid", pid: 5, label: "Coolant" },
      { t: "pid", pid: 6, label: "x".repeat(24) },
    ]);
  });
  test("an empty label is dropped", () => {
    expect(ov({ items: [{ t: "pid", pid: 5, label: "   " }] })?.items).toEqual([{ t: "pid", pid: 5 }]);
  });
  test("unknown items, duplicate PIDs and a second VIN or DTC are dropped", () => {
    expect(ov({ items: [{ t: "vin" }, { t: "vin" }, { t: "x" }, { t: "pid", pid: 5 }, { t: "pid", pid: 5 }, { t: "dtc" }, { t: "dtc" }] })?.items).toEqual([
      { t: "vin" },
      { t: "pid", pid: 5 },
      { t: "dtc" },
    ]);
  });
  test("at most six items", () => {
    const items = Array.from({ length: 9 }, (_, i) => ({ t: "pid", pid: i + 1 }));
    expect(ov({ items })?.items?.length).toBe(6);
  });
  test("an overview keeps no PIDs of its own and no show flags", () => {
    const c = ov({ items: [{ t: "pid", pid: 5 }] });
    expect(c?.pids).toEqual([]);
    expect(c && "showVin" in c).toBe(false);
  });
  test("a hero with a VIN item needs 5 x 2 or 3 x 4", () => {
    const o = card({ type: "overview", style: "hero", pids: [], items: [{ t: "pid", pid: 1 }, { t: "vin" }] });
    expect(clampSize(o, 3, 2)).toEqual({ w: 3, h: 4 });
    expect(clampSize(o, 5, 2)).toEqual({ w: 5, h: 2 });
  });
});
