import { describe, expect, test } from "bun:test";
import { CAPS, type Card, type DashboardsDoc, type Section, sectionsFromTemplate } from "./dashboardModel";
import * as ops from "./dashboardOps";
import { templateById } from "./dashboardTemplates";

const mk = (id: string): Card => ({ id, type: "value", w: 3, h: 2, pids: [1], style: "big" });
const base = (): DashboardsDoc => ({
  version: 1,
  updatedAt: 1,
  dashboards: [
    { id: "d1", name: "A", sections: [{ id: "s1", title: "", cards: [mk("c1"), mk("c2")] }, { id: "s2", title: "T", cards: [mk("c3")] }] },
    { id: "d2", name: "B", sections: [] },
  ],
});
const ok = <T extends { ok: boolean }>(r: T) => {
  if (!r.ok) throw new Error("expected ok: " + JSON.stringify(r));
  return (r as unknown as { doc: DashboardsDoc }).doc;
};
const ids = (d: DashboardsDoc, did: string, sid: string) => d.dashboards.find((x) => x.id === did)!.sections.find((s) => s.id === sid)!.cards.map((c) => c.id);

describe("dashboards", () => {
  test("add, rename, remove", () => {
    let d = ok(ops.addDashboard(base(), "  New  "));
    expect(d.dashboards.at(-1)!.name).toBe("New");
    d = ok(ops.renameDashboard(d, "d1", "Z"));
    expect(d.dashboards[0].name).toBe("Z");
    d = ok(ops.removeDashboard(d, "d2"));
    expect(d.dashboards.length).toBe(2);
  });
  test("input doc is never mutated", () => {
    const b = base();
    const snap = JSON.stringify(b);
    ops.addDashboard(b, "x"); ops.moveCard(b, "d1", "c1", "s2", 0); ops.removeCard(b, "d1", "c1");
    expect(JSON.stringify(b)).toBe(snap);
  });
  test("cap of 8 is rejected with a message", () => {
    let d = base();
    for (let i = 0; i < CAPS.dashboards - 2; i++) d = ok(ops.addDashboard(d, `n${i}`));
    const r = ops.addDashboard(d, "one too many");
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.message).toContain("8");
  });
  test("last dashboard cannot be removed, unknown ids fail without throwing", () => {
    expect(ops.removeDashboard(ok(ops.removeDashboard(base(), "d2")), "d1").ok).toBe(false);
    expect(ops.renameDashboard(base(), "nope", "x").ok).toBe(false);
    expect(ops.removeCard(base(), "nope", "c1").ok).toBe(false);
    expect(ops.moveCard(base(), "d1", "c1", "nope", 0).ok).toBe(false);
  });
  test("empty names fall back or are rejected", () => {
    expect(ops.renameDashboard(base(), "d1", "   ").ok).toBe(false);
  });
});

describe("sections", () => {
  test("add with optional title, rename, move, remove", () => {
    let d = ok(ops.addSection(base(), "d1", "Fuel"));
    const sid = d.dashboards[0].sections.at(-1)!.id;
    expect(d.dashboards[0].sections.at(-1)!.title).toBe("Fuel");
    d = ok(ops.renameSection(d, "d1", sid, ""));
    expect(d.dashboards[0].sections.at(-1)!.title).toBe("");
    d = ok(ops.moveSection(d, "d1", sid, 0));
    expect(d.dashboards[0].sections[0].id).toBe(sid);
    d = ok(ops.removeSection(d, "d1", sid));
    expect(d.dashboards[0].sections.length).toBe(2);
  });
  test("cap of 12 per dashboard", () => {
    let d = base();
    for (let i = 0; i < CAPS.sectionsPerDashboard - 2; i++) d = ok(ops.addSection(d, "d1"));
    expect(ops.addSection(d, "d1").ok).toBe(false);
  });
});

describe("cards", () => {
  test("add clamps size and assigns id", () => {
    const d = ok(ops.addCard(base(), "d1", "s1", { type: "value", pids: [5], style: "big", w: 99, h: 99 }));
    const c = d.dashboards[0].sections[0].cards.at(-1)!;
    expect(c).toMatchObject({ w: 6, h: 3, pids: [5] });
    expect(c.id.length).toBeGreaterThan(0);
  });
  test("add rejects invalid input", () => {
    expect(ops.addCard(base(), "d1", "s1", { type: "value", pids: [] }).ok).toBe(false);
    expect(ops.addCard(base(), "d1", "s1", { type: "bogus" as never, pids: [] }).ok).toBe(false);
  });
  test("update re-clamps size to the new style/pids", () => {
    const d = ok(ops.updateCard(base(), "d1", "c1", { type: "value", pids: [9], style: "spark", w: 1, h: 1 }));
    expect(d.dashboards[0].sections[0].cards[0]).toMatchObject({ pids: [9], style: "spark", w: 2, h: 1, id: "c1" });
  });
  test("move between sections at an index, and within a section", () => {
    let d = ok(ops.moveCard(base(), "d1", "c1", "s2", 0));
    expect(ids(d, "d1", "s1")).toEqual(["c2"]);
    expect(ids(d, "d1", "s2")).toEqual(["c1", "c3"]);
    d = ok(ops.moveCard(d, "d1", "c3", "s2", 0));
    expect(ids(d, "d1", "s2")).toEqual(["c3", "c1"]);
    d = ok(ops.moveCard(d, "d1", "c3", "s2", 99));
    expect(ids(d, "d1", "s2")).toEqual(["c1", "c3"]);
  });
  test("duplicate lands after the original with a new id", () => {
    const d = ok(ops.duplicateCard(base(), "d1", "c1"));
    const got = ids(d, "d1", "s1");
    expect(got.length).toBe(3);
    expect(got[0]).toBe("c1");
    expect(got[1]).not.toBe("c1");
    expect(got[2]).toBe("c2");
  });
  test("per-section cap 24 and total cap 150 reject add, duplicate and move", () => {
    const fill = (n: number): Card[] => Array.from({ length: n }, (_, i) => mk(`f${i}`));
    const full: DashboardsDoc = { ...base(), dashboards: [{ id: "d1", name: "A", sections: [{ id: "s1", title: "", cards: fill(24) }, { id: "s2", title: "", cards: [mk("x")] }] }] };
    const r = ops.addCard(full, "d1", "s1", { type: "value", pids: [1] });
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.message).toContain("24");
    expect(ops.duplicateCard(full, "d1", "f0").ok).toBe(false);
    expect(ops.moveCard(full, "d1", "x", "s1", 0).ok).toBe(false);
    expect(ops.moveCard(full, "d1", "f0", "s1", 3).ok).toBe(true);
    const sections = Array.from({ length: 7 }, (_, i) => ({ id: `s${i}`, title: "", cards: Array.from({ length: 21 }, (_, j) => mk(`k${i}-${j}`)) }));
    const total: DashboardsDoc = { ...base(), dashboards: [{ id: "d1", name: "A", sections: [...sections, { id: "z", title: "", cards: [] }] }] };
    expect(sections.length * 21).toBe(147);
    let d = ok(ops.addCard(total, "d1", "z", { type: "value", pids: [1] }));
    d = ok(ops.addCard(d, "d1", "z", { type: "value", pids: [1] }));
    d = ok(ops.addCard(d, "d1", "z", { type: "value", pids: [1] }));
    const r2 = ops.addCard(d, "d1", "z", { type: "value", pids: [1] });
    expect(r2.ok).toBe(false);
    if (!r2.ok) expect(r2.message).toContain("150");
  });
});

describe("templates and reset", () => {
  const minimal = () => templateById("minimal");
  const cardsOf = (sections: Section[]) => sections.reduce((n, s) => n + s.cards.length, 0);
  const sectionOf = (n: number, p: string): Section => ({ id: `${p}-s`, title: "", cards: Array.from({ length: n }, (_, i) => mk(`${p}-${i}`)) });
  const seven147 = (p: string): DashboardsDoc => ({
    ...base(),
    dashboards: [
      { id: "d1", name: "A", sections: [] },
      { id: "d2", name: "B", sections: Array.from({ length: 7 }, (_, i) => ({ id: `${p}${i}`, title: "", cards: Array.from({ length: 21 }, (_, j) => mk(`${p}${i}-${j}`)) })) },
    ],
  });

  test("add with template sections keeps the template's sections and cards", () => {
    const sections = sectionsFromTemplate(minimal());
    const d = ok(ops.addDashboard(base(), "Trip", sections));
    expect(d.dashboards.at(-1)!.name).toBe("Trip");
    expect(d.dashboards.at(-1)!.sections).toEqual(sections);
  });

  test("add with template is refused when it would take the total past 150", () => {
    const r = ops.addDashboard(seven147("t"), "Trip", [sectionOf(4, "x")]);
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.message).toContain("150");
  });

  test("reset replaces only the target's sections and keeps its id, name and the other dashboards", () => {
    const fresh = sectionsFromTemplate(minimal());
    const d = ok(ops.resetDashboard(base(), "d1", fresh));
    expect(d.dashboards[0]).toMatchObject({ id: "d1", name: "A" });
    expect(d.dashboards[0].sections).toEqual(fresh);
    expect(d.dashboards[1]).toEqual(base().dashboards[1]);
  });

  test("reset counts the other dashboards against the 150 cap and rejects unknown ids", () => {
    const r = ops.resetDashboard(seven147("u"), "d1", [sectionOf(4, "y")]);
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.message).toContain("150");
    expect(ok(ops.resetDashboard(seven147("v"), "d1", [sectionOf(3, "z")])).dashboards[0].sections.length).toBe(1);
    expect(ops.resetDashboard(base(), "nope", []).ok).toBe(false);
  });

  test("reset to an empty section list is allowed", () => {
    expect(cardsOf(ok(ops.resetDashboard(base(), "d1", [])).dashboards[0].sections)).toBe(0);
  });
});
