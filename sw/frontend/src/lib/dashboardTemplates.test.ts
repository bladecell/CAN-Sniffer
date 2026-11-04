import { describe, expect, test } from "bun:test";
import { CAPS, defaultDashboards, normalizeCard, parseDashboards, sectionsFromTemplate, sizeLimits, type Section } from "./dashboardModel";
import { TEMPLATES, templateById, templateForName } from "./dashboardTemplates";

const allIds = (sections: Section[]) => sections.flatMap((s) => [s.id, ...s.cards.map((c) => c.id)]);

describe("templates", () => {
  test("four starter templates, Driving first, with unique ids and names", () => {
    expect(TEMPLATES.map((t) => t.id)).toEqual(["driving", "diagnostics", "chart", "minimal"]);
    expect(TEMPLATES.map((t) => t.name)).toEqual(["Driving", "Diagnostics", "Chart view", "Minimal"]);
  });

  test("every template fits the caps and the per-type size limits as declared", () => {
    for (const t of TEMPLATES) {
      expect(t.sections.length).toBeLessThanOrEqual(CAPS.sectionsPerDashboard);
      let total = 0;
      for (const s of t.sections) {
        expect(s.cards.length).toBeLessThanOrEqual(CAPS.cardsPerSection);
        total += s.cards.length;
        for (const c of s.cards) {
          const { min, max } = sizeLimits(c);
          expect(c.w).toBeGreaterThanOrEqual(min[0]);
          expect(c.w).toBeLessThanOrEqual(max[0]);
          expect(c.h).toBeGreaterThanOrEqual(min[1]);
          expect(c.h).toBeLessThanOrEqual(max[1]);
        }
      }
      expect(total).toBeLessThanOrEqual(CAPS.cardsTotal);
    }
  });

  test("every declared card survives normalisation unchanged (no dropped card, no clamped size)", () => {
    for (const t of TEMPLATES) {
      for (const s of t.sections) {
        for (const c of s.cards) {
          const n = normalizeCard(c, new Set());
          expect(n).not.toBeNull();
          expect([n!.type, n!.style, n!.w, n!.h, n!.pids]).toEqual([c.type, c.style, c.w, c.h, c.pids]);
        }
      }
    }
  });

  test("every template dashboard passes parseDashboards with the same section and card counts", () => {
    for (const t of TEMPLATES) {
      const sections = sectionsFromTemplate(t);
      const doc = { version: 1, updatedAt: 1, dashboards: [{ id: "d1", name: t.name, sections }] };
      const parsed = parseDashboards(JSON.stringify(doc));
      expect(parsed).not.toBeNull();
      const got = parsed!.dashboards[0].sections;
      expect(got.length).toBe(t.sections.length);
      expect(got.reduce((n, s) => n + s.cards.length, 0)).toBe(t.sections.reduce((n, s) => n + s.cards.length, 0));
    }
  });

  test("template PIDs are ones the stock PID set defines", () => {
    const stock = new Set([4, 5, 12, 8707, 8710, 9747]);
    for (const t of TEMPLATES) {
      for (const s of t.sections) for (const c of s.cards) {
        for (const p of [...c.pids, ...(c.items ?? []).flatMap((i) => (i.t === "pid" ? [i.pid] : []))]) expect(stock.has(p)).toBe(true);
      }
    }
  });

  test("instances get fresh ids and never share one with another instance", () => {
    const a = allIds(sectionsFromTemplate(templateById("driving")));
    const b = allIds(sectionsFromTemplate(templateById("driving")));
    expect(new Set(a).size).toBe(a.length);
    expect(a.some((id) => b.includes(id))).toBe(false);
  });

  test("the default dashboard is the Driving template", () => {
    const d = defaultDashboards(1).dashboards[0];
    expect(d.name).toBe("Driving");
    expect(d.sections.map((s) => s.cards.map((c) => c.type))).toEqual(
      templateById("driving").sections.map((s) => s.cards.map((c) => c.type)),
    );
  });

  test("Driving shows RPM and coolant, and its control cards", () => {
    const cards = templateById("driving").sections.flatMap((s) => s.cards);
    const pids = cards.flatMap((c) => c.pids);
    expect(pids).toEqual(expect.arrayContaining([12, 5]));
    expect(cards.map((c) => c.type)).toEqual(expect.arrayContaining(["polling", "recorder"]));
  });

  test("a dashboard takes its template by name, Driving when none matches", () => {
    expect(templateForName("Chart view").id).toBe("chart");
    expect(templateForName("Minimal").id).toBe("minimal");
    expect(templateForName("Trip").id).toBe("driving");
  });
});
