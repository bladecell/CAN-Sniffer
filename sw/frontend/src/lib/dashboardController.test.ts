import { describe, expect, test } from "bun:test";
import { createDashboardController } from "./dashboardController";
import { createDashboardPersistence, LOCAL_KEY } from "./dashboardPersistence";
import { parseDashboards } from "./dashboardModel";
import { templateById } from "./dashboardTemplates";

function setup(opts: { loggedIn?: boolean; sdFails?: boolean } = {}) {
  const store = new Map<string, string>();
  const files = new Map<string, string>();
  let t = 100;
  const persistence = createDashboardPersistence({
    storage: { getItem: (k) => store.get(k) ?? null, setItem: (k, v) => void store.set(k, v) },
    sd: {
      download: async (p) => (files.has(p) ? new Response(files.get(p)) : new Response("", { status: 404 })),
      upload: async (p, b) => { if (opts.sdFails) return new Response("", { status: 500 }); files.set(p, await b.text()); return new Response("{}"); },
      copy: async (s, d) => { files.set(d, files.get(s)!); return new Response("{}"); },
      remove: async (p) => { files.delete(p); return new Response("{}"); },
    },
    now: () => t,
  });
  let changes = 0;
  const c = createDashboardController({ persistence, isLoggedIn: () => opts.loggedIn ?? false, now: () => ++t, onChange: () => void changes++ });
  return { c, store, files, changes: () => changes };
}

describe("controller", () => {
  test("a device with nothing stored is fresh until the user saves a layout", async () => {
    const { c, store } = setup();
    expect(c.view().fresh).toBe(true);
    c.startEdit();
    c.resetToTemplate(templateById("minimal"));
    await c.done();
    expect(c.view().fresh).toBe(false);
    expect(store.size).toBeGreaterThan(0);
  });
  test("an unreadable SD copy falls back to the local copy and is flagged", async () => {
    const { c, files } = setup({ loggedIn: true });
    files.set("config/dashboards.json", "{not json");
    await c.sync();
    expect(c.view().unreadable).toBe(true);
    expect(c.view().loading).toBe(false);
    expect(c.view().active.name).toBe("Driving");
  });
  test("a stored layout is not fresh", () => {
    const { c } = setup();
    c.startEdit();
    c.addDashboard("Trip");
    void c.done();
    expect(setup().c.view().fresh).toBe(true);
    expect(c.view().fresh).toBe(false);
  });
  test("starts from the local copy or the default template, with an active dashboard", () => {
    const { c } = setup();
    expect(c.view().dashboards.length).toBe(1);
    expect(c.view().active.name).toBe("Driving");
    expect(c.view().editing).toBe(false);
  });
  test("mutators refuse outside edit mode", () => {
    const { c } = setup();
    expect(c.addDashboard("x")).toBe("Not in edit mode.");
  });
  test("cancel discards, done commits and saves (local only when logged out)", async () => {
    const { c, store } = setup();
    c.startEdit();
    expect(c.addDashboard("Trip")).toBeNull();
    expect(c.view().dashboards.length).toBe(2);
    c.cancel();
    expect(c.view().dashboards.length).toBe(1);
    expect(c.view().editing).toBe(false);
    c.startEdit();
    c.addDashboard("Trip");
    await c.done();
    expect(c.view().dashboards.length).toBe(2);
    expect(parseDashboards(store.get(LOCAL_KEY))?.dashboards.length).toBe(2);
    expect(c.view().save.sd).toBe("skipped");
  });
  test("done while logged in writes the SD file; failure is reported and the edit is kept", async () => {
    const ok = setup({ loggedIn: true });
    ok.c.startEdit(); ok.c.addDashboard("Trip");
    await ok.c.done();
    expect(ok.files.has("config/dashboards.json")).toBe(true);
    expect(ok.c.view().save.sd).toBe("saved");
    const bad = setup({ loggedIn: true, sdFails: true });
    bad.c.startEdit(); bad.c.addDashboard("Trip");
    await bad.c.done();
    expect(bad.c.view().dashboards.length).toBe(2);
    expect(bad.c.view().save.sd).toBe("failed");
  });
  test("cap errors come back as messages and leave the draft unchanged", () => {
    const { c } = setup();
    c.startEdit();
    for (let i = 0; i < 7; i++) expect(c.addDashboard(`n${i}`)).toBeNull();
    expect(c.addDashboard("nine")).toContain("8");
    expect(c.view().dashboards.length).toBe(8);
  });
  test("active dashboard follows removal and cancel", () => {
    const { c } = setup();
    c.startEdit();
    c.addDashboard("B");
    const b = c.view().dashboards[1].id;
    c.setActive(b);
    expect(c.view().active.id).toBe(b);
    c.removeDashboard(b);
    expect(c.view().active.name).toBe("Driving");
    c.cancel();
    expect(c.view().dashboards.length).toBe(1);
    c.startEdit(); c.addDashboard("C"); c.setActive(c.view().dashboards[1].id); c.cancel();
    expect(c.view().active.name).toBe("Driving");
  });
  test("card mutators work on the active dashboard", () => {
    const { c } = setup();
    c.startEdit();
    const sid = c.view().active.sections[0].id;
    expect(c.addCard(sid, { type: "value", pids: [12] })).toBeNull();
    const card = c.view().active.sections[0].cards.at(-1)!;
    expect(c.duplicateCard(card.id)).toBeNull();
    c.addSection("Two");
    const s2 = c.view().active.sections[1].id;
    expect(c.moveCard(card.id, s2, 0)).toBeNull();
    expect(c.view().active.sections[1].cards[0].id).toBe(card.id);
    expect(c.removeCard(card.id)).toBeNull();
  });
  test("sync after login adopts a newer SD copy but not while editing", async () => {
    const { c, files } = setup({ loggedIn: true });
    const remote = JSON.stringify({ version: 1, updatedAt: 9_999_999, dashboards: [{ id: "r", name: "Remote", sections: [] }] });
    files.set("config/dashboards.json", remote);
    c.startEdit();
    await c.sync();
    expect(c.view().active.name).toBe("Driving");
    c.cancel();
    await c.sync();
    expect(c.view().active.name).toBe("Remote");
  });
  test("a new dashboard can start from a template", () => {
    const { c } = setup();
    c.startEdit();
    expect(c.addDashboard("Trip", templateById("chart"))).toBeNull();
    expect(c.view().active.name).toBe("Trip");
    expect(c.view().active.sections.length).toBe(templateById("chart").sections.length);
  });
  test("reset to template is a draft: Cancel restores the sections, Done keeps them, other dashboards stay", async () => {
    const { c, store } = setup();
    const driving = c.view().active;
    const minimal = templateById("minimal");
    const minimalCards = minimal.sections.reduce((n, s) => n + s.cards.length, 0);
    c.startEdit();
    c.addDashboard("Trip");
    c.setActive(driving.id);
    expect(c.resetToTemplate(minimal)).toBeNull();
    expect(c.view().active.sections.reduce((n, s) => n + s.cards.length, 0)).toBe(minimalCards);
    expect(c.view().dashboards.length).toBe(2);
    c.cancel();
    expect(c.view().dashboards.length).toBe(1);
    expect(c.view().active.sections).toEqual(driving.sections);
    c.startEdit();
    c.resetToTemplate(minimal);
    await c.done();
    expect(c.view().active.name).toBe("Driving");
    expect(c.view().active.sections.reduce((n, s) => n + s.cards.length, 0)).toBe(minimalCards);
    expect(parseDashboards(store.get(LOCAL_KEY))!.dashboards[0].sections.length).toBe(minimal.sections.length);
  });
  test("reset is refused outside edit mode", () => {
    const { c } = setup();
    expect(c.resetToTemplate(templateById("minimal"))).toBe("Not in edit mode.");
  });
});
