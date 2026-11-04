import { describe, expect, test } from "bun:test";
import { createDashboardPersistence, DASHBOARDS_PATH, DASHBOARDS_TMP_PATH, LOCAL_KEY, type SdFiles } from "./dashboardPersistence";
import type { DashboardsDoc } from "./dashboardModel";

const docAt = (updatedAt: number, name = "D"): DashboardsDoc => ({ version: 1, updatedAt, dashboards: [{ id: "d" + updatedAt, name, sections: [] }] });

function setup(opts: { local?: string; sd?: string | number | "throw"; now?: number } = {}) {
  const store = new Map<string, string>();
  if (opts.local !== undefined) store.set(LOCAL_KEY, opts.local);
  const files = new Map<string, string>();
  if (typeof opts.sd === "string" && opts.sd !== "throw") files.set(DASHBOARDS_PATH, opts.sd);
  const calls: string[] = [];
  const sd: SdFiles = {
    async download(p) {
      calls.push(`get ${p}`);
      if (opts.sd === "throw") throw new Error("net");
      if (typeof opts.sd === "number") return new Response("{}", { status: opts.sd });
      const t = files.get(p);
      return t === undefined ? new Response("{}", { status: 404 }) : new Response(t);
    },
    async upload(p, blob) { calls.push(`put ${p}`); files.set(p, await blob.text()); return new Response("{}"); },
    async copy(s, d) { calls.push(`copy ${s} ${d}`); const t = files.get(s); if (t === undefined) return new Response("{}", { status: 404 }); files.set(d, t); return new Response("{}"); },
    async remove(p) { calls.push(`del ${p}`); files.delete(p); return new Response("{}"); },
  };
  const p = createDashboardPersistence({
    storage: { getItem: (k) => store.get(k) ?? null, setItem: (k, v) => void store.set(k, v) },
    sd,
    now: () => opts.now ?? 1000,
  });
  return { p, store, files, calls };
}

describe("local copy", () => {
  test("round trip, corrupt and absent give null", () => {
    const { p, store } = setup();
    expect(p.loadLocal()).toBeNull();
    p.saveLocal(docAt(3));
    expect(p.loadLocal()?.updatedAt).toBe(3);
    store.set(LOCAL_KEY, "{garbage");
    expect(p.loadLocal()).toBeNull();
  });
  test("a throwing storage never throws out", () => {
    const p = createDashboardPersistence({
      storage: { getItem: () => { throw new Error("denied"); }, setItem: () => { throw new Error("quota"); } },
      sd: setup().p as never,
      now: () => 1,
    });
    expect(p.loadLocal()).toBeNull();
    expect(p.saveLocal(docAt(1))).toBe(false);
  });
});

describe("writeSd", () => {
  test("temp file then atomic copy, then temp removed", async () => {
    const { p, calls, files } = setup();
    expect(await p.writeSd(docAt(7))).toBeNull();
    expect(calls).toEqual([`put ${DASHBOARDS_TMP_PATH}`, `copy ${DASHBOARDS_TMP_PATH} ${DASHBOARDS_PATH}`, `del ${DASHBOARDS_TMP_PATH}`]);
    expect(JSON.parse(files.get(DASHBOARDS_PATH)!).updatedAt).toBe(7);
  });
  test("failed copy reports a message and leaves the old file", async () => {
    const { p, files } = setup({ sd: JSON.stringify(docAt(1)) });
    const orig = files.get(DASHBOARDS_PATH);
    const failing = createDashboardPersistence({
      storage: { getItem: () => null, setItem: () => {} },
      sd: { download: async () => new Response(""), upload: async () => new Response("{}"), copy: async () => new Response('{"reason":"SD busy"}', { status: 503 }), remove: async () => new Response("{}") },
      now: () => 1,
    });
    expect(await failing.writeSd(docAt(2))).toContain("503");
    expect(files.get(DASHBOARDS_PATH)).toBe(orig);
    void p;
  });
  test("a rejected request is a message, not a throw", async () => {
    const failing = createDashboardPersistence({
      storage: { getItem: () => null, setItem: () => {} },
      sd: { download: async () => new Response(""), upload: async () => { throw new Error("offline"); }, copy: async () => new Response(""), remove: async () => new Response("") },
      now: () => 1,
    });
    expect(await failing.writeSd(docAt(2))).toBe("offline");
  });
});

describe("save", () => {
  test("logged out: local only", async () => {
    const { p, calls, store } = setup();
    const r = await p.save(docAt(5), false);
    expect(r).toEqual({ sd: "skipped" });
    expect(calls).toEqual([]);
    expect(store.has(LOCAL_KEY)).toBe(true);
  });
  test("logged in: local and SD; SD failure keeps the local copy", async () => {
    const a = setup();
    expect(await a.p.save(docAt(5), true)).toEqual({ sd: "saved" });
    const b = createDashboardPersistence({
      storage: { getItem: () => null, setItem: () => {} },
      sd: { download: async () => new Response(""), upload: async () => new Response("no", { status: 500 }), copy: async () => new Response(""), remove: async () => new Response("") },
      now: () => 1,
    });
    const r = await b.save(docAt(1), true);
    expect(r.sd).toBe("failed");
  });
});

describe("sync after login", () => {
  test("newer SD wins and refreshes the local copy", async () => {
    const { p, store } = setup({ local: JSON.stringify(docAt(5, "local")), sd: JSON.stringify(docAt(9, "sd")) });
    const r = await p.sync();
    expect(r.source).toBe("sd");
    expect(r.doc.dashboards[0].name).toBe("sd");
    expect(JSON.parse(store.get(LOCAL_KEY)!).updatedAt).toBe(9);
  });
  test("newer local wins and is pushed to the SD card", async () => {
    const { p, files } = setup({ local: JSON.stringify(docAt(9, "local")), sd: JSON.stringify(docAt(5, "sd")) });
    const r = await p.sync();
    expect(r.source).toBe("local");
    expect(JSON.parse(files.get(DASHBOARDS_PATH)!).dashboards[0].name).toBe("local");
  });
  test("equal timestamps keep the SD copy", async () => {
    const { p } = setup({ local: JSON.stringify(docAt(5, "local")), sd: JSON.stringify(docAt(5, "sd")) });
    expect((await p.sync()).doc.dashboards[0].name).toBe("sd");
  });
  test("missing SD file: local is pushed; nothing at all gives the default template", async () => {
    const a = setup({ local: JSON.stringify(docAt(5)) });
    expect((await a.p.sync()).source).toBe("local");
    expect(a.files.has(DASHBOARDS_PATH)).toBe(true);
    const b = setup({ now: 42 });
    const r = await b.p.sync();
    expect(r.source).toBe("default");
    expect(r.doc.dashboards[0].name).toBe("Driving");
    // First launch: the default is a placeholder, never stored, so the next launch is still a first launch.
    expect(b.store.has(LOCAL_KEY)).toBe(false);
    expect(b.p.initial().source).toBe("default");
  });
  test("corrupt SD file falls back to local, then default, and is not overwritten", async () => {
    const a = setup({ local: JSON.stringify(docAt(5, "local")), sd: "{not json" });
    expect((await a.p.sync()).doc.dashboards[0].name).toBe("local");
    expect(a.files.get(DASHBOARDS_PATH)).toBe("{not json");
    const b = setup({ sd: '{"dashboards":[]}' });
    expect((await b.p.sync()).source).toBe("default");
  });
  test("network error, HTTP error and throwing download never reject", async () => {
    for (const sd of ["throw", 500, 403] as const) {
      const { p } = setup({ local: JSON.stringify(docAt(5, "local")), sd });
      const r = await p.sync();
      expect(r.doc.dashboards[0].name).toBe("local");
    }
  });
});

describe("initial", () => {
  test("local copy or default, synchronous", () => {
    expect(setup({ local: JSON.stringify(docAt(5, "local")) }).p.initial().doc.dashboards[0].name).toBe("local");
    expect(setup({ local: "junk" }).p.initial().source).toBe("default");
  });
});
