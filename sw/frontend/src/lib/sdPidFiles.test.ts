import { describe, expect, test } from "bun:test";
import {
  DEFAULT_SD_FOLDER,
  MAX_PID_FILE_BYTES,
  cardFilePath,
  folderChoices,
  importSummary,
  normalizeSdFolder,
  parsePidFile,
  readFolder,
  serializePidFile,
  treePath,
  validFileName,
} from "./sdPidFiles";

const def = (over: Record<string, unknown> = {}) => ({
  id: 1,
  mode: 1,
  pid: 12,
  length: 2,
  name: "RPM",
  unit: "rpm",
  desc: "",
  formula: "getPID(A,B)",
  minV: null,
  maxV: null,
  priority: 5,
  interval: 512,
  color: 0x01aaff,
  icon: "",
  ...over,
});

describe("normalizeSdFolder", () => {
  test("accepts the mount path and folders under it", () => {
    expect(normalizeSdFolder("/sdcard/pids")).toEqual({ ok: true, folder: "/sdcard/pids" });
    expect(normalizeSdFolder("/sdcard")).toEqual({ ok: true, folder: "/sdcard" });
    expect(DEFAULT_SD_FOLDER).toBe("/sdcard/configs");
  });

  test("normalises slashes and treats other paths as relative to the card", () => {
    expect(normalizeSdFolder("sdcard/pids/")).toEqual({ ok: true, folder: "/sdcard/pids" });
    expect(normalizeSdFolder("/pids")).toEqual({ ok: true, folder: "/sdcard/pids" });
    expect(normalizeSdFolder("//pids//track_day/")).toEqual({ ok: true, folder: "/sdcard/pids/track_day" });
    expect(normalizeSdFolder("/")).toEqual({ ok: true, folder: "/sdcard" });
  });

  test("rejects empty input and parent or current directory parts", () => {
    expect(normalizeSdFolder("  ").ok).toBe(false);
    expect(normalizeSdFolder("/sdcard/../etc").ok).toBe(false);
    expect(normalizeSdFolder("/pids/..").ok).toBe(false);
    expect(normalizeSdFolder("/pids/./x").ok).toBe(false);
  });

  test("rejects characters outside letters, digits, dash and underscore", () => {
    expect(normalizeSdFolder("/pids/a b").ok).toBe(false);
    expect(normalizeSdFolder("/pids/a.json").ok).toBe(false);
    expect(normalizeSdFolder("/pids/%2e%2e").ok).toBe(false);
  });

  test("rejects a path longer than 255 characters", () => {
    expect(normalizeSdFolder("/" + "a".repeat(260)).ok).toBe(false);
  });
});

describe("card paths", () => {
  test("maps the folder to the card-relative paths the file API takes", () => {
    expect(treePath("/sdcard/pids")).toBe("pids");
    expect(treePath("/sdcard")).toBe("");
    expect(cardFilePath("/sdcard/pids", "ford.json")).toBe("/pids/ford.json");
    expect(cardFilePath("/sdcard", "ford.json")).toBe("/ford.json");
  });
});

describe("validFileName", () => {
  test("adds .json and accepts letters, digits, dash and underscore", () => {
    expect(validFileName("ford_focus-2")).toEqual({ ok: true, name: "ford_focus-2.json" });
    expect(validFileName(" winter.json ")).toEqual({ ok: true, name: "winter.json" });
  });

  test("rejects separators, parent directories, empty and long names", () => {
    expect(validFileName("").ok).toBe(false);
    expect(validFileName("a/b").ok).toBe(false);
    expect(validFileName("..").ok).toBe(false);
    expect(validFileName("a\\b").ok).toBe(false);
    expect(validFileName("x".repeat(33)).ok).toBe(false);
    expect(validFileName("a.b").ok).toBe(false);
  });
});

describe("parsePidFile", () => {
  test("reads a PID array in the file shape", () => {
    const defs = parsePidFile(JSON.stringify([def(), def({ id: 2, pid: 13, name: "Speed" })]));
    expect(defs.map((d) => d.pid)).toEqual([12, 13]);
    expect(defs[0].name).toBe("RPM");
    expect(defs[0].update_interval_ms).toBe(512);
    expect(defs[0].description).toBe("");
  });

  test("accepts the short key names of older files", () => {
    const [d] = parsePidFile(JSON.stringify([{ id: 1, mode: 1, pid: 5, len: 1, name: "T", formula: "A-40", interval: 1000 }]));
    expect(d.length).toBe(1);
    expect(d.update_interval_ms).toBe(1000);
  });

  test("rejects hostile input with a reason and never throws anything else", () => {
    const bad = (text: string) => {
      try {
        parsePidFile(text);
        return "accepted";
      } catch (e) {
        return e instanceof Error ? e.message : "not an Error";
      }
    };
    expect(bad("{nope")).toBe("Not valid JSON.");
    expect(bad('{"pid":1}')).toBe("Not a PID list (a JSON array).");
    expect(bad("[1,2]")).toMatch(/^PID 1: /);
    expect(bad("[]")).toBe("accepted");
    expect(bad(JSON.stringify([def({ formula: "" })]))).toMatch(/formula/);
    expect(bad(JSON.stringify([def({ pid: "x" })]))).toMatch(/pid/);
    expect(bad(JSON.stringify([def({ interval: 3 })]))).toMatch(/update interval/);
    expect(bad(JSON.stringify([def(), def()]))).toMatch(/share 0x000C/);
    expect(bad(JSON.stringify(Array.from({ length: 257 }, (_, i) => def({ pid: i % 200, id: i }))))).toMatch(/256/);
    expect(bad("x".repeat(MAX_PID_FILE_BYTES + 1))).toMatch(/larger than 64 KiB/);
  });
});

describe("serializePidFile", () => {
  test("writes the file shape that parsePidFile reads back", () => {
    const defs = parsePidFile(JSON.stringify([def({ minV: 0, maxV: 100 })]));
    const text = serializePidFile(defs);
    expect(JSON.parse(text)[0]).toMatchObject({ pid: 12, interval: 512, desc: "", minV: 0, maxV: 100 });
    expect(parsePidFile(text)).toEqual(defs);
  });
});

describe("importSummary", () => {
  test("counts imported PIDs that are new, and existing ones that now differ from the device", () => {
    const imported = [
      { pid: 13, pending: false }, // already listed and the same as the device
      { pid: 14, pending: true }, // already listed, now differs
      { pid: 15, pending: true }, // new to the list
    ];
    expect(importSummary(new Set([13, 14]), imported)).toEqual({ added: 1, changed: 1 });
    expect(importSummary(new Set(), [])).toEqual({ added: 0, changed: 0 });
  });
});

describe("readFolder", () => {
  const reply = (status: number, body?: unknown) =>
    Promise.resolve(new Response(body === undefined ? null : JSON.stringify(body), { status }));

  test("lists only .json files, sorted, with their sizes", async () => {
    const out = await readFolder("/sdcard/pids", () =>
      reply(200, {
        path: "/pids",
        status: "success",
        children: [
          { name: "b.json", type: "file", size: 20 },
          { name: "dtcs.bin", type: "file", size: 9 },
          { path: "/pids/sub", children: [] },
          { name: "a.JSON", type: "file", size: 10 },
          { name: "x/y.json", type: "file", size: 1 },
        ],
      }),
    );
    expect(out).toEqual({
      kind: "ok",
      files: [
        { name: "a.JSON", size: 10 },
        { name: "b.json", size: 20 },
      ],
    });
  });

  test("maps the card and folder statuses", async () => {
    expect(await readFolder("/sdcard/x", () => reply(503))).toEqual({ kind: "nocard" });
    expect(await readFolder("/sdcard/x", () => reply(404))).toEqual({ kind: "missing" });
    expect((await readFolder("/sdcard/x", () => reply(500))).kind).toBe("error");
  });

  test("treats a malformed or unreachable reply as an error", async () => {
    expect((await readFolder("/sdcard", () => reply(200, { nope: 1 }))).kind).toBe("error");
    expect((await readFolder("/sdcard", () => Promise.resolve(new Response("{", { status: 200 })))).kind).toBe("error");
    expect((await readFolder("/sdcard", () => Promise.reject(new TypeError("offline")))).kind).toBe("error");
  });
});

describe("folderChoices", () => {
  test("lists every folder on the card with its .json count", () => {
    const tree = {
      path: "/",
      children: [
        { name: "a.json", type: "file", size: 1 },
        {
          path: "/pids",
          children: [
            { name: "x.json", type: "file", size: 1 },
            { name: "y.json", type: "file", size: 1 },
            { name: "d.bin", type: "file", size: 1 },
            { path: "/pids/track", children: [{ name: "z.json", type: "file", size: 1 }] },
          ],
        },
        { path: "/logs", children: [], truncated: true },
      ],
    };
    expect(folderChoices(tree)).toEqual([
      { folder: "/sdcard", jsonCount: 1 },
      { folder: "/sdcard/pids", jsonCount: 2 },
      { folder: "/sdcard/pids/track", jsonCount: 1 },
      { folder: "/sdcard/logs", jsonCount: 0 },
    ]);
  });

  test("ignores hostile nodes", () => {
    expect(folderChoices(null)).toEqual([]);
    expect(folderChoices({ path: "/", children: [null, 4, { path: 7 }, { path: "/x", children: "no" }] })).toEqual([
      { folder: "/sdcard", jsonCount: 0 },
      { folder: "/sdcard/x", jsonCount: 0 },
    ]);
  });
});
