import { describe, expect, test } from "bun:test";
import {
  ACCENTS,
  DEFAULT_APPEARANCE,
  STORAGE_KEYS,
  applyAppearance,
  glassFactors,
  nextOption,
  parseBgVar,
  parseGlass,
  parseAppearance,
  readAppearance,
  writeAppearance,
} from "./appearance";

function fakeStore(initial: Record<string, string> = {}) {
  const data = new Map(Object.entries(initial));
  return {
    data,
    getItem: (key: string) => data.get(key) ?? null,
    setItem: (key: string, value: string) => {
      data.set(key, value);
    },
  };
}

describe("parseAppearance", () => {
  test("uses the defaults when nothing is stored", () => {
    expect(parseAppearance({})).toEqual(DEFAULT_APPEARANCE);
    expect(DEFAULT_APPEARANCE).toEqual({ theme: "system", accent: "orange", ctl: "clear", glass: 50, bg: "default", bgVar: 12 });
  });

  test("keeps valid values", () => {
    expect(parseAppearance({ theme: "dark", accent: "rose", ctl: "strong" })).toEqual({
      theme: "dark",
      accent: "rose",
      ctl: "strong",
      glass: 50,
      bg: "default",
      bgVar: 12,
    });
  });

  test("replaces unknown, wrong-case and prototype-key values with defaults", () => {
    expect(parseAppearance({ theme: "DARK", accent: "__proto__", ctl: "constructor" })).toEqual(
      DEFAULT_APPEARANCE,
    );
    expect(parseAppearance({ theme: 1, accent: null, ctl: ["strong"] })).toEqual(DEFAULT_APPEARANCE);
  });
});

describe("readAppearance", () => {
  test("reads the three storage keys", () => {
    const store = fakeStore({ "cs-theme": "light", "cs-accent": "teal", "cs-ctl": "strong" });
    expect(readAppearance(store)).toEqual({ theme: "light", accent: "teal", ctl: "strong", glass: 50, bg: "default", bgVar: 12 });
  });

  test("falls back to defaults when storage is missing or throws", () => {
    expect(readAppearance(null)).toEqual(DEFAULT_APPEARANCE);
    const throwing = {
      getItem: () => {
        throw new Error("SecurityError");
      },
    };
    expect(readAppearance(throwing)).toEqual(DEFAULT_APPEARANCE);
  });
});

describe("writeAppearance", () => {
  test("writes all three keys", () => {
    const store = fakeStore();
    writeAppearance(store, { theme: "dark", accent: "blue", ctl: "strong", glass: 30, bg: "contour", bgVar: 40 });
    expect(store.data.get(STORAGE_KEYS.theme)).toBe("dark");
    expect(store.data.get(STORAGE_KEYS.accent)).toBe("blue");
    expect(store.data.get(STORAGE_KEYS.ctl)).toBe("strong");
    expect(store.data.get(STORAGE_KEYS.glass)).toBe("30");
    expect(store.data.get(STORAGE_KEYS.bg)).toBe("contour");
    expect(store.data.get(STORAGE_KEYS.bgVar)).toBe("40");
  });

  test("does not throw when storage refuses writes", () => {
    const throwing = {
      setItem: () => {
        throw new Error("QuotaExceededError");
      },
    };
    expect(() => writeAppearance(throwing, DEFAULT_APPEARANCE)).not.toThrow();
    expect(() => writeAppearance(null, DEFAULT_APPEARANCE)).not.toThrow();
  });

  test("a written value reads back unchanged", () => {
    const store = fakeStore();
    const value = { theme: "system", accent: "violet", ctl: "clear", glass: 80, bg: "contour", bgVar: 77 } as const;
    writeAppearance(store, value);
    expect(readAppearance(store)).toEqual(value);
  });
});

describe("applyAppearance", () => {
  test("sets data-theme, data-accent and data-ctl on the root element", () => {
    const root = { dataset: {} as DOMStringMap };
    applyAppearance(root, { theme: "light", accent: "green", ctl: "strong", glass: 50, bg: "default", bgVar: 12 });
    expect(root.dataset).toEqual({ theme: "light", accent: "green", ctl: "strong", bg: "default" });
  });
});

describe("nextOption", () => {
  const options = ["system", "light", "dark"] as const;

  test("moves forward and back with arrow keys and wraps at the ends", () => {
    expect(nextOption(options, "system", "ArrowRight")).toBe("light");
    expect(nextOption(options, "dark", "ArrowDown")).toBe("system");
    expect(nextOption(options, "system", "ArrowLeft")).toBe("dark");
    expect(nextOption(options, "light", "ArrowUp")).toBe("system");
  });

  test("ignores other keys", () => {
    expect(nextOption(options, "light", "Enter")).toBeUndefined();
  });
});

describe("ACCENTS", () => {
  test("has the eight DESIGN.md presets, orange first, with unique ids", () => {
    expect(ACCENTS.map((a) => a.id)).toEqual([
      "orange",
      "yellow",
      "green",
      "teal",
      "blue",
      "violet",
      "rose",
      "neutral",
    ]);
  });
});

describe("glass", () => {
  test("parseGlass clamps, rounds and rejects junk", () => {
    expect(parseGlass("70")).toBe(70);
    expect(parseGlass(120)).toBe(100);
    expect(parseGlass(-5)).toBe(0);
    expect(parseGlass(33.6)).toBe(34);
    for (const bad of ["", "abc", NaN, null, undefined, [], {}]) expect(parseGlass(bad)).toBe(50);
  });

  test("factors are 1 at the default and tiers differ elsewhere", () => {
    expect(glassFactors(50)).toEqual({ float: 1, card: 1, inner: 1 });
    const clear = glassFactors(100);
    expect(clear.card).toBeLessThan(clear.inner);
    expect(clear.inner).toBeLessThan(clear.float);
    const solid = glassFactors(0);
    expect(solid.card).toBeGreaterThan(solid.inner);
    expect(solid.inner).toBeGreaterThan(solid.float);
  });
});

describe("background", () => {
  test("an unknown background falls back to the default", () => {
    expect(parseAppearance({ bg: "plasma" }).bg).toBe("default");
    expect(parseAppearance({ bg: "contour" }).bg).toBe("contour");
  });
  test("variation is a clamped whole number, else the default", () => {
    expect(parseBgVar("40.6")).toBe(41);
    expect(parseBgVar(-5)).toBe(0);
    expect(parseBgVar(900)).toBe(100);
    expect(parseBgVar("abc")).toBe(12);
    expect(parseBgVar(NaN)).toBe(12);
  });
  test("applyAppearance sets data-bg", () => {
    const root = { dataset: {} as DOMStringMap };
    applyAppearance(root, { ...DEFAULT_APPEARANCE, bg: "contour" });
    expect(root.dataset.bg).toBe("contour");
  });
});
