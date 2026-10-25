import { describe, expect, test } from "bun:test";
import {
  ACCENTS,
  DEFAULT_APPEARANCE,
  STORAGE_KEYS,
  applyAppearance,
  nextOption,
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
    expect(DEFAULT_APPEARANCE).toEqual({ theme: "system", accent: "orange", ctl: "clear" });
  });

  test("keeps valid values", () => {
    expect(parseAppearance({ theme: "dark", accent: "rose", ctl: "strong" })).toEqual({
      theme: "dark",
      accent: "rose",
      ctl: "strong",
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
    expect(readAppearance(store)).toEqual({ theme: "light", accent: "teal", ctl: "strong" });
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
    writeAppearance(store, { theme: "dark", accent: "blue", ctl: "strong" });
    expect(store.data.get(STORAGE_KEYS.theme)).toBe("dark");
    expect(store.data.get(STORAGE_KEYS.accent)).toBe("blue");
    expect(store.data.get(STORAGE_KEYS.ctl)).toBe("strong");
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
    const value = { theme: "system", accent: "violet", ctl: "clear" } as const;
    writeAppearance(store, value);
    expect(readAppearance(store)).toEqual(value);
  });
});

describe("applyAppearance", () => {
  test("sets data-theme, data-accent and data-ctl on the root element", () => {
    const root = { dataset: {} as DOMStringMap };
    applyAppearance(root, { theme: "light", accent: "green", ctl: "strong" });
    expect(root.dataset).toEqual({ theme: "light", accent: "green", ctl: "strong" });
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
