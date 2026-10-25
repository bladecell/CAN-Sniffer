export const THEMES = ["system", "light", "dark"] as const;
export type Theme = (typeof THEMES)[number];

export const ACCENTS = [
  { id: "orange", label: "Orange", hue: 55, chroma: 0.17 },
  { id: "yellow", label: "Yellow", hue: 95, chroma: 0.14 },
  { id: "green", label: "Green", hue: 150, chroma: 0.15 },
  { id: "teal", label: "Teal", hue: 190, chroma: 0.11 },
  { id: "blue", label: "Blue", hue: 255, chroma: 0.17 },
  { id: "violet", label: "Violet", hue: 295, chroma: 0.19 },
  { id: "rose", label: "Rose", hue: 5, chroma: 0.2 },
  { id: "neutral", label: "Neutral", hue: 0, chroma: 0 },
] as const;
export type Accent = (typeof ACCENTS)[number]["id"];
export const ACCENT_IDS: readonly Accent[] = ACCENTS.map((a) => a.id);

export const CONTROL_FILLS = ["clear", "strong"] as const;
export type ControlFill = (typeof CONTROL_FILLS)[number];

export interface Appearance {
  theme: Theme;
  accent: Accent;
  ctl: ControlFill;
}

export const DEFAULT_APPEARANCE: Appearance = { theme: "system", accent: "orange", ctl: "clear" };

export const STORAGE_KEYS = { theme: "cs-theme", accent: "cs-accent", ctl: "cs-ctl" } as const;

/** Returns `value` when it is one of `options`, otherwise `fallback`. */
function pick<T extends string>(options: readonly T[], value: unknown, fallback: T): T {
  return options.find((o) => o === value) ?? fallback;
}

export function parseAppearance(raw: Partial<Record<keyof Appearance, unknown>>): Appearance {
  return {
    theme: pick(THEMES, raw.theme, DEFAULT_APPEARANCE.theme),
    accent: pick(ACCENT_IDS, raw.accent, DEFAULT_APPEARANCE.accent),
    ctl: pick(CONTROL_FILLS, raw.ctl, DEFAULT_APPEARANCE.ctl),
  };
}

export function readAppearance(store: Pick<Storage, "getItem"> | null): Appearance {
  const get = (key: string): string | undefined => {
    try {
      return store?.getItem(key) ?? undefined;
    } catch {
      return undefined;
    }
  };
  return parseAppearance({
    theme: get(STORAGE_KEYS.theme),
    accent: get(STORAGE_KEYS.accent),
    ctl: get(STORAGE_KEYS.ctl),
  });
}

export function writeAppearance(store: Pick<Storage, "setItem"> | null, value: Appearance): void {
  for (const key of Object.keys(STORAGE_KEYS) as (keyof Appearance)[]) {
    try {
      store?.setItem(STORAGE_KEYS[key], value[key]);
    } catch {
      // Storage full or blocked: the choice still applies for this page load.
    }
  }
}

export function applyAppearance(root: { dataset: DOMStringMap }, value: Appearance): void {
  root.dataset.theme = value.theme;
  root.dataset.accent = value.accent;
  root.dataset.ctl = value.ctl;
}

/** The option after `current` for an arrow key (wrapping), or undefined for any other key. */
export function nextOption<T>(options: readonly T[], current: T, key: string): T | undefined {
  const step = key === "ArrowRight" || key === "ArrowDown" ? 1 : key === "ArrowLeft" || key === "ArrowUp" ? -1 : 0;
  if (step === 0) return undefined;
  const i = options.indexOf(current);
  return options[(i + step + options.length) % options.length];
}
