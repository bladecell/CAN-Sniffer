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

/** Surface transparency, 0 (most opaque) to 100 (most see-through). 50 is the designed look. */
export const GLASS_MIN = 0;
export const GLASS_MAX = 100;
export const GLASS_DEFAULT = 50;

/** Blur radius of every glass surface, in px. 10 is the default. */
export const BLUR_MIN = 0;
export const BLUR_MAX = 30;
export const BLUR_DEFAULT = 10;

export const BACKGROUNDS = [
  { id: "default", label: "Default" },
  { id: "contour", label: "Contour lines" },
  { id: "wave", label: "Sound wave" },
  { id: "shiny", label: "Simple shiny" },
  { id: "meteor", label: "Meteor" },
  { id: "overlay", label: "Shiny overlay" },
  { id: "circuit", label: "Circuit board" },
  { id: "layers", label: "Wave" },
  { id: "hexagon", label: "Hexagon" },
  { id: "polygon", label: "Polygon" },
] as const;
export type Background = (typeof BACKGROUNDS)[number]["id"];
export const BACKGROUND_IDS: readonly Background[] = BACKGROUNDS.map((b) => b.id);

/** Pattern variation, 0 to 100: morphs the selected background. */
export const BG_VAR_MIN = 0;
export const BG_VAR_MAX = 100;
export const BG_VAR_DEFAULT = 12;

export interface Appearance {
  theme: Theme;
  accent: Accent;
  ctl: ControlFill;
  glass: number;
  blur: number;
  bg: Background;
  bgVar: number;
}

export const DEFAULT_APPEARANCE: Appearance = { theme: "system", accent: "orange", ctl: "clear", glass: GLASS_DEFAULT, blur: BLUR_DEFAULT, bg: "default", bgVar: BG_VAR_DEFAULT };

export const STORAGE_KEYS = { theme: "cs-theme", accent: "cs-accent", ctl: "cs-ctl", glass: "cs-glass", blur: "cs-blur", bg: "cs-bg", bgVar: "cs-bgvar" } as const;

/** A whole number in 0..100, or the default for anything else (NaN, text, out of range is clamped). */
export function parseGlass(value: unknown): number {
  const n = typeof value === "number" ? value : typeof value === "string" && value.trim() !== "" ? Number(value) : NaN;
  return Number.isFinite(n) ? Math.round(Math.min(GLASS_MAX, Math.max(GLASS_MIN, n))) : GLASS_DEFAULT;
}

/** A whole number in BLUR_MIN..BLUR_MAX, or the default for anything else. */
export function parseBlur(value: unknown): number {
  const n = typeof value === "number" ? value : typeof value === "string" && value.trim() !== "" ? Number(value) : NaN;
  return Number.isFinite(n) ? Math.round(Math.min(BLUR_MAX, Math.max(BLUR_MIN, n))) : BLUR_DEFAULT;
}

/** A whole number in 0..100, or the default for anything else. */
export function parseBgVar(value: unknown): number {
  const n = typeof value === "number" ? value : typeof value === "string" && value.trim() !== "" ? Number(value) : NaN;
  return Number.isFinite(n) ? Math.round(Math.min(BG_VAR_MAX, Math.max(BG_VAR_MIN, n))) : BG_VAR_DEFAULT;
}

/** Multiples of each tier's designed alpha (app.css multiplies them in). One slider, three tiers that respond differently:
 * floats keep most of their body so text stays readable, cards swing the most (0 is fully solid in dark and light), inner rows and panels follow half way. */
export function glassFactors(level: number): { float: number; card: number; inner: number } {
  const t = (parseGlass(level) - GLASS_DEFAULT) / GLASS_DEFAULT; // -1 opaque .. 1 clear
  const lerp = (up: number, down: number) => Math.round((1 - t * (t < 0 ? up : down)) * 1000) / 1000;
  return { float: lerp(0.2, 0.4), card: lerp(1, 0.6), inner: lerp(0.5, 0.5) };
}

/** Returns `value` when it is one of `options`, otherwise `fallback`. */
function pick<T extends string>(options: readonly T[], value: unknown, fallback: T): T {
  return options.find((o) => o === value) ?? fallback;
}

export function parseAppearance(raw: Partial<Record<keyof Appearance, unknown>>): Appearance {
  return {
    theme: pick(THEMES, raw.theme, DEFAULT_APPEARANCE.theme),
    accent: pick(ACCENT_IDS, raw.accent, DEFAULT_APPEARANCE.accent),
    ctl: pick(CONTROL_FILLS, raw.ctl, DEFAULT_APPEARANCE.ctl),
    glass: parseGlass(raw.glass),
    blur: parseBlur(raw.blur),
    bg: pick(BACKGROUND_IDS, raw.bg, DEFAULT_APPEARANCE.bg),
    bgVar: parseBgVar(raw.bgVar),
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
    glass: get(STORAGE_KEYS.glass),
    blur: get(STORAGE_KEYS.blur),
    bg: get(STORAGE_KEYS.bg),
    bgVar: get(STORAGE_KEYS.bgVar),
  });
}

export function writeAppearance(store: Pick<Storage, "setItem"> | null, value: Appearance): void {
  for (const key of Object.keys(STORAGE_KEYS) as (keyof Appearance)[]) {
    try {
      store?.setItem(STORAGE_KEYS[key], String(value[key]));
    } catch {
      // Storage full or blocked: the choice still applies for this page load.
    }
  }
}

export function applyAppearance(
  root: { dataset: DOMStringMap; style?: Pick<CSSStyleDeclaration, "setProperty"> },
  value: Appearance,
): void {
  root.dataset.theme = value.theme;
  root.dataset.accent = value.accent;
  root.dataset.ctl = value.ctl;
  root.dataset.bg = value.bg;
  const k = glassFactors(value.glass);
  root.style?.setProperty("--k-float", String(k.float));
  root.style?.setProperty("--k-card", String(k.card));
  root.style?.setProperty("--k-inner", String(k.inner));
  root.style?.setProperty("--blur", `${parseBlur(value.blur)}px`);
}

/** The option after `current` for an arrow key (wrapping), or undefined for any other key. */
export function nextOption<T>(options: readonly T[], current: T, key: string): T | undefined {
  const step = key === "ArrowRight" || key === "ArrowDown" ? 1 : key === "ArrowLeft" || key === "ArrowUp" ? -1 : 0;
  if (step === 0) return undefined;
  const i = options.indexOf(current);
  return options[(i + step + options.length) % options.length];
}
