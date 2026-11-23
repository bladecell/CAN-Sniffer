import { ACCENTS, type Appearance } from "./appearance";
import { contourSvg, type ContourOpts } from "./contourBg";
import { circuitBoardSvg, type CircuitOpts } from "./circuitBoard";
import { polygonNetSvg, type PolygonOpts } from "./polygonNet";
import { hexGridSvg, type HexOpts } from "./hexGrid";
import { waveLayersSvg, type WaveLayersOpts } from "./layerWaves";
import { meteorSvg, type MeteorOpts } from "./meteor";
import { shinyOverlaySvg, type OverlayOpts } from "./shinyOverlay";
import { simpleShinySvg, type ShinyOpts } from "./simpleShiny";
import { soundWaveSvg } from "./soundWave";

/** Colours from the accent hue and chroma, light or dark. */
export function contourColors(hue: number, chroma: number, dark: boolean) {
  return dark
    ? { lineColor: `oklch(0.78 ${chroma} ${hue})`, bgColor: `oklch(0.17 ${chroma * 0.06} ${hue})`, bgColor2: `oklch(0.25 ${chroma * 0.18} ${hue})` }
    : { lineColor: `oklch(0.4 ${chroma} ${hue})`, bgColor: `oklch(0.95 ${chroma * 0.1} ${hue})`, bgColor2: `oklch(0.88 ${chroma * 0.35} ${hue})` };
}

/** Line width shared by the line-based patterns: 1.5 px at 1080, never under 1. */
const lineWidth = (screenW: number, screenH: number) => Math.max(1, Math.min(screenW, screenH) / 720);

/** Slider value 0 is "Random": a pattern seeded from the date, so it changes once a day. */
export const todaySeed = (d: Date = new Date()) => d.getFullYear() * 10000 + (d.getMonth() + 1) * 100 + d.getDate();
const FIXED_SEED = 7, RANDOM_PHASE = 3;

/** Sized from the physical screen, never the window, so resizing only crops the pattern. */
export function contourOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): ContourOpts {
  const side = Math.max(screenW, screenH, 1), scale = Math.max(80, Math.min(screenW, screenH) * 0.28);
  return {
    width: side, height: side, seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4, scale, octaves: 3, lo: 0.25, hi: 0.75,
    levels: Math.max(4, Math.min(10, Math.round(screenW / 195))),
    cell: Math.max(5, Math.round(scale / 22)),
    stroke: lineWidth(screenW, screenH), lineOpacity: 0.25, gradient: true, ...contourColors(hue, chroma, dark),
  };
}

/** Bars a little wider on big screens; the band they sit in is the screen height, so they stay centred in the view. */
export function waveOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): ContourOpts {
  return {
    ...contourOptions(screenW, screenH, variation, hue, chroma, dark, today),
    width: Math.max(screenW, screenH, 1), height: Math.max(screenH, 1),
    scale: Math.max(12, Math.min(26, Math.round(Math.min(screenW, screenH) * 0.022))),
    lineOpacity: 0.25,
  };
}

/** Flat base plus two accent-tinted blobs; the whole canvas is the physical screen. */
export function shinyOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): ShinyOpts {
  return {
    width: Math.max(screenW, 1), height: Math.max(screenH, 1),
    seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4,
    ...(dark
      ? { baseColor: `oklch(0.2 ${chroma * 0.3} ${hue})`, darkColor: `oklch(0.15 ${chroma * 0.2} ${hue})`, lightColor: `oklch(0.26 ${chroma * 0.45} ${hue})` }
      : { baseColor: `oklch(0.93 ${chroma * 0.2} ${hue})`, darkColor: `oklch(0.88 ${chroma * 0.3} ${hue})`, lightColor: `oklch(0.97 ${chroma * 0.12} ${hue})` }),
  };
}

/** Streaks in three accent tints on the same gradient as the contour background. */
export function meteorOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): MeteorOpts {
  const { bgColor, bgColor2 } = contourColors(hue, chroma, dark);
  const L = dark ? [0.88, 0.8, 0.92] : [0.42, 0.5, 0.36];
  return {
    width: Math.max(screenW, 1), height: Math.max(screenH, 1),
    seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4,
    bgColor, bgColor2, opacity: 0.5,
    colors: [`oklch(${L[0]} ${chroma} ${hue})`, `oklch(${L[1]} ${chroma} ${hue + 30})`, `oklch(${L[2]} ${chroma * 0.4} ${hue})`],
  };
}

/** The contour gradient under light and dark facets. */
export function overlayOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): OverlayOpts {
  const { bgColor, bgColor2 } = contourColors(hue, chroma, dark);
  return {
    width: Math.max(screenW, 1), height: Math.max(screenH, 1),
    seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4,
    bgColor, bgColor2,
  };
}

/** Each slider value is its own board (no morph between them); 0 is the board of the day. */
export function circuitOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): CircuitOpts {
  const { bgColor, bgColor2, lineColor } = contourColors(hue, chroma, dark);
  return {
    width: Math.max(screenW, 1), height: Math.max(screenH, 1),
    seed: variation === 0 ? todaySeed(today) : variation,
    grid: Math.max(16, Math.min(32, Math.round(Math.min(screenW, screenH) / 43))),
    bgColor, bgColor2, lineColor, opacity: 0.3,
  };
}

/** Three wave layers in accent tints, each a step further from the background. */
export function layersOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): WaveLayersOpts {
  const { bgColor, bgColor2 } = contourColors(hue, chroma, dark);
  const layers = (dark ? [0.34, 0.29, 0.24] : [0.82, 0.86, 0.9]).map((l, i) => `oklch(${l} ${chroma * (0.45 - i * 0.1)} ${hue})`);
  return {
    width: Math.max(screenW, 1), height: Math.max(screenH, 1),
    seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4,
    bgColor, bgColor2, layers,
  };
}

/** Two hex grids sized from the short screen side (50 and 75 px at 1080). */
export function hexOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): HexOpts {
  const { bgColor, bgColor2, lineColor } = contourColors(hue, chroma, dark);
  const short = Math.max(Math.min(screenW, screenH), 1);
  return {
    width: Math.max(screenW, 1), height: Math.max(screenH, 1),
    seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4,
    side: [Math.max(28, Math.round(short * 0.046)), Math.max(42, Math.round(short * 0.07))],
    bgColor, bgColor2, lineColor, opacity: [0.25, 0.35], stroke: Math.max(1.5, short / 540),
  };
}

/** Points, links and dots in the accent colour; sizes follow the short screen side. */
export function polygonOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): PolygonOpts {
  const { bgColor, bgColor2, lineColor } = contourColors(hue, chroma, dark);
  const short = Math.max(Math.min(screenW, screenH), 1);
  return {
    width: Math.max(screenW, 1), height: Math.max(screenH, 1),
    seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4,
    bgColor, bgColor2, lineColor, opacity: 0.3, stroke: lineWidth(screenW, screenH), dot: Math.max(2.5, short / 216),
  };
}

/** Keeps `--bg-image` on the root in step with the appearance. Runs for the life of the page. */
export function startBackground(appearance: Appearance, root: HTMLElement): void {
  const mq = matchMedia("(prefers-color-scheme: dark)");
  let systemDark = $state(mq.matches);
  mq.addEventListener("change", () => (systemDark = mq.matches));
  let blobUrl = "";
  let latest = 0;

  $effect.root(() => {
    $effect(() => {
      const { bg, bgVar, accent, theme } = appearance;
      if (bg === "default") return;
      const a = ACCENTS.find((x) => x.id === accent) ?? ACCENTS[0];
      const dark = theme === "dark" || (theme === "system" && systemDark);
      const frame = requestAnimationFrame(async () => {
        const [w, h] = [screen.width, screen.height];
        const svg = bg === "wave" ? soundWaveSvg(waveOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : bg === "polygon" ? polygonNetSvg(polygonOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : bg === "hexagon" ? hexGridSvg(hexOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : bg === "layers" ? waveLayersSvg(layersOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : bg === "circuit" ? circuitBoardSvg(circuitOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : bg === "overlay" ? shinyOverlaySvg(overlayOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : bg === "meteor" ? meteorSvg(meteorOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : bg === "shiny" ? simpleShinySvg(shinyOptions(w, h, bgVar, a.hue, a.chroma, dark))
          : contourSvg(contourOptions(w, h, bgVar, a.hue, a.chroma, dark));
        const next = URL.createObjectURL(new Blob([svg], { type: "image/svg+xml" }));
        const mine = ++latest;
        // Decode first: swapping straight to an undecoded image blanks the background while it loads.
        const img = new Image();
        img.src = next;
        await img.decode().catch(() => {});
        if (mine !== latest) return URL.revokeObjectURL(next);
        root.style.setProperty("--bg-image", `url("${next}")`);
        if (blobUrl) URL.revokeObjectURL(blobUrl);
        blobUrl = next;
      });
      return () => cancelAnimationFrame(frame);
    });
  });
}
