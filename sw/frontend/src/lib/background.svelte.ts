import { ACCENTS, type Appearance } from "./appearance";
import { contourSvg, type ContourOpts } from "./contourBg";

/** Colours from the accent hue and chroma, light or dark. */
export function contourColors(hue: number, chroma: number, dark: boolean) {
  return dark
    ? { lineColor: `oklch(0.78 ${chroma} ${hue})`, bgColor: `oklch(0.17 ${chroma * 0.15} ${hue})`, bgColor2: `oklch(0.27 ${chroma * 0.5} ${hue})` }
    : { lineColor: `oklch(0.4 ${chroma} ${hue})`, bgColor: `oklch(0.95 ${chroma * 0.1} ${hue})`, bgColor2: `oklch(0.88 ${chroma * 0.35} ${hue})` };
}

/** Slider value 0 is "Random": a pattern seeded from the date, so it changes once a day. */
export const todaySeed = (d: Date = new Date()) => d.getFullYear() * 10000 + (d.getMonth() + 1) * 100 + d.getDate();
const FIXED_SEED = 7, RANDOM_PHASE = 3;

/** Sized from the physical screen, never the window, so resizing only crops the pattern. */
export function contourOptions(screenW: number, screenH: number, variation: number, hue: number, chroma: number, dark: boolean, today: Date = new Date()): ContourOpts {
  const side = Math.max(screenW, screenH, 1), scale = Math.max(80, Math.min(screenW, screenH) * 0.28);
  return {
    width: side, height: side, seed: variation === 0 ? todaySeed(today) : FIXED_SEED, phase: variation === 0 ? RANDOM_PHASE : variation / 4, scale, octaves: 3, lo: 0.25, hi: 0.75,
    levels: Math.max(4, Math.min(11, Math.round(screenW / 170))),
    cell: Math.max(5, Math.round(scale / 22)),
    stroke: screenW < 700 ? 1.2 : 1.5, lineOpacity: 0.25, gradient: true, ...contourColors(hue, chroma, dark),
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
      if (bg !== "contour") return;
      const a = ACCENTS.find((x) => x.id === accent) ?? ACCENTS[0];
      const dark = theme === "dark" || (theme === "system" && systemDark);
      const frame = requestAnimationFrame(async () => {
        const svg = contourSvg(contourOptions(screen.width, screen.height, bgVar, a.hue, a.chroma, dark));
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
