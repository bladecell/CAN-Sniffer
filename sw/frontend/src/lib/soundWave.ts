// Sound-wave background, "three bar" style: vertical bars of varying height, each cut into 1 to 3 rounded segments.
// Absolute coordinates only, so the pattern does not depend on the window size. Reuses the contour option and frame types.
import { svgFrame, type ContourOpts } from "./contourBg";

export const hash = (i: number, s: number) => {
  let h = Math.imul(i, 374761393) ^ Math.imul(s, 668265263);
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967295;
};
export const fade = (t: number) => t * t * t * (t * (t * 6 - 15) + 10);
const noise1 = (x: number, s: number) => {
  const i = Math.floor(x), t = fade(x - i), a = hash(i, s);
  return a + (hash(i + 1, s) - a) * t;
};
const r1 = (n: number) => Math.round(n * 10) / 10;

/** Blends layer `z` and `z + 1` of a pattern by the fractional part of `phase`, so a slider morphs it smoothly. */
export function morph(phase: number, at: (layer: number) => number): number {
  const z = Math.floor(phase), a = at(z);
  return a + (at(z + 1) - a) * fade(phase - z);
}

/** Bar height 0..1. Two noise layers blend by the fractional phase, so the slider morphs the wave smoothly. */
function height01(i: number, o: ContourOpts, freq: number): number {
  const z = Math.floor(o.phase), t = fade(o.phase - z);
  const at = (layer: number) => 0.7 * (0.65 * noise1(i * freq, o.seed + layer) + 0.35 * noise1(i * freq * 2.7, o.seed + layer + 500)) + 0.3 * hash(i, o.seed + layer);
  const a = at(z);
  return Math.min(1, Math.max(0, (a + (at(z + 1) - a) * t - 0.2) / 0.6));
}

/** o.scale is the bar width, o.height the visible band the bars are centred in. */
export function soundWaveSvg(o: ContourOpts): string {
  const w = o.scale, p = w * 0.75, maxH = o.height * 0.6, minH = maxH / 10, cy = o.height / 2;
  const bars = Math.floor(o.width / w), x0 = (o.width - bars * w) / 2;
  let d = "";
  for (let i = 0; i < bars; i++) {
    const len = minH + (maxH - minH) * height01(i, o, 0.09);
    const x = r1(x0 + w * (i + 0.5)), top = cy - len / 2;
    // Taller bars may split into more pieces; the count is a stable per-bar pick.
    const pick = hash(i, o.seed + 9000);
    const pieces = len >= 7 * p ? 1 + Math.floor(pick * 3) : len >= 5 * p ? 1 + Math.floor(pick * 2) : 1;
    const gap = p * 0.6, usable = len - (pieces - 1) * gap;
    const weights = Array.from({ length: pieces }, (_, k) => 0.6 + hash(i * 7 + k, o.seed + 4000));
    const total = weights.reduce((a, b) => a + b, 0);
    let y = top;
    for (let k = 0; k < pieces; k++) {
      const seg = (usable * weights[k]) / total;
      // Round caps extend half a stroke past each end.
      const a = y + p / 2, b = y + seg - p / 2;
      d += b > a ? `M${x} ${r1(a)}L${x} ${r1(b)}` : `M${x} ${r1(y + seg / 2)}L${x} ${r1(y + seg / 2)}`;
      y += seg + gap;
    }
  }
  return svgFrame({ ...o, stroke: p }, d);
}
