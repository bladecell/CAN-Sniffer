// "Wave" background: filled wave layers stacked from the bottom, back layer highest. Absolute coordinates, so the pattern
// does not depend on the window size. The slider glides every crest to a new height.
import { smoothPath } from "./contourBg";
import { hash, morph } from "./soundWave";

export type WaveLayersOpts = {
  width: number; height: number; seed: number;
  phase: number; // continuous morph, see soundWave.morph
  bgColor: string; bgColor2: string;
  layers: string[]; // fills, front (lowest) first
};

const r1 = (n: number) => Math.round(n * 10) / 10;
const SEGMENT_SHARE = [0.45, 0.3, 0.2]; // wave length per layer as a share of the short side: the front layer is the longest

export function waveLayersSvg(o: WaveLayersOpts): string {
  const count = o.layers.length, band = o.height / count, short = Math.min(o.width, o.height);
  let paths = "";
  // Layer 0 is the front (lowest); draw from the back so the front paints last.
  for (let s = count - 1; s >= 0; s--) {
    const base = o.height - band * s - band / 2;
    const segs = Math.max(2, Math.round(o.width / (short * (SEGMENT_SHARE[s] ?? 0.2))));
    const pts: [number, number][] = [];
    for (let i = 0; i <= segs; i++) {
      const up = i % 2 === 0 ? -1 : 1; // crests and troughs alternate
      const h = morph(o.phase, (z) => hash(i + s * 101, o.seed + z * 31));
      pts.push([i === segs ? o.width : (o.width / segs) * i, base + up * band * 0.45 * (0.2 + 0.8 * h)]);
    }
    paths += `<path d="${smoothPath(pts, false, r1)}L${r1(o.width)} ${r1(o.height)}L0 ${r1(o.height)}z" fill="${o.layers[s]}"/>`;
  }
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">` +
    `<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${o.bgColor}"/><stop offset="1" stop-color="${o.bgColor2}"/></linearGradient></defs>` +
    `<rect width="100%" height="100%" fill="url(#g)"/>${paths}</svg>`;
}
