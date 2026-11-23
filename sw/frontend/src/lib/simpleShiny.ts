// "Simple shiny" background: a flat base with two soft blobs, a darker one in the top-left corner and a lighter one in the
// bottom-right. Absolute coordinates, so the pattern does not depend on the window size.
import { smoothPath } from "./contourBg";
import { hash, morph } from "./soundWave";

export type ShinyOpts = {
  width: number; height: number; seed: number;
  phase: number; // continuous morph, see soundWave.morph
  baseColor: string; darkColor: string; lightColor: string;
};

const POINTS = 10;
const r1 = (n: number) => Math.round(n * 10) / 10;

/** A closed blob centred on (cx, cy). Radius 0.5 to 1.0 of its size, chosen per point; size spans the short to the long screen side. */
function blob(cx: number, cy: number, o: ShinyOpts, seed: number, spin: number): string {
  const lo = Math.min(o.width, o.height), hi = Math.max(o.width, o.height);
  const size = morph(o.phase, (z) => lo + (hi - lo) * hash(0, seed + z * 31));
  const pts: [number, number][] = [];
  for (let i = 0; i < POINTS; i++) {
    const rad = (size / 2) * (0.5 + 0.5 * morph(o.phase, (z) => hash(i + 1, seed + z * 31 + 1)));
    const a = (i / POINTS) * Math.PI * 2 + o.phase * spin; // the slider also turns the blob
    pts.push([cx + Math.cos(a) * rad, cy + Math.sin(a) * rad]);
  }
  return smoothPath(pts, true, r1);
}

export function simpleShinySvg(o: ShinyOpts): string {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">` +
    `<rect width="100%" height="100%" fill="${o.baseColor}"/>` +
    `<path d="${blob(0, 0, o, o.seed, 0.25)}" fill="${o.darkColor}"/>` +
    `<path d="${blob(o.width, o.height, o, o.seed + 777, -0.2)}" fill="${o.lightColor}"/></svg>`;
}
