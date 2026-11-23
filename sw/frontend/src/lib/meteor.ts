// "Meteor" background: short diagonal streaks whose tails fade out, on a gradient. Absolute coordinates,
// so the pattern does not depend on the window size. The slider glides every streak to a new place.
import { hash, morph } from "./soundWave";

export type MeteorOpts = {
  width: number; height: number; seed: number;
  phase: number; // continuous morph, see soundWave.morph
  bgColor: string; bgColor2: string;
  colors: [string, string, string]; // streak colours; one tail-to-head gradient each
  opacity: number;
};

const r1 = (n: number) => Math.round(n * 10) / 10;
const WIDTHS = [0.6, 0.8, 1]; // of the base stroke
const COS45 = Math.SQRT1_2;

export function meteorSvg(o: MeteorOpts): string {
  const diag = Math.hypot(o.width, o.height), short = Math.min(o.width, o.height);
  const count = Math.max(8, Math.round(diag / 45)), base = Math.max(4, short / 160);
  let paths = "";
  for (let k = 0; k < count; k++) {
    const x = morph(o.phase, (z) => hash(k * 2, o.seed + z * 31)) * o.width;
    const y = morph(o.phase, (z) => hash(k * 2 + 1, o.seed + z * 31)) * o.height;
    const len = short * (0.12 + 0.3 * hash(k, o.seed + 5000)) * COS45;
    const w = base * WIDTHS[Math.floor(hash(k, o.seed + 6000) * 3)];
    const c = Math.floor(hash(k, o.seed + 7000) * 3);
    const fade = 0.4 + 0.6 * hash(k, o.seed + 8000); // depth: some streaks sit further back
    paths += `<path d="M${r1(x - len)} ${r1(y - len)}L${r1(x)} ${r1(y)}" stroke="url(#m${c})" stroke-width="${r1(w)}" stroke-opacity="${r1(fade * 100) / 100}"/>`;
  }
  const grads = o.colors
    .map((col, i) => `<linearGradient id="m${i}" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${col}" stop-opacity="0"/><stop offset="1" stop-color="${col}"/></linearGradient>`)
    .join("");
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">` +
    `<defs><linearGradient id="bg" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${o.bgColor}"/><stop offset="1" stop-color="${o.bgColor2}"/></linearGradient>${grads}</defs>` +
    `<rect width="100%" height="100%" fill="url(#bg)"/><g fill="none" stroke-linecap="round" opacity="${o.opacity}">${paths}</g></svg>`;
}
