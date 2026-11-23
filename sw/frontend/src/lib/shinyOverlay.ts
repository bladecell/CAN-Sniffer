// "Shiny overlay" background: a gradient with four translucent light facets stacked from the top-right corner and four dark
// ones from the bottom-left. Absolute coordinates, so the pattern does not depend on the window size.
import { hash, morph } from "./soundWave";

export type OverlayOpts = {
  width: number; height: number; seed: number;
  phase: number; // continuous morph, see soundWave.morph
  bgColor: string; bgColor2: string;
};

const ALPHAS = [0.1, 0.075, 0.05, 0.025];
const r1 = (n: number) => Math.round(n * 10) / 10;

/** Four facets nested against one corner; `spread` widens or narrows them. */
function facets(o: OverlayOpts, fromRight: boolean, fromTop: boolean, spread: number, fill: string): string {
  const px = (x: number) => r1(fromRight ? o.width - x : x), py = (y: number) => r1(fromTop ? y : o.height - y);
  let s = 0, a = 0, out = "";
  ALPHAS.forEach((alpha, i) => {
    const d = Math.max(1.1, (2 - 0.2 * i) * spread);
    const l = Math.round((o.width - s) / d), u = Math.round((o.height - a) / d);
    const d0 = i === 0
      ? `M${px(0)} ${py(0)}L${px(l)} ${py(0)}L${px(0)} ${py(u)}z`
      : `M${px(s)} ${py(0)}L${px(0)} ${py(a)}L${px(0)} ${py(a + u)}L${px(s + l)} ${py(0)}z`;
    out += `<path d="${d0}" fill="${fill}" fill-opacity="${alpha}"/>`;
    s += l; a += u;
  });
  return out;
}

export function shinyOverlaySvg(o: OverlayOpts): string {
  // Two independent spreads (0.75 to 1.45), one per corner, so the slider reshapes both stacks.
  const spread = (k: number) => 0.75 + 0.7 * morph(o.phase, (z) => hash(k, o.seed + z * 31));
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">` +
    `<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${o.bgColor}"/><stop offset="1" stop-color="${o.bgColor2}"/></linearGradient></defs>` +
    `<rect width="100%" height="100%" fill="url(#g)"/>${facets(o, true, true, spread(1), "#fff")}${facets(o, false, false, spread(2), "#000")}</svg>`;
}
