// "Hexagon" background: two overlapping hex grids of outlined cells, a random share of cells present, the larger grid with
// dots on its corners. Absolute coordinates, so the pattern does not depend on the window size. The slider adds and removes cells.
import { hash, morph } from "./soundWave";

export type HexOpts = {
  width: number; height: number; seed: number;
  phase: number; // continuous morph, see soundWave.morph
  side: [number, number]; // cell side per layer, px; the second layer gets corner dots
  bgColor: string; bgColor2: string; lineColor: string;
  opacity: [number, number]; stroke: number;
};

const SQ3 = 1.7321;
const r1 = (n: number) => Math.round(n * 10) / 10;
const SHARE = 0.32; // fraction of cells drawn

function layer(o: HexOpts, side: number, li: number, dots: boolean): { cells: string; dots: string } {
  const cols = Math.ceil(o.width / (SQ3 * side)) + 1, rows = Math.ceil(o.height / (1.5 * side) + 0.5) + 1;
  const corners = [[0, -side], [(SQ3 * side) / 2, -side / 2], [(SQ3 * side) / 2, side / 2], [0, side], [(-SQ3 * side) / 2, side / 2], [(-SQ3 * side) / 2, -side / 2]];
  const dotR = Math.max(3, side / 10), seen = new Set<string>();
  let cells = "", dotPath = "";
  for (let f = 0; f < cols; f++) for (let e = 0; e < rows; e++) {
    // A cell shows while its blended value stays under the share: sliding adds and removes cells one by one.
    if (morph(o.phase, (z) => hash(f * 4099 + e, o.seed + li * 7919 + z * 31)) >= SHARE) continue;
    const cx = SQ3 * side * f + (e % 2) * ((SQ3 * side) / 2), cy = 1.5 * side * e;
    const pts = corners.map(([dx, dy]) => [r1(cx + dx), r1(cy + dy)] as const);
    cells += `M${pts.map((p) => `${p[0]} ${p[1]}`).join("L")}z`;
    if (dots) for (const [x, y] of pts) {
      const key = `${x},${y}`;
      if (seen.has(key)) continue;
      seen.add(key);
      dotPath += `M${r1(x - dotR)} ${y}a${r1(dotR)} ${r1(dotR)} 0 1 0 ${r1(2 * dotR)} 0a${r1(dotR)} ${r1(dotR)} 0 1 0 ${r1(-2 * dotR)} 0z`;
    }
  }
  return { cells, dots: dotPath };
}

export function hexGridSvg(o: HexOpts): string {
  const out = o.side.map((side, li) => {
    const l = layer(o, Math.max(8, side), li, li === 1);
    return `<g opacity="${o.opacity[li]}" fill="${o.lineColor}" stroke="${o.lineColor}"><path d="${l.cells}" fill="none" stroke-width="${o.stroke}" stroke-linejoin="round"/>` +
      (l.dots ? `<path d="${l.dots}" stroke="none"/>` : "") + `</g>`;
  });
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">` +
    `<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${o.bgColor}"/><stop offset="1" stop-color="${o.bgColor2}"/></linearGradient></defs>` +
    `<rect width="100%" height="100%" fill="url(#g)"/>${out.join("")}</svg>`;
}
