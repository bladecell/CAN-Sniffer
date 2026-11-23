// "Polygon luminary" background, single layer: jittered grid points inside an ellipse, each joined to a few of its nearest
// neighbours, with a dot on every point. Absolute coordinates, so the pattern does not depend on the window size.
// The slider glides the points to new places; links and missing points change as the blend crosses their thresholds.
import { hash, morph } from "./soundWave";

export type PolygonOpts = {
  width: number; height: number; seed: number;
  phase: number; // continuous morph, see soundWave.morph
  bgColor: string; bgColor2: string; lineColor: string;
  opacity: number; stroke: number; dot: number; // dot radius, px
};

const r1 = (n: number) => Math.round(n * 10) / 10;
const MAX_LINKS = 7, DROP = 0.08;

export function polygonNetSvg(o: PolygonOpts): string {
  const cell = Math.max(24, Math.min(150, Math.max(o.width, o.height) / 8));
  const cols = Math.ceil(o.width / cell) + 2, rows = Math.ceil(o.height / cell) + 2;
  const [cx, cy, rx, ry] = [o.width / 2, o.height / 2, (o.width / 2) * 0.95, (o.height / 2) * 0.95];
  const pts: { x: number; y: number; links: number }[] = [];
  for (let i = 0; i < cols; i++) for (let j = 0; j < rows; j++) {
    const k = i * 8191 + j, n = (salt: number) => morph(o.phase, (z) => hash(k * 4 + salt, o.seed + z * 31));
    const x = (i - 0.5) * cell + (n(0) - 0.5) * cell, y = (j - 0.5) * cell + (n(1) - 0.5) * cell;
    if (((x - cx) / rx) ** 2 + ((y - cy) / ry) ** 2 > 1 || n(2) < DROP) continue;
    pts.push({ x, y, links: Math.round(MAX_LINKS * (0.25 + 0.75 * n(3))) });
  }
  const seen = new Set<number>();
  let lines = "", dots = "";
  const d = o.dot;
  pts.forEach((p, a) => {
    dots += `M${r1(p.x - d)} ${r1(p.y)}a${r1(d)} ${r1(d)} 0 1 0 ${r1(2 * d)} 0a${r1(d)} ${r1(d)} 0 1 0 ${r1(-2 * d)} 0z`;
    const near = pts.map((q, b) => ({ b, dist: b === a ? Infinity : Math.hypot(p.x - q.x, p.y - q.y) })).sort((u, v) => u.dist - v.dist);
    for (let i = 0; i < Math.min(p.links, near.length); i++) {
      const { b, dist } = near[i];
      if (dist > 4 * cell) break;
      const key = Math.min(a, b) * pts.length + Math.max(a, b);
      if (seen.has(key)) continue;
      seen.add(key);
      lines += `M${r1(p.x)} ${r1(p.y)}L${r1(pts[b].x)} ${r1(pts[b].y)}`;
    }
  });
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">` +
    `<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${o.bgColor}"/><stop offset="1" stop-color="${o.bgColor2}"/></linearGradient></defs>` +
    `<rect width="100%" height="100%" fill="url(#g)"/><g opacity="${o.opacity}" fill="${o.lineColor}" stroke="${o.lineColor}">` +
    `<path d="${lines}" fill="none" stroke-width="${o.stroke}" stroke-linecap="round"/><path d="${dots}" stroke="none"/></g></svg>`;
}
