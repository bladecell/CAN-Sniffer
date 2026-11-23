// Contour-line background generator. Pure TS: seeded fBm noise sampled on a grid,
// marching squares per level, output as SVG path data. No WASM needed at this size.
export type ContourOpts = {
  width: number; height: number; seed: number;
  phase: number;    // continuous morph: fractional part blends two noise layers
  scale: number;    // feature size in px
  octaves: number;  // 1..5
  levels: number;   // number of iso-lines
  lo: number; hi: number; // fixed noise range the levels span (not grid min/max, so the pattern never depends on canvas size)
  cell: number;     // grid cell px (smaller = smoother, slower)
  stroke: number; lineColor: string; bgColor: string; lineOpacity: number;
  gradient: boolean; bgColor2: string;
};

const hash = (x: number, y: number, s: number) => {
  let h = Math.imul(x, 374761393) ^ Math.imul(y, 668265263) ^ Math.imul(s, 2147483647);
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967295;
};
const fade = (t: number) => t * t * t * (t * (t * 6 - 15) + 10); // quintic: C2, no creases at cell borders
function vnoise(x: number, y: number, s: number) {
  const xi = Math.floor(x), yi = Math.floor(y), u = fade(x - xi), v = fade(y - yi);
  const a = hash(xi, yi, s), b = hash(xi + 1, yi, s), c = hash(xi, yi + 1, s), d = hash(xi + 1, yi + 1, s);
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
}
function fbm(x: number, y: number, o: ContourOpts) {
  let sum = 0, amp = 1, norm = 0, f = 1;
  const z = Math.floor(o.phase), t = fade(o.phase - z);
  for (let i = 0; i < o.octaves; i++, amp *= 0.35, f *= 2) {
    const s = o.seed + i * 1000 + z, n0 = vnoise(x * f, y * f, s);
    sum += amp * (n0 + (vnoise(x * f, y * f, s + 1) - n0) * t); norm += amp;
  }
  return sum / norm;
}

type Pt = [number, number];

// Closed loops and open chains of iso-line points, joined through shared grid-edge ids.
export function smoothPath(pts: Pt[], closed: boolean, r: (n: number) => number): string {
  const n = pts.length;
  if (n < 3) return '';
  const mid = (a: Pt, b: Pt) => `${r((a[0] + b[0]) / 2)} ${r((a[1] + b[1]) / 2)}`;
  if (closed) {
    let d = `M${mid(pts[n - 1], pts[0])}`;
    for (let k = 0; k < n; k++) d += `Q${r(pts[k][0])} ${r(pts[k][1])} ${mid(pts[k], pts[(k + 1) % n])}`;
    return d + 'Z';
  }
  let d = `M${r(pts[0][0])} ${r(pts[0][1])}`;
  for (let k = 1; k < n - 1; k++) d += `Q${r(pts[k][0])} ${r(pts[k][1])} ${mid(pts[k], pts[k + 1])}`;
  return d + `L${r(pts[n - 1][0])} ${r(pts[n - 1][1])}`;
}

export function contourPath(o: ContourOpts): string {
  const cols = Math.ceil(o.width / o.cell) + 1, rows = Math.ceil(o.height / o.cell) + 1;
  const g = new Float32Array(cols * rows);
  for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) g[j * cols + i] = fbm(i * o.cell / o.scale, j * o.cell / o.scale, o);
  const r = (n: number) => Math.round(n * 10) / 10;
  // Two 3x3 blur passes remove grid-scale kinks and narrow cusps before contouring.
  let bl = g;
  for (let pass = 0; pass < 2; pass++) {
    const src = bl, dst = new Float32Array(g.length);
    for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
      let sum = 0, n = 0;
      for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        const x = i + di, y = j + dj;
        if (x >= 0 && x < cols && y >= 0 && y < rows) { sum += src[y * cols + x]; n++; }
      }
      dst[j * cols + i] = sum / n;
    }
    bl = dst;
  }
  const at = (i: number, j: number) => bl[j * cols + i];
  let d = '';
  for (let l = 1; l <= o.levels; l++) {
    const t = o.lo + ((o.hi - o.lo) * l) / (o.levels + 1);
    const pos = new Map<number, Pt>();
    const adj = new Map<number, number[]>();
    // Edge ids: horizontal (i,j)-(i+1,j) = 2*(j*cols+i); vertical (i,j)-(i,j+1) = 2*(j*cols+i)+1.
    const hId = (i: number, j: number) => 2 * (j * cols + i), vId = (i: number, j: number) => 2 * (j * cols + i) + 1;
    const hPt = (i: number, j: number): Pt => { const a = at(i, j); return [(i + (t - a) / (at(i + 1, j) - a)) * o.cell, j * o.cell]; };
    const vPt = (i: number, j: number): Pt => { const a = at(i, j); return [i * o.cell, (j + (t - a) / (at(i, j + 1) - a)) * o.cell]; };
    const link = (a: number, pa: () => Pt, b: number, pb: () => Pt) => {
      if (!pos.has(a)) pos.set(a, pa());
      if (!pos.has(b)) pos.set(b, pb());
      (adj.get(a) ?? adj.set(a, []).get(a)!).push(b);
      (adj.get(b) ?? adj.set(b, []).get(b)!).push(a);
    };
    for (let j = 0; j < rows - 1; j++) for (let i = 0; i < cols - 1; i++) {
      const code = (at(i, j) > t ? 8 : 0) | (at(i + 1, j) > t ? 4 : 0) | (at(i + 1, j + 1) > t ? 2 : 0) | (at(i, j + 1) > t ? 1 : 0);
      if (code === 0 || code === 15) continue;
      const E = [ // top, right, bottom, left
        [hId(i, j), () => hPt(i, j)], [vId(i + 1, j), () => vPt(i + 1, j)],
        [hId(i, j + 1), () => hPt(i, j + 1)], [vId(i, j), () => vPt(i, j)],
      ] as const;
      const T: Record<number, number[][]> = {
        1: [[3, 2]], 2: [[2, 1]], 3: [[3, 1]], 4: [[0, 1]], 5: [[0, 3], [2, 1]], 6: [[0, 2]], 7: [[0, 3]],
        8: [[0, 3]], 9: [[0, 2]], 10: [[0, 1], [3, 2]], 11: [[0, 1]], 12: [[3, 1]], 13: [[2, 1]], 14: [[3, 2]],
      };
      for (const [a, b] of T[code]) link(E[a][0], E[a][1], E[b][0], E[b][1]);
    }
    const seen = new Set<number>();
    const trace = (start: number) => {
      const ids = [start];
      seen.add(start);
      let prev = -1, cur = start, closed = false;
      for (;;) {
        const next = adj.get(cur)!.find((n) => n !== prev && !seen.has(n));
        if (next === undefined) { closed = ids.length > 2 && adj.get(cur)!.includes(start) && cur !== start && adj.get(start)!.length === 2; break; }
        seen.add(next); ids.push(next); prev = cur; cur = next;
      }
      d += smoothPath(ids.map((id) => pos.get(id)!), closed, r);
    };
    for (const [id, nb] of adj) if (nb.length === 1 && !seen.has(id)) trace(id);
    for (const id of adj.keys()) if (!seen.has(id)) trace(id);
  }
  return d;
}

// Background rect/gradient plus the given stroked path, shared by every pattern.
export function svgFrame(o: ContourOpts, d: string, extra = ''): string {
  const bg = o.gradient
    ? `<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${o.bgColor}"/><stop offset="1" stop-color="${o.bgColor2}"/></linearGradient></defs><rect width="100%" height="100%" fill="url(#g)"/>`
    : `<rect width="100%" height="100%" fill="${o.bgColor}"/>`;
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">${bg}<path d="${d}" fill="none" stroke="${o.lineColor}" stroke-opacity="${o.lineOpacity}" stroke-width="${o.stroke}" stroke-linecap="round" stroke-linejoin="round"${extra}/></svg>`;
}

export const contourSvg = (o: ContourOpts) => svgFrame(o, contourPath(o));
