// "Circuit board" background: random-walk traces on a grid that move in 45 degree steps, with ringed pads at both ends.
// Each seed is a different board, so the slider steps between boards instead of morphing. Absolute coordinates only.

export type CircuitOpts = {
  width: number; height: number; seed: number;
  grid: number; // cell size in px
  bgColor: string; bgColor2: string; lineColor: string; opacity: number;
};

const DIRS = [[-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0]] as const;
const MAX_POINTS = 20, MIN_POINTS = 3, STRAIGHT_BIAS = 10;
const r1 = (n: number) => Math.round(n * 10) / 10;

function mulberry32(seed: number) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return (((t ^ (t >>> 14)) >>> 0)) / 4294967296;
  };
}

/** Traces as lists of grid cells, each cell used at most once and no two diagonal steps crossing. */
export function traces(cols: number, rows: number, seed: number): [number, number][][] {
  const rand = mulberry32(seed);
  const free = Array.from({ length: cols * rows }, (_, i) => i);
  const taken = new Uint8Array(cols * rows);
  const diag = new Map<number, number>(); // cell corner -> bitmask of the diagonals used ("\" = 1, "/" = 2)
  const take = (cell: number) => {
    taken[cell] = 1;
    const i = free.indexOf(cell);
    free[i] = free[free.length - 1];
    free.pop();
  };
  const out: [number, number][][] = [];
  while (free.length > 0) {
    const start = free[Math.floor(rand() * free.length)];
    take(start);
    let x = start % cols, y = Math.floor(start / cols);
    let dir = Math.floor(rand() * 8);
    const cells: [number, number][] = [[x, y]];
    while (cells.length < MAX_POINTS) {
      const turns = rand() > 0.5 ? [0, 1, -1] : [0, -1, 1];
      let moved = false;
      while (turns.length > 0 && !moved) {
        const turn = turns.splice(Math.floor(Math.pow(rand(), STRAIGHT_BIAS) * turns.length), 1)[0];
        const nd = (dir + turn + 8) % 8, [dx, dy] = DIRS[nd], nx = x + dx, ny = y + dy;
        if (nx < 0 || nx >= cols || ny < 0 || ny >= rows || taken[ny * cols + nx]) continue;
        if (dx !== 0 && dy !== 0) {
          const corner = Math.min(y, ny) * cols + Math.min(x, nx), bit = dx * dy > 0 ? 1 : 2;
          if ((diag.get(corner) ?? 0) & (3 - bit)) continue; // would cross a diagonal already drawn
          diag.set(corner, (diag.get(corner) ?? 0) | bit);
        }
        take(ny * cols + nx);
        x = nx; y = ny; dir = nd; cells.push([x, y]);
        moved = true;
      }
      if (!moved) break;
    }
    if (cells.length >= MIN_POINTS) out.push(cells);
  }
  return out;
}

/** Corner points only: runs in one direction collapse to their two ends. */
function corners(cells: [number, number][]): [number, number][] {
  const pts = [cells[0]];
  for (let i = 1; i < cells.length - 1; i++) {
    const [a, b, c] = [cells[i - 1], cells[i], cells[i + 1]];
    if (b[0] - a[0] !== c[0] - b[0] || b[1] - a[1] !== c[1] - b[1]) pts.push(b);
  }
  pts.push(cells[cells.length - 1]);
  return pts;
}

export function circuitBoardSvg(o: CircuitOpts): string {
  const g = o.grid, ring = g / 4, hole = g / 8;
  const cols = Math.ceil(o.width / g), rows = Math.ceil(o.height / g);
  const px = (c: number) => (c + 0.5) * g;
  let lines = "", pads = "";
  const circle = (cx: number, cy: number, r: number) => `M${r1(cx - r)} ${r1(cy)}a${r1(r)} ${r1(r)} 0 1 0 ${r1(2 * r)} 0a${r1(r)} ${r1(r)} 0 1 0 ${r1(-2 * r)} 0z`;
  for (const cells of traces(cols, rows, o.seed)) {
    const pts = corners(cells).map(([x, y]) => [px(x), px(y)] as [number, number]);
    const last = pts.length - 1;
    // Stop each end at its pad ring so the hole stays empty.
    const pull = (from: [number, number], to: [number, number]): [number, number] => {
      const len = Math.hypot(to[0] - from[0], to[1] - from[1]);
      return [from[0] + ((to[0] - from[0]) / len) * ring, from[1] + ((to[1] - from[1]) / len) * ring];
    };
    const path = pts.slice();
    path[0] = pull(pts[0], pts[1]);
    path[last] = pull(pts[last], pts[last - 1]);
    lines += path.map((p, i) => `${i === 0 ? "M" : "L"}${r1(p[0])} ${r1(p[1])}`).join("");
    for (const p of [pts[0], pts[last]]) pads += circle(p[0], p[1], ring) + circle(p[0], p[1], hole);
  }
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${o.width}" height="${o.height}" viewBox="0 0 ${o.width} ${o.height}">` +
    `<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="${o.bgColor}"/><stop offset="1" stop-color="${o.bgColor2}"/></linearGradient></defs>` +
    `<rect width="100%" height="100%" fill="url(#g)"/><g opacity="${o.opacity}" fill="${o.lineColor}" stroke="${o.lineColor}">` +
    `<path d="${lines}" fill="none" stroke-width="${r1(g / 6)}" stroke-linejoin="round"/><path d="${pads}" fill-rule="evenodd" stroke="none"/></g></svg>`;
}
