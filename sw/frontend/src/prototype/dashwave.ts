// Dashed wave rows. Absolute coordinates only, so the pattern does not depend on the canvas size.
import { smoothPath, svgFrame, type ContourOpts } from './contour';

const rnd = (n: number) => Math.round(n * 10) / 10;
const rand = (s: number) => { const x = Math.sin(s * 127.1 + 311.7) * 43758.5453; return x - Math.floor(x); };

export function dashWaveSvg(o: ContourOpts): string {
  const spacing = o.scale * 0.34, amp = o.scale * 0.28, wl = o.scale * 2.4;
  const rows = Math.ceil(o.height / spacing) + 3, step = Math.max(8, wl / 14);
  const ph = o.phase * 0.9, p1 = rand(o.seed) * 6.28, p2 = rand(o.seed + 1) * 6.28;
  let d = '';
  for (let k = -1; k < rows; k++) {
    const pts: [number, number][] = [];
    for (let x = -step; x <= o.width + step; x += step) {
      const y = k * spacing
        + amp * Math.sin((x / wl) * 6.283 + p1 + k * 0.32 + ph)
        + amp * 0.45 * Math.sin((x / (wl * 0.47)) * 6.283 + p2 - k * 0.2 - ph * 1.7);
      pts.push([x, y]);
    }
    d += smoothPath(pts, false, rnd);
  }
  const dash = Math.round(o.stroke * 5), gap = Math.round(o.stroke * 6);
  return svgFrame(o, d, ` stroke-dasharray="${dash} ${gap}"`);
}
