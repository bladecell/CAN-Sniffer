import { createChartBuffer, type ChartBuffer } from "../chartBuffer";
import type { TripSeries } from "../tripCsv";

/** One chart buffer per PID, sized to its samples (the parser caps the total), so the whole trip stays readable. */
export function tripBuffers(series: readonly TripSeries[]): Map<number, ChartBuffer> {
  const buffers = new Map<number, ChartBuffer>();
  for (const s of series) {
    if (s.t.length === 0) continue;
    const buffer = createChartBuffer(s.t.length);
    for (let i = 0; i < s.t.length; i++) buffer.push(s.t[i], s.v[i] ?? Number.NaN);
    buffers.set(s.pid, buffer);
  }
  return buffers;
}
