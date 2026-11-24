// Reads a recorder trip CSV (docs/sd-card.md, Trip files) into per-PID series for the telemetry plots.
// The file comes off an SD card and a browser fetch, so it is treated as hostile: size, rows and PIDs are capped,
// a row that does not match the format exactly is skipped and counted, and nothing throws on bad content.

/** A full trip file is at most 8 MiB (rotation); the margin covers a row that straddles the limit. */
export const TRIP_MAX_BYTES = 9 * 1024 * 1024;
export const TRIP_MAX_ROWS = 600_000;
export const TRIP_MAX_PIDS = 64;

const HEADER = "t_ms,pid,value,valid";
/** t_ms is a 32-bit counter. */
const MAX_T = 0xffffffff;
const UINT = /^\d{1,10}$/;
const NUMBER = /^[+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?$/i;

export interface TripSeries {
  pid: number;
  /** Device ms since boot, non-decreasing. */
  t: number[];
  /** null marks a gap: an invalid sample or an empty value. */
  v: (number | null)[];
}

export interface TripLimits {
  maxRows?: number;
  maxPids?: number;
}

export type TripParse =
  | { ok: true; series: TripSeries[]; startMs: number; endMs: number; rows: number; skipped: number; truncated: boolean }
  | { ok: false; error: "empty" | "bad-header" | "no-data" };

export function parseTripCsv(text: string, limits: TripLimits = {}): TripParse {
  const maxRows = limits.maxRows ?? TRIP_MAX_ROWS;
  const maxPids = limits.maxPids ?? TRIP_MAX_PIDS;
  if (text.length === 0) return { ok: false, error: "empty" };
  const start = text.charCodeAt(0) === 0xfeff ? 1 : 0;
  let pos = text.indexOf("\n", start);
  const head = (pos < 0 ? text.slice(start) : text.slice(start, pos)).trim();
  if (head !== HEADER) return { ok: false, error: "bad-header" };

  const byPid = new Map<number, TripSeries>();
  let rows = 0;
  let skipped = 0;
  let truncated = false;
  while (pos >= 0 && pos < text.length) {
    const from = pos + 1;
    let end = text.indexOf("\n", from);
    if (end < 0) end = text.length;
    const line = text.slice(from, end).trim();
    pos = end < text.length ? end : -1;
    if (line === "") continue;
    if (rows >= maxRows) {
      truncated = true;
      break;
    }
    const f = line.split(",");
    const t = f.length === 4 && UINT.test(f[0]) ? Number(f[0]) : -1;
    const pid = UINT.test(f[1] ?? "") ? Number(f[1]) : -1;
    const validFlag = f[3];
    const textValue = f[2];
    if (t < 0 || t > MAX_T || pid < 0 || pid > 65535 || (validFlag !== "0" && validFlag !== "1") || (textValue !== "" && !NUMBER.test(textValue))) {
      skipped++;
      continue;
    }
    const number = textValue === "" ? Number.NaN : Number(textValue);
    const value = validFlag === "1" && Number.isFinite(number) ? number : null;
    let series = byPid.get(pid);
    if (!series) {
      if (byPid.size >= maxPids) {
        skipped++;
        truncated = true;
        continue;
      }
      series = { pid, t: [], v: [] };
      byPid.set(pid, series);
    }
    const last = series.t.length - 1;
    if (last >= 0 && t < series.t[last]) {
      skipped++;
      continue;
    }
    rows++;
    if (value === null && last >= 0 && series.v[last] === null) continue;
    series.t.push(t);
    series.v.push(value);
  }
  if (rows === 0) return { ok: false, error: "no-data" };
  const series = [...byPid.values()].filter((s) => s.t.length > 0);
  let startMs = Infinity;
  let endMs = -Infinity;
  for (const s of series) {
    startMs = Math.min(startMs, s.t[0]);
    endMs = Math.max(endMs, s.t[s.t.length - 1]);
  }
  return { ok: true, series, startMs, endMs, rows, skipped, truncated };
}

/** Reads a response body as text, refusing more than maxBytes (the header is not trusted: the stream is counted). */
export async function readTextLimited(response: Response, maxBytes = TRIP_MAX_BYTES): Promise<string> {
  const tooLarge = () => new Error("Trip file too large");
  const declared = Number(response.headers.get("Content-Length"));
  if (Number.isFinite(declared) && declared > maxBytes) throw tooLarge();
  if (!response.body) return "";
  const reader = response.body.getReader();
  const chunks: Uint8Array[] = [];
  let total = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    total += value.byteLength;
    if (total > maxBytes) {
      void reader.cancel();
      throw tooLarge();
    }
    chunks.push(value);
  }
  const all = new Uint8Array(total);
  let offset = 0;
  for (const c of chunks) {
    all.set(c, offset);
    offset += c.byteLength;
  }
  return new TextDecoder().decode(all);
}
