// Wide CSV of a trip for spreadsheets: t_ms, then one column per PID, one row per distinct t_ms (docs/sd-card.md).
// Built in the browser from the parsed long-format trip; the card keeps the long file.
import { csvField, formatCsvValue } from "./telemetryChart/plotCsv";
import type { TripSeries } from "./tripCsv";

export interface PidLabel {
  name: string;
  unit: string;
}

const MAX_LABEL = 64;
const MAX_SIDECAR_PIDS = 256;

/** Column header: "name (unit)" from the sidecar, else the PID number. */
function header(pid: number, label: PidLabel | undefined): string {
  if (!label || label.name === "") return String(pid);
  return csvField(label.unit ? `${label.name} (${label.unit})` : label.name);
}

/** Missing samples and gaps (invalid or empty) are empty cells. A repeated t_ms in one PID keeps the later value. */
export function buildWideCsv(series: readonly TripSeries[], labels?: ReadonlyMap<number, PidLabel>): string {
  const times = new Set<number>();
  for (const s of series) for (const t of s.t) times.add(t);
  const sorted = [...times].sort((a, b) => a - b);
  const next = series.map(() => 0);
  const lines = [["t_ms", ...series.map((s) => header(s.pid, labels?.get(s.pid)))].join(",")];
  for (const t of sorted) {
    const cells: string[] = [String(t)];
    series.forEach((s, k) => {
      let cell = "";
      while (next[k] < s.t.length && s.t[next[k]] <= t) {
        if (s.t[next[k]] === t) {
          const v = s.v[next[k]];
          cell = v === null || v === undefined ? "" : formatCsvValue(v);
        }
        next[k]++;
      }
      cells.push(cell);
    });
    lines.push(cells.join(","));
  }
  return lines.join("\n") + "\n";
}

/** Names and units from the trip sidecar JSON. The file is hostile input: anything unexpected is skipped. */
export function parseSidecarNames(text: string): Map<number, PidLabel> {
  const out = new Map<number, PidLabel>();
  let doc: unknown;
  try {
    doc = JSON.parse(text);
  } catch {
    return out;
  }
  const pids = (doc as { pids?: unknown } | null)?.pids;
  if (!Array.isArray(pids)) return out;
  for (const p of pids.slice(0, MAX_SIDECAR_PIDS)) {
    const e = p as { pid?: unknown; name?: unknown; unit?: unknown } | null;
    if (!e || typeof e.pid !== "number" || !Number.isInteger(e.pid) || typeof e.name !== "string") continue;
    out.set(e.pid, { name: e.name.slice(0, MAX_LABEL), unit: typeof e.unit === "string" ? e.unit.slice(0, MAX_LABEL) : "" });
  }
  return out;
}

/** trip_000007.csv -> trip_000007_wide.csv */
export const wideFileName = (csvName: string): string => csvName.replace(/\.csv$/, "_wide.csv");
