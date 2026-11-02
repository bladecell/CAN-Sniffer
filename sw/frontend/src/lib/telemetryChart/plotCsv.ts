// CSV export of one plot's buffered samples. The first four columns and the value format are the recorder's trip CSV
// (docs/sd-card.md). Differences: t_ms is device ms since boot (the clock the chart's time axis is drawn on), rows are
// not thinned, and name and unit follow because a download has no sidecar file. Missing, NaN and infinite values are
// written empty with valid=0, and text fields that spreadsheets would run as formulas get an apostrophe.

export interface PlotCsvSeries {
  pid: number;
  name: string;
  unit: string;
  t: readonly number[];
  v: readonly (number | null)[];
}

/** Inclusive range in device ms. */
export interface PlotCsvDomain {
  fromT: number;
  toT: number;
}

export const PLOT_CSV_HEADER = "t_ms,pid,value,valid,name,unit";

const FORMULA_START = /^(?:\s*[=+\-@]|[\t\r])/;
const NEEDS_QUOTES = /[",\r\n]/;

/** RFC 4180 quoting, with an apostrophe first when the field would start a spreadsheet formula. */
export function csvField(text: string): string {
  const safe = FORMULA_START.test(text) ? `'${text}` : text;
  return NEEDS_QUOTES.test(safe) ? `"${safe.replace(/"/g, '""')}"` : safe;
}

const trimZeros = (s: string) => (s.includes(".") ? s.replace(/\.?0+$/, "") : s);

/** C's %.8g for a finite number: eight significant digits, trailing zeros dropped, exponent form below 1e-4 and from 1e8. */
export function formatCsvValue(v: number): string {
  if (v === 0) return "0";
  const [mantissa, exp] = v.toExponential(7).split("e");
  const e = Number(exp);
  if (e < -4 || e >= 8) {
    const sign = e < 0 ? "-" : "+";
    return `${trimZeros(mantissa)}e${sign}${String(Math.abs(e)).padStart(2, "0")}`;
  }
  return trimZeros(v.toFixed(7 - e));
}

/** Rows of all series merged by t_ms; rows with the same time keep the series order. */
export function buildPlotCsv(series: readonly PlotCsvSeries[], domain: PlotCsvDomain | null): string {
  const rows: { t: number; s: number; line: string }[] = [];
  series.forEach((s, si) => {
    const n = Math.min(s.t.length, s.v.length);
    for (let i = 0; i < n; i++) {
      const t = s.t[i];
      if (!Number.isFinite(t)) continue;
      if (domain && (t < domain.fromT || t > domain.toT)) continue;
      const v = s.v[i];
      const value = typeof v === "number" && Number.isFinite(v) ? formatCsvValue(v) : null;
      const cells = [Math.round(t), s.pid, value ?? "", value === null ? 0 : 1, csvField(s.name), csvField(s.unit)];
      rows.push({ t, s: si, line: cells.join(",") });
    }
  });
  rows.sort((a, b) => a.t - b.t || a.s - b.s);
  return [PLOT_CSV_HEADER, ...rows.map((r) => r.line)].join("\n") + "\n";
}

const pad = (n: number) => String(n).padStart(2, "0");

/** can-sniffer-plot-<n>-<YYYYMMDD-HHMMSS>.csv in local time. */
export function plotCsvFileName(plot: number, now: Date): string {
  const day = `${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}`;
  const time = `${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
  return `can-sniffer-plot-${plot}-${day}-${time}.csv`;
}

/** Saves the text through a temporary object URL, as recordingStore does for trip files. */
export function downloadCsv(text: string, fileName: string): void {
  const url = URL.createObjectURL(new Blob([text], { type: "text/csv;charset=utf-8" }));
  const link = document.createElement("a");
  link.href = url;
  link.download = fileName;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1_000);
}
