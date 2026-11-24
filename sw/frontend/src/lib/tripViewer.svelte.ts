import { firmwareApi } from "./firmwareApi.svelte";
import { isTripCsvName, sidecarName } from "./recorderView";
import { createChartBuffer, type ChartBuffer } from "./chartBuffer";
import type { SeriesSource } from "./telemetryChart/feed.svelte";
import { tripBuffers } from "./telemetryChart/tripBuffers";
import { parseTripCsv, readTextLimited } from "./tripCsv";
import { parseSidecarPids, SIDECAR_MAX_BYTES, type SidecarPid } from "./tripPids";

const TRIPS_DIR = "logs/trips";
const EMPTY: ChartBuffer = createChartBuffer(1);

export interface LoadedTrip {
  name: string;
  startMs: number;
  endMs: number;
  /** PIDs with samples, ascending. */
  pids: number[];
  rows: number;
  skipped: number;
  truncated: boolean;
  /** Names and units from the trip sidecar; null when it is missing or unusable. */
  sidecar: Map<number, SidecarPid> | null;
}

const PARSE_ERRORS = {
  empty: "The trip file is empty.",
  "bad-header": "This is not a recorder trip file (header is not t_ms,pid,value,valid).",
  "no-data": "The trip file has no readable rows.",
} as const;

/** A missing, oversized or malformed sidecar is not an error: the trip plays with the live PID names. */
async function loadSidecar(name: string): Promise<Map<number, SidecarPid> | null> {
  try {
    const response = await firmwareApi.downloadSdFile(`${TRIPS_DIR}/${sidecarName(name)}`);
    if (!response.ok) return null;
    return parseSidecarPids(await readTextLimited(response, SIDECAR_MAX_BYTES));
  } catch {
    return null;
  }
}

/** The trip shown in the Telemetry Recording tab: one CSV in memory at a time, and the Recorder page's request to open one. */
class TripViewer {
  /** Set by the Recorder page; the Telemetry page opens it and clears it. */
  requested = $state<string | null>(null);
  trip = $state.raw<LoadedTrip | null>(null);
  loading = $state(false);
  error = $state<string | null>(null);
  #buffers = new Map<number, ChartBuffer>();
  #token = 0;

  /** Samples for the plots. Resubscribe by remounting the plots when `trip` changes. */
  source: SeriesSource = {
    live: false,
    subscribe: (pid, listener) => {
      listener(this.#buffers.get(pid) ?? EMPTY);
      return () => {};
    },
  };

  request(name: string) {
    this.requested = name;
  }

  async load(name: string) {
    const token = ++this.#token;
    this.loading = true;
    this.error = null;
    try {
      if (!isTripCsvName(name)) throw new Error("Not a trip file");
      const response = await firmwareApi.downloadSdFile(`${TRIPS_DIR}/${name}`);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const parsed = parseTripCsv(await readTextLimited(response));
      if (token !== this.#token) return;
      if (!parsed.ok) throw new Error(PARSE_ERRORS[parsed.error]);
      const sidecar = await loadSidecar(name);
      if (token !== this.#token) return;
      this.#buffers = tripBuffers(parsed.series);
      this.trip = {
        name,
        startMs: parsed.startMs,
        endMs: parsed.endMs,
        pids: parsed.series.map((s) => s.pid).sort((a, b) => a - b),
        rows: parsed.rows,
        skipped: parsed.skipped,
        truncated: parsed.truncated,
        sidecar,
      };
    } catch (e) {
      if (token !== this.#token) return;
      this.#buffers = new Map();
      this.trip = null;
      this.error = e instanceof Error ? e.message : String(e);
    } finally {
      if (token === this.#token) this.loading = false;
    }
  }
}

export const tripViewer = new TripViewer();
