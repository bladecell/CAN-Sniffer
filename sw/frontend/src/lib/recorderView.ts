import type { RecorderState, RecorderTrips } from "./types";

/** Firmware refuses a start less than this after the previous start or stop (`kStartCooldownMs`). */
export const START_COOLDOWN_MS = 10_000;

export type TripEntry = RecorderTrips["trips"][number];

export interface TripRow {
  key: string;
  name: string;
  size: number;
  sidecar: boolean;
  live: boolean;
}

export interface LiveTrip {
  state: RecorderState;
  trip: number;
  fileBytes: number;
}

/** Seconds from a 429 `Retry-After`, clamped to the 1..10 s window. Missing or non-numeric means the full window. */
export function retryAfterSeconds(header: string | null): number {
  if (header === null || header.trim() === "") return START_COOLDOWN_MS / 1000;
  const seconds = Math.ceil(Number(header));
  if (!Number.isFinite(seconds)) return START_COOLDOWN_MS / 1000;
  return Math.min(Math.max(seconds, 1), START_COOLDOWN_MS / 1000);
}

/** Whole seconds left before `until` (a `Date.now()`-style clock in ms). Zero once it has passed. */
export function cooldownSeconds(until: number, now: number): number {
  return Math.max(0, Math.ceil((until - now) / 1000));
}

export function tripFileName(index: number): string {
  return `trip_${String(index).padStart(6, "0")}.csv`;
}

const TRIP_CSV = /^trip_\d{6}\.csv$/;

/** Only names the recorder writes (`trip_NNNNNN.csv`). Anything else from the device is refused. */
export function isTripCsvName(name: string): boolean {
  return TRIP_CSV.test(name);
}

/** Drops list entries whose name is not a recorder trip, so no device-supplied path reaches a request. */
export function validTrips<T extends { name: string }>(trips: T[]): T[] {
  return trips.filter((t) => isTripCsvName(t.name));
}

export function sidecarName(csvName: string): string {
  return csvName.replace(/\.csv$/, ".json");
}

/** kB below one million bytes (never 0), MB with one decimal above. Matches the mockup. */
export function formatBytes(bytes: number): string {
  if (bytes < 1_000_000) return `${Math.max(1, Math.round(bytes / 1_000))} kB`;
  return `${(bytes / 1_000_000).toFixed(1)} MB`;
}

/**
 * The trip list in API order (newest first). While a trip is being written, its row carries the
 * live size from the recorder status; a row for it is added first if the list does not have it yet.
 */
export function tripRows(trips: TripEntry[], live: LiveTrip | null): TripRow[] {
  const writing = live !== null && (live.state === "recording" || live.state === "stopping") && live.trip > 0;
  const liveName = writing ? tripFileName(live.trip) : null;
  const rows: TripRow[] = trips.map((t) => {
    const isLive = t.name === liveName;
    return { key: t.name, name: t.name, size: isLive && live ? live.fileBytes : t.size, sidecar: t.sidecar, live: isLive };
  });
  if (liveName !== null && live && !rows.some((r) => r.live)) {
    rows.unshift({ key: liveName, name: liveName, size: live.fileBytes, sidecar: false, live: true });
  }
  return rows;
}
