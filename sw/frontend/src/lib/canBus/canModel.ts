// The capture model: every frame from the raw stream goes in, whether the table shows it or not.
// Counters keep running while the view is paused (the page copies a snapshot, not this object).
import type { RawBatch } from "../wsFrames";
import { compileRules, type CompiledRules, type FrameLike } from "./canRules";
import { idKey, isObdId, ecuRequestId } from "./canIds";

/** Rows shown in the table (DESIGN: about 500). The log keeps up to twice that between trims. */
export const TABLE_ROWS = 500;
export const MAX_IDS = 1024;
const LOG_KEEP = TABLE_ROWS * 2;
const RECENT = 5;
const FPS_HISTORY = 60;
/** An id seen this recently (ms, client clock) counts as active. */
export const ACTIVE_MS = 3000;

export type CanFrame = FrameLike & { t: number; rtr: boolean; tx: boolean; data: Uint8Array };
export type LogRow = CanFrame & { changed: number; parsed: string; hidden: boolean };

export type IdEntry = {
  id: number;
  ext: boolean;
  count: number;
  /** Frames since the last tick, for the rate. */
  window: number;
  hz: number;
  last: CanFrame;
  /** Newest last, at most RECENT frames. */
  recent: CanFrame[];
  /** Bit i set: byte i differs from the previous frame of this id. */
  changed: number;
  hidden: boolean;
  name: string;
  /** Client clock (ms) of the last frame. */
  seenMs: number;
};

/** Bit i of the result is set when byte i differs. A DLC change counts every byte past the shorter frame. */
export function changedMask(prev: ArrayLike<number>, cur: ArrayLike<number>): number {
  let mask = 0;
  const n = Math.max(prev.length, cur.length);
  for (let i = 0; i < n && i < 8; i++) if (prev[i] !== cur[i]) mask |= 1 << i;
  return mask;
}

export class CanModel {
  readonly ids = new Map<number, IdEntry>();
  /** Frames from ids past MAX_IDS: counted and logged, not listed. */
  untracked = 0;
  /** Every frame, shown or hidden by a filter rule. Trimmed to LOG_KEEP. */
  log: LogRow[] = [];
  frames = 0;
  /** Frames per second, one entry per tick, newest last. */
  fps: number[] = [];
  /** Session start (device time, µs), the zero of the time column. */
  startUs: number | null = null;
  private rules: CompiledRules = compileRules([]);
  private anchorLow: number | null = null;
  private anchorAbs = 0;
  private framesAtTick = 0;

  setRules(rules: CompiledRules) {
    this.rules = rules;
  }

  /** Device time is the low 32 bits of µs, so each batch is unwrapped against the previous one. */
  private unwrap(low: number): number {
    if (this.anchorLow === null) this.anchorAbs = low;
    else this.anchorAbs += (low - this.anchorLow) | 0;
    this.anchorLow = low;
    return this.anchorAbs;
  }

  ingest(batch: RawBatch, nowMs: number) {
    let t = this.unwrap(batch.baseTs);
    for (const r of batch.records) {
      t += r.dt;
      this.add({ t, id: r.id, ext: r.ext, rtr: r.rtr, tx: r.tx, data: r.data }, nowMs);
    }
  }

  private add(f: CanFrame, nowMs: number) {
    this.startUs ??= f.t;
    this.frames++;
    const result = this.rules.apply(f);
    const key = idKey(f.id, f.ext);
    let entry = this.ids.get(key);
    if (!entry && this.ids.size < MAX_IDS) {
      entry = { id: f.id, ext: f.ext, count: 0, window: 0, hz: 0, last: f, recent: [], changed: 0, hidden: false, name: "", seenMs: nowMs };
      this.ids.set(key, entry);
    }
    if (!entry) this.untracked++;
    let changed = 0;
    if (entry) {
      changed = entry.count > 0 ? changedMask(entry.last.data, f.data) : 0;
      entry.count++;
      entry.window++;
      entry.last = f;
      entry.recent.push(f);
      if (entry.recent.length > RECENT) entry.recent.shift();
      entry.changed = changed;
      entry.hidden = result.hidden;
      entry.name = result.name || entry.name;
      entry.seenMs = nowMs;
    }
    this.log.push({ ...f, changed, parsed: result.parsed, hidden: result.hidden });
    if (this.log.length > LOG_KEEP) this.log.splice(0, this.log.length - TABLE_ROWS);
  }

  /** Once a second (or at any interval): per-id rate and frames per second. */
  tick(seconds: number) {
    const s = seconds > 0 ? seconds : 1;
    for (const e of this.ids.values()) {
      e.hz = e.window / s;
      e.window = 0;
    }
    this.fps.push((this.frames - this.framesAtTick) / s);
    this.framesAtTick = this.frames;
    if (this.fps.length > FPS_HISTORY) this.fps.splice(0, this.fps.length - FPS_HISTORY);
  }

  reset() {
    this.ids.clear();
    this.untracked = 0;
    this.log = [];
    this.frames = 0;
    this.fps = [];
    this.startUs = null;
    this.anchorLow = null;
    this.anchorAbs = 0;
    this.framesAtTick = 0;
  }
}

export type TableFilter = "all" | "changed" | "obd";

export function passesFilter(row: { id: number; ext: boolean; changed: number }, filter: TableFilter): boolean {
  if (filter === "changed") return row.changed !== 0;
  if (filter === "obd") return isObdId(row.id, row.ext);
  return true;
}

/** Newest first, shown rows only (hidden rows are the plain-filter matches), capped at TABLE_ROWS. */
export function logRows(model: CanModel, filter: TableFilter): LogRow[] {
  const out: LogRow[] = [];
  for (let i = model.log.length - 1; i >= 0 && out.length < TABLE_ROWS; i--) {
    const row = model.log[i];
    if (!row.hidden && passesFilter(row, filter)) out.push(row);
  }
  return out;
}

export type EcuRow = { id: number; ext: boolean; request: number; dlc: number; hz: number; seenMs: number };

/** Passive: the ids that follow the OBD response pattern, from what the bus already carries. */
export function ecuRows(model: CanModel): EcuRow[] {
  const out: EcuRow[] = [];
  for (const e of model.ids.values()) {
    const request = ecuRequestId(e.id, e.ext);
    if (request === null) continue;
    out.push({ id: e.id, ext: e.ext, request, dlc: e.last.data.length, hz: e.hz, seenMs: e.seenMs });
  }
  return out.sort((a, b) => a.id - b.id || Number(a.ext) - Number(b.ext));
}

