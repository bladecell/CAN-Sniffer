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
/** An id stays active for a few of its own periods, so a slow signal (one frame every 4 s) does not flip to silent between frames. */
export const activeLimitMs = (gapMs: number) => Math.max(ACTIVE_MS, 2.5 * gapMs);
/** An ECU id silent for this long leaves the ECU list. */
export const ECU_AGE_MS = 60_000;
/** A bit that flipped this recently (ms, client clock) shows as changing now in the bit view. */
export const FLIP_WINDOW_MS = 1000;

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
  /** Parsed values of the last frame. */
  parsed: string;
  /** Client clock (ms) of the last frame. */
  seenMs: number;
  /** Client-clock gap (ms) between the last two frames, 0 until a second frame arrives. */
  gapMs: number;
  /** Frames whose data differed from the previous frame of this id. */
  changes: number;
  /** Bit j of byte i is set once it has flipped. */
  everBits: number[];
  /** Client clock (ms) of the last flip of bit j of byte i at index i * 8 + j, -Infinity until it flips. */
  flipMs: Float64Array;
};

/** Bit view masks at `nowMs`: `ever` for every bit that flipped, `recent` for those that flipped within FLIP_WINDOW_MS. */
export function bitStats(e: Pick<IdEntry, "everBits" | "flipMs">, nowMs: number): { ever: number[]; recent: number[] } {
  const recent = [0, 0, 0, 0, 0, 0, 0, 0];
  for (let k = 0; k < 64; k++) if (nowMs - e.flipMs[k] < FLIP_WINDOW_MS) recent[k >> 3] |= 1 << (k & 7);
  return { ever: e.everBits.slice(), recent };
}

/** Bit i of the result is set when byte i differs. A DLC change counts every byte past the shorter frame. */
export function changedMask(prev: ArrayLike<number>, cur: ArrayLike<number>): number {
  let mask = 0;
  const n = Math.max(prev.length, cur.length);
  for (let i = 0; i < n && i < 8; i++) if (prev[i] !== cur[i]) mask |= 1 << i;
  return mask;
}

/** Records the bits that differ between two frames of one id. A missing byte counts as 0, like changedMask. */
function markFlips(e: IdEntry, prev: ArrayLike<number>, cur: ArrayLike<number>, nowMs: number) {
  for (let i = 0; i < 8; i++) {
    const diff = (prev[i] ?? 0) ^ (cur[i] ?? 0);
    if (diff === 0) continue;
    e.everBits[i] |= diff;
    for (let j = 0; j < 8; j++) if (diff & (1 << j)) e.flipMs[i * 8 + j] = nowMs;
  }
}

/** Changed mask per frame of one id, newest first: each frame against the next older one. The oldest has nothing to compare with, so 0. */
export function changedMasks(newestFirst: readonly { data: ArrayLike<number> }[]): number[] {
  return newestFirst.map((f, i) => (i + 1 < newestFirst.length ? changedMask(newestFirst[i + 1].data, f.data) : 0));
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
  /** Latest value text per formula rule id ("invalid" for a failed formula). */
  ruleValues = new Map<string, string>();
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
    for (const v of result.values) this.ruleValues.set(v.ruleId, v.text);
    const key = idKey(f.id, f.ext);
    let entry = this.ids.get(key);
    if (!entry && this.ids.size < MAX_IDS) {
      entry = { id: f.id, ext: f.ext, count: 0, window: 0, hz: 0, last: f, recent: [], changed: 0, hidden: false, name: "", parsed: "", seenMs: nowMs, gapMs: 0, changes: 0, everBits: [0, 0, 0, 0, 0, 0, 0, 0], flipMs: new Float64Array(64).fill(-Infinity) };
      this.ids.set(key, entry);
    }
    if (!entry) this.untracked++;
    let changed = 0;
    if (entry) {
      changed = entry.count > 0 ? changedMask(entry.last.data, f.data) : 0;
      if (changed) {
        entry.changes++;
        markFlips(entry, entry.last.data, f.data, nowMs);
      }
      entry.count++;
      entry.window++;
      entry.last = f;
      entry.recent.push(f);
      if (entry.recent.length > RECENT) entry.recent.shift();
      entry.changed = changed;
      entry.hidden = result.hidden;
      entry.name = result.name || entry.name;
      entry.parsed = result.parsed;
      if (entry.count > 1) entry.gapMs = nowMs - entry.seenMs;
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
    this.ruleValues.clear();
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
export function logRows(model: CanModel, filter: TableFilter, keep: (f: CanFrame) => boolean = () => true): LogRow[] {
  const out: LogRow[] = [];
  for (let i = model.log.length - 1; i >= 0 && out.length < TABLE_ROWS; i--) {
    const row = model.log[i];
    if (!row.hidden && passesFilter(row, filter) && keep(row)) out.push(row);
  }
  return out;
}

/** One row of the by-ID table: the last frame of an id, with its rate and changed bytes. */
export type IdView = CanFrame & { key: number; count: number; hz: number; changed: number; parsed: string; name: string };

/** Sorted by id, shown ids only, capped at TABLE_ROWS. */
export function byIdRows(model: CanModel, filter: TableFilter, keep: (f: CanFrame) => boolean = () => true): IdView[] {
  const list: IdView[] = [];
  for (const [key, e] of model.ids) {
    if (e.hidden || !passesFilter(e, filter) || !keep(e.last)) continue;
    list.push({ ...e.last, key, count: e.count, hz: e.hz, changed: e.changed, parsed: e.parsed, name: e.name });
  }
  list.sort((a, b) => a.id - b.id || Number(a.ext) - Number(b.ext));
  return list.slice(0, TABLE_ROWS);
}

/** The newest frames of every id: what a draft rule is previewed against. */
export function* recentFrames(model: CanModel): Generator<CanFrame> {
  for (const e of model.ids.values()) yield* e.recent;
}

export type EcuRow = { id: number; ext: boolean; request: number; dlc: number; hz: number; seenMs: number };

/** Passive: the ids that follow the OBD response pattern, seen within ECU_AGE_MS. Nothing is sent. */
export function ecuRows(model: CanModel, nowMs: number): EcuRow[] {
  const out: EcuRow[] = [];
  for (const e of model.ids.values()) {
    const request = ecuRequestId(e.id, e.ext);
    if (request === null || nowMs - e.seenMs > ECU_AGE_MS) continue;
    out.push({ id: e.id, ext: e.ext, request, dlc: e.last.data.length, hz: e.hz, seenMs: e.seenMs });
  }
  return out.sort((a, b) => a.id - b.id || Number(a.ext) - Number(b.ext));
}

