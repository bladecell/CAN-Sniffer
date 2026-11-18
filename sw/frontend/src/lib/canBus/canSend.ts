// Send-tab input checks. Hostile input is refused here too: the firmware checks again, and nothing is sent on a failure.

export const SEND_MAX_FRAMES = 32;
export const SEND_MIN_PERIOD_MS = 10;
export const SEND_MAX_PERIOD_MS = 3_600_000;
export const SEND_MAX_REPEAT = 65_535;

export type SendFrame = { id: number; data: string };
export type Checked<T> = { ok: true; value: T } | { ok: false; reason: string };

/** Standard ids only (0 to 0x7FF). The firmware refuses extended frames too. */
export function parseSendId(text: string): Checked<number> {
  const t = text.trim();
  if (!/^[0-9a-fA-F]{1,3}$/.test(t)) return { ok: false, reason: "ID must be 1 to 3 hex digits (7FF at most)" };
  const v = parseInt(t, 16);
  return v <= 0x7ff ? { ok: true, value: v } : { ok: false, reason: "ID must be 7FF or lower (standard ids)" };
}

/** 1 to 8 bytes of hex, spaces optional. Returns the bytes spaced as the firmware reads them: `02 01 0C`. */
export function parseSendData(text: string): Checked<string> {
  const digits = text.replace(/\s+/g, "");
  if (!/^[0-9a-fA-F]+$/.test(digits) || digits.length % 2 !== 0) return { ok: false, reason: "Data must be whole hex bytes" };
  const count = digits.length / 2;
  if (count < 1 || count > 8) return { ok: false, reason: `Data must be 1 to 8 bytes, not ${count}` };
  return { ok: true, value: (digits.match(/../g) ?? []).map((b) => b.toUpperCase()).join(" ") };
}

export function parseIntIn(text: string, min: number, max: number): number | null {
  if (!/^\d+$/.test(text.trim())) return null;
  const v = Number(text.trim());
  return v >= min && v <= max ? v : null;
}

/** Sequence plan body for POST /can/send. `repeat` 0 means until stopped. */
export function planBody(frames: readonly SendFrame[], periodMs: number, repeat: number) {
  return { frames: frames.map((f) => ({ id: f.id, data: f.data })), period_ms: periodMs, repeat };
}

export function idHex(id: number): string {
  return id.toString(16).toUpperCase().padStart(3, "0");
}

/** The list with item `i` swapped with its neighbour `d` away; the same list when that is out of range. */
export function moveAt<T>(list: readonly T[], i: number, d: number): readonly T[] {
  const j = i + d;
  if (i < 0 || i >= list.length || j < 0 || j >= list.length) return list;
  const next = [...list];
  [next[i], next[j]] = [next[j], next[i]];
  return next;
}

/** The list in the order of `keys` (after a drag); the same list unless `keys` names every item once. */
export function reorderBy<T>(list: readonly T[], keyOf: (item: T) => string, keys: readonly string[]): readonly T[] {
  const byKey = new Map(list.map((x) => [keyOf(x), x]));
  const next = keys.flatMap((k) => byKey.get(k) ?? []);
  return next.length === list.length && new Set(keys).size === list.length ? next : list;
}

// Period slider: log scale from 10 ms to 1 h (docs/api.md), the same look as the PID edit interval slider.
export const PERIOD_SNAP = [10, 20, 50, 100, 250, 500, 1000, 5000, 10000, 30000, 60000, 300000, 600000, 3600000];
export const PERIOD_TICKS = [100, 1000, 60000];
const PERIOD_SPAN = Math.log(SEND_MAX_PERIOD_MS / SEND_MIN_PERIOD_MS);
const clamp = (v: number, min: number, max: number) => Math.min(max, Math.max(min, v));

/** Track position (0 to 1) of a period. */
export function periodToPos(ms: number): number {
  return clamp(Math.log(Math.max(ms, 1) / SEND_MIN_PERIOD_MS) / PERIOD_SPAN, 0, 1);
}

/** Period for a track position: rounded to a grain that grows with the value, then snapped to common steps. */
export function posToPeriod(x: number): number {
  let ms = SEND_MIN_PERIOD_MS * Math.exp(clamp(x, 0, 1) * PERIOD_SPAN);
  const grain = ms < 100 ? 1 : ms < 1000 ? 10 : ms < 10000 ? 50 : ms < 60000 ? 500 : ms < 600000 ? 5000 : 60000;
  ms = clamp(Math.round(ms / grain) * grain, SEND_MIN_PERIOD_MS, SEND_MAX_PERIOD_MS);
  return PERIOD_SNAP.find((s) => Math.abs(periodToPos(ms) - periodToPos(s)) < 0.02) ?? ms;
}

const periodStep = (ms: number) => (ms < 100 ? 10 : ms < 1000 ? 50 : ms < 10000 ? 500 : ms < 60000 ? 5000 : ms < 600000 ? 30000 : 300000);

/** The period a key moves to, or undefined when the key does nothing. */
export function periodKey(ms: number, key: string): number | undefined {
  const up = clamp(ms + periodStep(ms), SEND_MIN_PERIOD_MS, SEND_MAX_PERIOD_MS);
  const down = clamp(ms - periodStep(ms), SEND_MIN_PERIOD_MS, SEND_MAX_PERIOD_MS);
  const next: Record<string, number> = {
    ArrowUp: up,
    ArrowRight: up,
    ArrowDown: down,
    ArrowLeft: down,
    PageUp: PERIOD_SNAP.find((s) => s > ms) ?? SEND_MAX_PERIOD_MS,
    PageDown: [...PERIOD_SNAP].reverse().find((s) => s < ms) ?? SEND_MIN_PERIOD_MS,
    Home: SEND_MIN_PERIOD_MS,
    End: SEND_MAX_PERIOD_MS,
  };
  return next[key];
}

export function periodBrief(ms: number): string {
  if (ms >= 3_600_000) return "1 h";
  if (ms >= 60_000) return `${+(ms / 60_000).toFixed(2)} min`;
  if (ms >= 1000) return `${+(ms / 1000).toFixed(2)} s`;
  return `${ms} ms`;
}

// Repeat slider: Once (1) at the left, a log scale to 65535 rounds, Endless (0 on the wire) at the hatched right stop.
export const REPEAT_STOP = 0.12;
export const REPEAT_SNAP = [1, 2, 3, 5, 10, 20, 50, 100, 250, 500, 1000, 5000, 10000, 65535];
export const REPEAT_TICKS = [10, 100, 1000];
const REPEAT_LOG = Math.log(SEND_MAX_REPEAT);

export function repeatToPos(n: number): number {
  return n === 0 ? 1 : Math.min(1 - REPEAT_STOP, ((1 - REPEAT_STOP) * Math.log(Math.max(1, n))) / REPEAT_LOG);
}

export function posToRepeat(x: number): number {
  if (x >= 1 - REPEAT_STOP / 2) return 0;
  let n = Math.exp(Math.min(1, Math.max(0, x) / (1 - REPEAT_STOP)) * REPEAT_LOG);
  const grain = n < 1000 ? (n < 100 ? 1 : 10) : n < 10000 ? 50 : 500;
  n = clamp(Math.round(n / grain) * grain, 1, SEND_MAX_REPEAT);
  return REPEAT_SNAP.find((s) => Math.abs(repeatToPos(n) - repeatToPos(s)) < 0.02) ?? n;
}

const repeatStep = (n: number) => (n < 10 ? 1 : n < 100 ? 5 : n < 1000 ? 50 : n < 10000 ? 500 : 5000);

/** The repeat count a key moves to, or undefined. Up from the top and down from Endless wrap through the stop. */
export function repeatKey(n: number, key: string): number | undefined {
  const up = n === 0 || n >= SEND_MAX_REPEAT ? 0 : Math.min(SEND_MAX_REPEAT, n + repeatStep(n));
  const down = n === 0 ? SEND_MAX_REPEAT : Math.max(1, n - repeatStep(n));
  const next: Record<string, number> = {
    ArrowUp: up,
    ArrowRight: up,
    ArrowDown: down,
    ArrowLeft: down,
    PageUp: n === 0 ? 0 : (REPEAT_SNAP.find((s) => s > n) ?? 0),
    PageDown: n === 0 ? SEND_MAX_REPEAT : ([...REPEAT_SNAP].reverse().find((s) => s < n) ?? 1),
    Home: 1,
    End: 0,
  };
  return next[key];
}

export function repeatBrief(n: number): string {
  return n === 0 ? "Endless" : n === 1 ? "Once" : `${n} times`;
}

/** One line for the run bar: `3 frames, every 100 ms, until stopped`. */
export function sequenceSummary(count: number, periodMs: number, repeat: number): string {
  const rep = repeat === 0 ? "until stopped" : repeat === 1 ? "once" : `${repeat} rounds`;
  return `${count} frame${count === 1 ? "" : "s"}, every ${periodBrief(periodMs)}, ${rep}`;
}

// Sequence files: the JSON body of POST /can/send, used for Export, Import and the SD card (/config/send/<name>.json).
/** Hostile input: a file bigger than this is refused before JSON.parse. 32 frames need under 2 KB. */
export const SEQUENCE_MAX_BYTES = 16 * 1024;
export type Sequence = { frames: SendFrame[]; periodMs: number; repeat: number };

export function serializeSequence(s: Sequence): string {
  return JSON.stringify(planBody(s.frames, s.periodMs, s.repeat), null, 2);
}

const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);

/** All or nothing, with the same limits the firmware applies. `repeat` is optional (default 1) like on the wire. */
export function parseSequence(text: string): Checked<Sequence> {
  if (text.length > SEQUENCE_MAX_BYTES) return { ok: false, reason: "File is larger than 16 KB" };
  let doc: unknown;
  try {
    doc = JSON.parse(text);
  } catch {
    return { ok: false, reason: "File is not JSON" };
  }
  if (!isObj(doc)) return { ok: false, reason: "File is not a sequence" };
  const list = doc.frames;
  if (!Array.isArray(list) || list.length < 1 || list.length > SEND_MAX_FRAMES) return { ok: false, reason: `frames must be a list of 1 to ${SEND_MAX_FRAMES}` };
  const frames: SendFrame[] = [];
  for (let i = 0; i < list.length; i++) {
    const f: unknown = list[i];
    if (!isObj(f) || typeof f.id !== "number" || !Number.isInteger(f.id) || f.id < 0 || f.id > 0x7ff) return { ok: false, reason: `Frame ${i + 1}: id must be 0 to 2047` };
    const data = typeof f.data === "string" ? parseSendData(f.data) : null;
    if (!data?.ok) return { ok: false, reason: `Frame ${i + 1}: data must be 1 to 8 hex bytes` };
    frames.push({ id: f.id, data: data.value });
  }
  const period = doc.period_ms;
  if (typeof period !== "number" || !Number.isInteger(period) || period < SEND_MIN_PERIOD_MS || period > SEND_MAX_PERIOD_MS) return { ok: false, reason: `period_ms must be ${SEND_MIN_PERIOD_MS} to ${SEND_MAX_PERIOD_MS}` };
  const repeat = doc.repeat ?? 1;
  if (typeof repeat !== "number" || !Number.isInteger(repeat) || repeat < 0 || repeat > SEND_MAX_REPEAT) return { ok: false, reason: `repeat must be 0 to ${SEND_MAX_REPEAT}` };
  return { ok: true, value: { frames, periodMs: period, repeat } };
}
