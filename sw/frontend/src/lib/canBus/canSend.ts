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
