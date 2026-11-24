// PID names and units of a recorded trip, from its sidecar (docs/sd-card.md). The sidecar is an SD card file, so every field is checked.
import type { PidDefinition } from "./types";

export const SIDECAR_MAX_BYTES = 64 * 1024;
const MAX_PIDS = 256;
const MAX_LABEL = 64;

export interface SidecarPid {
  name: string;
  unit: string;
}

/** Plot colours for PIDs that have no live definition. */
const FALLBACK_COLORS = [0x10b981, 0x3b82f6, 0xf59e0b, 0xef4444, 0x8b5cf6, 0x14b8a6, 0xec4899, 0x84cc16];

const text = (v: unknown) => (typeof v === "string" ? v.trim().slice(0, MAX_LABEL) : "");

/** pid -> name and unit, or null when the text is not a usable sidecar. Bad entries are skipped. */
export function parseSidecarPids(source: string): Map<number, SidecarPid> | null {
  if (source.length > SIDECAR_MAX_BYTES) return null;
  let doc: unknown;
  try {
    doc = JSON.parse(source);
  } catch {
    return null;
  }
  const list = doc !== null && typeof doc === "object" ? (doc as { pids?: unknown }).pids : undefined;
  if (!Array.isArray(list)) return null;
  const out = new Map<number, SidecarPid>();
  for (const item of list.slice(0, MAX_PIDS * 4)) {
    if (out.size >= MAX_PIDS) break;
    if (item === null || typeof item !== "object") continue;
    const { pid, name, unit } = item as Record<string, unknown>;
    if (typeof pid !== "number" || !Number.isInteger(pid) || pid < 0 || pid > 0xffff || out.has(pid)) continue;
    const n = text(name);
    if (n === "") continue;
    out.set(pid, { name: n, unit: text(unit) });
  }
  return out;
}

/** One definition per trip PID. Name and unit come from the sidecar; colour, range and mode from the live definition when there is one. */
export function tripDefs(pids: readonly number[], sidecar: ReadonlyMap<number, SidecarPid> | null, live: readonly PidDefinition[]): PidDefinition[] {
  const byPid = new Map(live.map((d) => [d.pid, d]));
  return pids.map((pid, i) => {
    const l = byPid.get(pid);
    const s = sidecar?.get(pid);
    return {
      id: l?.id ?? 0,
      mode: l?.mode ?? 1,
      pid,
      length: l?.length ?? 1,
      name: s?.name || l?.name || `0x${pid.toString(16).toUpperCase().padStart(2, "0")}`,
      unit: s ? s.unit : (l?.unit ?? ""),
      description: l?.description ?? "",
      formula: l?.formula ?? "",
      minValue: l?.minValue ?? NaN,
      maxValue: l?.maxValue ?? NaN,
      update_interval_ms: l?.update_interval_ms ?? 500,
      color: l?.color ?? FALLBACK_COLORS[i % FALLBACK_COLORS.length],
      priority: l?.priority ?? 0,
      icon: l?.icon ?? "",
    };
  });
}
