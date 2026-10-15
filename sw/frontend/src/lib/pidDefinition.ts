import type { PidDefinition } from "./types";
import { safeHexColor } from "./color";

export const DEFAULT_PID_PRIORITY = 5;
export const DEFAULT_PID_COLOR = 0x01aaff;

const isSet = (v: unknown) => v !== undefined && v !== null;

/**
 * Accepts both the GET shape and the file/PUT shape. Required fields must be
 * present. Optional fields (unit, desc, priority, color, icon, minV, maxV) may
 * be null or missing: cJSON writes NaN as null.
 */
export function normalizePidDefinition(raw: unknown): PidDefinition {
  if (!raw || typeof raw !== "object") throw new Error("Invalid PID definition");
  const def = raw as Record<string, unknown>;

  // Presence is checked below; field types are trusted as before.
  const required = {
    id: def.id as number,
    mode: def.mode as number,
    pid: def.pid as number,
    length: (def.length ?? def.len) as number,
    name: def.name as string,
    formula: def.formula as string,
    update_interval_ms: (def.update_interval_ms ?? def.interval) as number,
  };
  if (Object.values(required).some((v) => !isSet(v))) throw new Error("Incomplete PID definition");

  return {
    ...required,
    unit: (def.unit ?? "") as string,
    description: (def.description ?? def.desc ?? "") as string,
    minValue: (def.minValue ?? def.minV ?? NaN) as number,
    maxValue: (def.maxValue ?? def.maxV ?? NaN) as number,
    color: parseInt(safeHexColor(def.color, "#01aaff").slice(1), 16),
    priority: (def.priority ?? DEFAULT_PID_PRIORITY) as number,
    icon: (def.icon ?? "") as string,
  };
}

/** `GET /pid_def` body to a list. Any unusable item fails the load: a partial list would be written back by the next PUT. */
export function parsePidList(raw: unknown): PidDefinition[] {
  const data = (raw as { data?: unknown } | null)?.data;
  if (!Array.isArray(data)) throw new Error("Invalid PID definition response");
  return data.map(normalizePidDefinition);
}

/** Request/file shape. NaN (unset) min/max go out as null, which JSON.stringify produces. */
export function serializePidDefinition(def: PidDefinition) {
  return {
    id: def.id,
    mode: def.mode,
    pid: def.pid,
    length: def.length,
    name: def.name,
    unit: def.unit,
    desc: def.description,
    formula: def.formula,
    minV: Number.isFinite(def.minValue) ? def.minValue : null,
    maxV: Number.isFinite(def.maxValue) ? def.maxValue : null,
    priority: def.priority,
    interval: def.update_interval_ms,
    color: def.color,
    icon: def.icon,
  };
}
