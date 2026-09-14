export const MAX_PID_DEFINITIONS = 256;
export const MAX_PID_NAME_LENGTH = 64;
export const MAX_PID_FORMULA_LENGTH = 1024;
export const MIN_PID_INTERVAL = 16;
export const MAX_PID_INTERVAL = 65535;
export const MIN_PID_PRIORITY = 0;
export const MAX_PID_PRIORITY = 255;

const numericFields = [
  "id",
  "mode",
  "pid",
  "length",
  "minValue",
  "maxValue",
  "update_interval_ms",
  "color",
  "priority",
] as const;

function isFiniteNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

export function validatePidDefinitionSet(definitions: unknown[]): string | null {
  if (definitions.length > MAX_PID_DEFINITIONS) {
    return `A maximum of ${MAX_PID_DEFINITIONS} PID definitions is supported.`;
  }

  for (const [index, definition] of definitions.entries()) {
    if (!definition || typeof definition !== "object") {
      return `PID definition ${index + 1} is invalid.`;
    }

    const def = definition as Record<string, unknown>;
    for (const field of numericFields) {
      if (!isFiniteNumber(def[field])) {
        return `PID definition ${index + 1}: ${field} must be a finite number.`;
      }
    }

    const pid = def.pid;
    const priority = def.priority;
    const interval = def.update_interval_ms;
    if (!isFiniteNumber(pid) || !isFiniteNumber(priority) || !isFiniteNumber(interval)) {
      return `PID definition ${index + 1}: numeric values must be finite numbers.`;
    }

    for (const [field, value] of [["pid", pid], ["priority", priority], ["update_interval_ms", interval]] as const) {
      if (!Number.isInteger(value)) {
        return `PID definition ${index + 1}: ${field} must be an integer.`;
      }
    }

    if (pid < 0 || pid > 0xffff) {
      return `PID definition ${index + 1}: pid must be between 0 and 65535.`;
    }

    if (priority < MIN_PID_PRIORITY || priority > MAX_PID_PRIORITY) {
      return `PID definition ${index + 1}: priority must be between ${MIN_PID_PRIORITY} and ${MAX_PID_PRIORITY}.`;
    }

    if (
      interval < 0 ||
      interval > MAX_PID_INTERVAL ||
      (interval !== 0 && interval < MIN_PID_INTERVAL)
    ) {
      return `PID definition ${index + 1}: update interval must be ${MIN_PID_INTERVAL}-${MAX_PID_INTERVAL} ms or 0.`;
    }

    if (typeof def.name !== "string" || def.name.length > MAX_PID_NAME_LENGTH) {
      return `PID definition ${index + 1}: name must be at most ${MAX_PID_NAME_LENGTH} characters.`;
    }

    if (
      typeof def.formula !== "string" ||
      def.formula.length === 0 ||
      def.formula.length > MAX_PID_FORMULA_LENGTH
    ) {
      return `PID definition ${index + 1}: formula must be non-empty and at most ${MAX_PID_FORMULA_LENGTH} characters.`;
    }
  }

  return null;
}
