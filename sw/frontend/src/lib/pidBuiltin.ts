// Built-in PIDs: the firmware's core set (PID 0x04, 0x05, 0x0C, 0x0D, 0x0F, 0x10). The device refuses any write that
// changes one, with HTTP 403 and code PID_BUILTIN. Pure, so it is unit-tested (pidBuiltin.test.ts).
import { ApiError } from "./api";

export const BUILTIN_REFUSAL = "Built-in PIDs cannot be changed or removed. Revert the change and apply again.";

/** The user-facing message for a PID_BUILTIN refusal, or null for any other error. */
export function builtinRefusal(error: unknown): string | null {
  return error instanceof ApiError && error.code === "PID_BUILTIN" ? BUILTIN_REFUSAL : null;
}
