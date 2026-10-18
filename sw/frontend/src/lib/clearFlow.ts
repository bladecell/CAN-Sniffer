// Two-press DTC clear: idle -> armed -> sending -> idle. The first press only arms (nothing is sent);
// the second press, inside the arming window, sends POST /req/clear_dtc once. Times are monotonic ms.
export type ClearState =
  | { kind: "idle" }
  | { kind: "armed"; expiresAt: number; armedAt: number }
  | { kind: "sending" };

/** A press this soon after arming is the same gesture (double click, key repeat). */
export const MIN_CONFIRM_DELAY_MS = 500;
/** How long the Confirm state waits before it reverts to idle. */
export const ARM_WINDOW_MS = 10_000;

export const idle: ClearState = { kind: "idle" };

/** `send` is true only for the press that confirms an armed clear. */
export function press(state: ClearState, now: number): { state: ClearState; send: boolean } {
  if (state.kind === "sending") return { state, send: false };
  if (state.kind === "armed" && now < state.expiresAt) {
    if (now - state.armedAt < MIN_CONFIRM_DELAY_MS) return { state, send: false };
    return { state: { kind: "sending" }, send: true };
  }
  return { state: { kind: "armed", armedAt: now, expiresAt: now + ARM_WINDOW_MS }, send: false };
}

/** The request finished, whatever its result. */
export function settle(): ClearState {
  return idle;
}

export function tick(state: ClearState, now: number): ClearState {
  return state.kind === "armed" && now >= state.expiresAt ? idle : state;
}

export function cancel(state: ClearState): ClearState {
  return state.kind === "armed" ? idle : state;
}
