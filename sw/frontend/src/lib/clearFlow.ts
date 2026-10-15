// Two-press DTC clear: idle -> sending -> armed(token) -> sending(token) -> idle.
// Times are monotonic ms (performance.now()). Only press 2 (state armed and not expired) ever sends a token. Nothing here confirms on its own.
export type ClearState =
  | { kind: "idle" }
  | { kind: "armed"; token: string; expiresAt: number; armedAt: number }
  | { kind: "sending"; token?: string };

export type ClearResult =
  | { kind: "confirm"; token: string; expiresMs: number }
  | { kind: "done" }
  | { kind: "error" };

/** A press this soon after the button turned to Confirm is the same gesture (double click, key repeat). */
export const MIN_CONFIRM_DELAY_MS = 500;

export const idle: ClearState = { kind: "idle" };

/** `send` is null when the press does nothing; `{}` is the token-less request. */
export function press(state: ClearState, now: number): { state: ClearState; send: { token?: string } | null } {
  if (state.kind === "sending") return { state, send: null };
  if (state.kind === "armed" && now < state.expiresAt) {
    if (now - state.armedAt < MIN_CONFIRM_DELAY_MS) return { state, send: null };
    return { state: { kind: "sending", token: state.token }, send: { token: state.token } };
  }
  return { state: { kind: "sending" }, send: {} };
}

export function settle(state: ClearState, result: ClearResult, now: number, sentAt = now): { state: ClearState; notice: "expired" | null } {
  if (state.kind !== "sending") return { state, notice: null };
  if (result.kind === "confirm") {
    // A 409 after press 2 means the token expired or rotated: revert, do not retry.
    if (state.token !== undefined) return { state: idle, notice: "expired" };
    return { state: { kind: "armed", token: result.token, expiresAt: sentAt + result.expiresMs, armedAt: now }, notice: null };
  }
  return { state: idle, notice: null };
}

export function tick(state: ClearState, now: number): ClearState {
  return state.kind === "armed" && now >= state.expiresAt ? idle : state;
}

export function cancel(state: ClearState): ClearState {
  return state.kind === "armed" ? idle : state;
}
