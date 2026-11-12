export const RECONNECT_BASE_MS = 2000;
export const RECONNECT_MAX_MS = 30000;

export function reconnectDelay(attempt: number): number {
  const n = Number.isFinite(attempt) ? Math.min(Math.max(0, Math.floor(attempt)), 10) : 0;
  return Math.min(RECONNECT_MAX_MS, RECONNECT_BASE_MS * 2 ** n);
}

/** Spreads a delay over 75..125% so tabs and a rebooted device do not reconnect in lockstep. `rand` is in [0, 1). */
export function jitter(ms: number, rand: number): number {
  return Math.round(ms * (0.75 + 0.5 * Math.min(Math.max(rand, 0), 1)));
}
