export const RECONNECT_BASE_MS = 2000;
export const RECONNECT_MAX_MS = 30000;

export function reconnectDelay(attempt: number): number {
  const n = Number.isFinite(attempt) ? Math.min(Math.max(0, Math.floor(attempt)), 10) : 0;
  return Math.min(RECONNECT_MAX_MS, RECONNECT_BASE_MS * 2 ** n);
}
