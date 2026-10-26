/** Toasts shown at once. Adding one past this drops the oldest. */
export const MAX_TOASTS = 3;
/** Auto-dismiss delay. Paused while the pointer or focus is on the toast. */
export const TOAST_MS = 5000;

/** Ids to drop so that `max - 1` remain, leaving room for one incoming toast. Oldest first. */
export function evictOldest(ids: readonly number[], max = MAX_TOASTS): number[] {
  return ids.slice(0, Math.max(0, ids.length - (max - 1)));
}

/** The toast Esc dismisses: the newest. Null while a dialog or popover is open, since they own Esc. */
export function escapeTarget(ids: readonly number[], overlayOpen: boolean): number | null {
  if (overlayOpen || ids.length === 0) return null;
  return ids[ids.length - 1];
}

/** Time left on an auto-dismiss timer that started at `startedAt` and was paused at `now`. */
export function remainingMs(leftMs: number, startedAt: number, now: number): number {
  return Math.max(0, leftMs - (now - startedAt));
}
