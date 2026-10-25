const FOCUSABLE =
  'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

/** Focusable, visible elements inside a dialog, in tab order. */
export function focusables(node: HTMLElement): HTMLElement[] {
  return [...node.querySelectorAll<HTMLElement>(FOCUSABLE)].filter(
    (el) => el.getClientRects().length > 0,
  );
}

/**
 * Index to focus for a Tab press inside a dialog, or -1 to let the browser move focus.
 * `index` is the focused item's position in `focusables`, or -1 when focus is not in the list.
 */
export function trapTarget(count: number, index: number, shift: boolean): number {
  if (count === 0) return -1;
  if (index < 0) return shift ? count - 1 : 0;
  if (shift && index === 0) return count - 1;
  if (!shift && index === count - 1) return 0;
  return -1;
}

/** Element to focus when a dialog closes: the opener while it is still on the page, else the fallback. */
export function returnFocusTarget<T>(opener: T | null, isConnected: (el: T) => boolean, fallback: T | null): T | null {
  if (opener && isConnected(opener)) return opener;
  return fallback;
}
