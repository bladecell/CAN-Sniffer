// Smooth layout changes in a section grid: grid spans cannot be transitioned, so each card is animated
// from where it was (position and size) to where it is now. Skipped for prefers-reduced-motion.

export const FLIP_MS = 160;

const reduced = () => typeof window !== "undefined" && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
const cardsOf = (grid: HTMLElement) => Array.from(grid.querySelectorAll<HTMLElement>(":scope > [data-card]"));

/** Where every card is now, as drawn (a running animation counts). Call before the layout changes. */
export function snapshotCards(grid: HTMLElement): Map<string, DOMRect> {
  const rects = new Map<string, DOMRect>();
  for (const el of cardsOf(grid)) rects.set(el.dataset.card ?? "", el.getBoundingClientRect());
  return rects;
}

/** Animates every card that moved or changed size since the snapshot. Call after the layout changed. */
export function playCards(grid: HTMLElement, before: Map<string, DOMRect>): void {
  if (reduced()) return;
  const els = cardsOf(grid);
  for (const el of els) el.getAnimations().forEach((a) => a.cancel());
  for (const el of els) {
    const a = before.get(el.dataset.card ?? "");
    if (!a) continue;
    const b = el.getBoundingClientRect();
    const dx = a.left - b.left;
    const dy = a.top - b.top;
    if (Math.abs(dx) < 1 && Math.abs(dy) < 1 && Math.abs(a.width - b.width) < 1 && Math.abs(a.height - b.height) < 1) continue;
    el.animate(
      [
        { transform: `translate(${dx}px, ${dy}px)`, width: `${a.width}px`, height: `${a.height}px` },
        { transform: "translate(0, 0)", width: `${b.width}px`, height: `${b.height}px` },
      ],
      { duration: FLIP_MS, easing: "cubic-bezier(.2,.8,.2,1)" },
    );
  }
}
