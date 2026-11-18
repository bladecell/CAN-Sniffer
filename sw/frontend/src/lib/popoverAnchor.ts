/**
 * Shared by the editor popovers (Unit, Icon, Colour) and the Mode dropdown.
 * A popover anchors to its field's box, so it never covers the field's label, and closes when anything
 * outside it scrolls, so it cannot drift away from the field.
 */

/** The field the element belongs to (label, control and hint), or the element itself. */
export function fieldBox(el: Element): DOMRect {
  return (el.closest(".fld") ?? el).getBoundingClientRect();
}

/** Calls onScroll when the page or any scroll container moves, except scrolling inside `inside`. Returns the stop function. */
export function closeOnOutsideScroll(inside: Element | null | undefined, onScroll: () => void): () => void {
  const handler = (e: Event) => {
    if (!(e.target instanceof Node && inside?.contains(e.target))) onScroll();
  };
  document.addEventListener("scroll", handler, { capture: true, passive: true });
  return () => document.removeEventListener("scroll", handler, { capture: true });
}

/** Action: fixes the popup to the viewport, clamped 8px inside it, below the anchor (above if there is more room). Re-places on resize, scroll and size change. */
export function placePopup(pop: HTMLElement, anchor: () => Element | undefined) {
  const go = () => {
    const t = anchor()?.getBoundingClientRect();
    if (!t) return;
    const vw = window.innerWidth;
    const vh = window.innerHeight;
    pop.style.maxWidth = `${vw - 16}px`;
    const w = pop.offsetWidth;
    pop.style.left = `${Math.max(8, Math.min(t.left, vw - w - 8))}px`;
    const foot = vw <= 768 ? 96 : 14; // the phone nav bar sits at the bottom
    const below = vh - t.bottom - foot;
    const above = t.top - 14;
    const up = below < Math.min(pop.scrollHeight, 260) && above > below;
    pop.style.maxHeight = `${Math.max(120, up ? above : below)}px`;
    pop.style.top = up ? "auto" : `${t.bottom + 6}px`;
    pop.style.bottom = up ? `${vh - t.top + 6}px` : "auto";
  };
  go();
  const ro = new ResizeObserver(go);
  ro.observe(pop);
  window.addEventListener("resize", go);
  document.addEventListener("scroll", go, { capture: true, passive: true });
  return {
    destroy() {
      ro.disconnect();
      window.removeEventListener("resize", go);
      document.removeEventListener("scroll", go, { capture: true });
    },
  };
}
