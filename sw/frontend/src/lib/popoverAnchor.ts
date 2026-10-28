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
