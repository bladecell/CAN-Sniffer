/**
 * Float layers (DESIGN.md, layering rule). Gecko drops a layer's backdrop blur when any ancestor is a scroller,
 * a clipped box, a transformed or translucent box, a filtered or masked box, or another blurred box. So a float
 * layer is a sibling of the scroller it covers, and a popover is portalled to body.
 */

/** Svelte attachment: writes the element's height to a CSS variable on its parent, so the scroller beside it can pad past it. */
export function measureHeight(prop: string) {
  return (el: HTMLElement) => {
    const host = el.parentElement;
    const sync = () => host?.style.setProperty(prop, `${el.offsetHeight}px`);
    const ro = new ResizeObserver(sync);
    ro.observe(el);
    sync();
    return () => ro.disconnect();
  };
}

/**
 * Svelte attachment for a scroller: writes data-under="top bottom" on its parent while content is scrolled past that
 * edge. A float layer beside the scroller (class rest-top or rest-bottom, app.css) floats only while content is under it.
 */
export function trackUnder(el: HTMLElement) {
  const host = el.parentElement;
  const sync = () => {
    const under = [el.scrollTop > 1 && "top", el.scrollTop + el.clientHeight < el.scrollHeight - 1 && "bottom"].filter(Boolean).join(" ");
    if (host && host.dataset.under !== under) host.dataset.under = under;
  };
  el.addEventListener("scroll", sync, { passive: true });
  // The content can grow or shrink without a scroll or a resize of the scroller.
  const ro = new ResizeObserver(sync);
  ro.observe(el);
  const mo = new MutationObserver(sync);
  mo.observe(el, { childList: true, subtree: true });
  sync();
  return () => {
    el.removeEventListener("scroll", sync);
    ro.disconnect();
    mo.disconnect();
    delete host?.dataset.under;
  };
}

/** Svelte attachment for a position:sticky bar on the page: writes data-under="top" on its parent while the bar is pinned. */
export function trackStuck(el: HTMLElement) {
  const host = el.parentElement;
  const top = parseFloat(getComputedStyle(el).top) || 0;
  // Pinned, the bar's top edge is 1px outside the shrunk root, so it is no longer fully visible.
  const io = new IntersectionObserver(
    ([e]) => {
      if (e.intersectionRatio < 1) host?.setAttribute("data-under", "top");
      else host?.removeAttribute("data-under");
    },
    { rootMargin: `${-top - 1}px 0px 0px 0px`, threshold: 1 },
  );
  io.observe(el);
  return () => {
    io.disconnect();
    host?.removeAttribute("data-under");
  };
}

/**
 * Svelte action: moves the element out of its scroller. The target is the nearest dialog, else body. A modal makes
 * everything outside it inert, so a popover in a sheet stays inside the sheet's dialog (the dialog is not a scroller,
 * clip or blur). Svelte removes the element with its component.
 */
export function portal(node: HTMLElement) {
  (node.closest("dialog") ?? document.body).append(node);
  return { destroy: () => node.remove() };
}

/** Computed style of one ancestor of a float layer, as read by getComputedStyle. */
export interface AncestorStyle {
  transform: string;
  opacity: string;
  filter: string;
  clipPath: string;
  maskImage: string;
  backdropFilter: string;
  overflowX: string;
  overflowY: string;
}

/** Reasons this ancestor can drop a float layer's blur. Empty when it cannot. */
export function blurBlockers(s: AncestorStyle): string[] {
  const why: string[] = [];
  if (s.transform !== "none") why.push("transform");
  if (s.opacity !== "1") why.push("opacity");
  if (s.filter !== "none") why.push("filter");
  if (s.clipPath !== "none") why.push("clip-path");
  if (s.maskImage !== "none") why.push("mask");
  if (s.backdropFilter !== "none") why.push("backdrop-filter");
  if (s.overflowX !== "visible" || s.overflowY !== "visible") why.push("overflow");
  return why;
}
