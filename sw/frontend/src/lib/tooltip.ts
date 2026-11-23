/**
 * Replaces the browser's native title tooltip with the app's popup surface (same look as SidebarHint).
 * One delegated listener: any element with a title attribute gets the popup on mouse hover or keyboard focus.
 * The title moves to data-tip while shown (so the native tooltip stays off) and goes back on hide.
 */
import { closeOnOutsideScroll } from "$lib/popoverAnchor";

const DELAY_MS = 400;

export function installTooltips(): () => void {
  const pop = document.createElement("div");
  pop.className = "popup tip";
  pop.setAttribute("popover", "manual");
  pop.setAttribute("role", "tooltip");
  document.body.append(pop);

  let target: HTMLElement | null = null;
  let timer = 0;
  let stopScroll: (() => void) | undefined;

  const restore = () => {
    if (target?.dataset.tip !== undefined) {
      target.setAttribute("title", target.dataset.tip);
      delete target.dataset.tip;
    }
  };
  const hide = () => {
    clearTimeout(timer);
    stopScroll?.();
    stopScroll = undefined;
    if (pop.matches(":popover-open")) pop.hidePopover();
    restore();
    target = null;
  };
  const place = (el: HTMLElement) => {
    const r = el.getBoundingClientRect();
    const w = pop.offsetWidth;
    const h = pop.offsetHeight;
    pop.style.left = `${Math.max(8, Math.min(r.left + r.width / 2 - w / 2, window.innerWidth - w - 8))}px`;
    const below = r.bottom + 8;
    pop.style.top = `${below + h > window.innerHeight - 8 ? Math.max(8, r.top - h - 8) : below}px`;
  };
  const show = (el: HTMLElement) => {
    const text = el.dataset.tip;
    if (!text || target !== el) return;
    pop.textContent = text;
    pop.showPopover();
    place(el);
    stopScroll = closeOnOutsideScroll(pop, hide);
  };
  const arm = (el: HTMLElement, delay: number) => {
    const text = el.getAttribute("title");
    if (!text) return;
    hide();
    target = el;
    el.dataset.tip = text;
    el.removeAttribute("title");
    if (!el.hasAttribute("aria-label") && !el.textContent?.trim()) el.setAttribute("aria-label", text);
    timer = window.setTimeout(() => show(el), delay);
  };
  const tipped = (n: EventTarget | null) => (n instanceof Element ? n.closest<HTMLElement>("[title], [data-tip]") : null);

  const onOver = (e: PointerEvent) => {
    if (e.pointerType !== "mouse") return;
    const el = tipped(e.target);
    if (el && el !== target) arm(el, DELAY_MS);
  };
  const onOut = (e: PointerEvent) => {
    if (target && e.pointerType === "mouse" && !target.contains(e.relatedTarget as Node | null)) hide();
  };
  const onFocus = (e: FocusEvent) => {
    const el = tipped(e.target);
    if (el && (e.target as HTMLElement).matches(":focus-visible")) arm(el, 0);
  };
  const onBlur = () => hide();
  const onKey = (e: KeyboardEvent) => e.key === "Escape" && hide();

  document.addEventListener("pointerover", onOver);
  document.addEventListener("pointerout", onOut);
  document.addEventListener("pointerdown", onBlur, true);
  document.addEventListener("focusin", onFocus);
  document.addEventListener("focusout", onBlur);
  document.addEventListener("keydown", onKey);
  return () => {
    hide();
    document.removeEventListener("pointerover", onOver);
    document.removeEventListener("pointerout", onOut);
    document.removeEventListener("pointerdown", onBlur, true);
    document.removeEventListener("focusin", onFocus);
    document.removeEventListener("focusout", onBlur);
    document.removeEventListener("keydown", onKey);
    pop.remove();
  };
}
