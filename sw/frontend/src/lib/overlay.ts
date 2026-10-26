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

// Overlays can stack (a confirm over a sheet); the page unlocks when the last one closes.
let scrollLocks = 0;

export interface OverlayOptions {
  open: boolean;
  dismissible: boolean;
  onClose?: () => void;
}

/**
 * Svelte action behind Sheet and Dialog. Opens the native <dialog> with showModal() while `open`.
 * Esc and a backdrop click call onClose when dismissible. Tab is trapped inside, the page is scroll-locked,
 * and focus returns to the opener (or <main> when the opener was removed). Render the body inside {#if open}
 * so each open starts from a fresh form.
 */
export function overlay(node: HTMLDialogElement, initial: OverlayOptions) {
  let opts = initial;
  let opener: HTMLElement | null = null;
  let locked = false;
  let pressedOnBackdrop = false;

  function requestClose() {
    if (opts.dismissible) opts.onClose?.();
  }

  function lock(on: boolean) {
    if (on === locked) return;
    locked = on;
    if (on) {
      scrollLocks++;
      document.body.classList.add("overlay-lock");
    } else if (--scrollLocks === 0) {
      document.body.classList.remove("overlay-lock");
    }
  }

  function sync() {
    if (opts.open && !node.open) {
      // body means the trigger was already removed by the time this runs: no opener to return to.
      const active = document.activeElement as HTMLElement | null;
      opener = active && active !== document.body ? active : null;
      node.showModal();
      lock(true);
    } else if (!opts.open && node.open) {
      node.close();
      lock(false);
      const main = document.querySelector<HTMLElement>("main");
      if (main && !main.hasAttribute("tabindex")) main.tabIndex = -1;
      const target = returnFocusTarget(opener, (el) => el.isConnected, main);
      opener = null;
      // Deferred: the browser's own focus restore runs after close() and would blur a synchronous focus.
      setTimeout(() => target?.focus(), 0);
    }
  }

  function onCancel(e: Event) {
    // Esc reaches here only when no popover (SelectMenu) consumed it first.
    e.preventDefault();
    requestClose();
  }

  function onKeydown(e: KeyboardEvent) {
    // defaultPrevented: a nested overlay already moved focus.
    if (e.key !== "Tab" || e.defaultPrevented) return;
    const items = focusables(node);
    const next = trapTarget(items.length, items.indexOf(document.activeElement as HTMLElement), e.shiftKey);
    if (next < 0) return;
    e.preventDefault();
    items[next].focus();
  }

  function onPointerdown(e: PointerEvent) {
    pressedOnBackdrop = e.target === node;
  }

  function onClick(e: MouseEvent) {
    if (pressedOnBackdrop && e.target === node) requestClose();
  }

  node.addEventListener("cancel", onCancel);
  node.addEventListener("keydown", onKeydown);
  node.addEventListener("pointerdown", onPointerdown);
  node.addEventListener("click", onClick);
  sync();

  return {
    update(next: OverlayOptions) {
      opts = next;
      sync();
    },
    destroy() {
      node.removeEventListener("cancel", onCancel);
      node.removeEventListener("keydown", onKeydown);
      node.removeEventListener("pointerdown", onPointerdown);
      node.removeEventListener("click", onClick);
      if (node.open) node.close();
      lock(false);
    },
  };
}
