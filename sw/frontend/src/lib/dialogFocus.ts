import type { Attachment } from "svelte/attachments";

const FOCUSABLE =
  'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

function visibleItems(node: HTMLElement) {
  return [...node.querySelectorAll<HTMLElement>(FOCUSABLE)].filter(
    (el) => el.getClientRects().length > 0,
  );
}

/**
 * Modal behaviour for a role="dialog" element: moves focus inside, closes on
 * Escape, keeps Tab inside, and returns focus to the opener on removal.
 * Listens on the dialog node, so Escape reaches only the focused (topmost) dialog.
 */
export function dialogFocus(onClose: () => void): Attachment<HTMLElement> {
  return (node) => {
    const opener = document.activeElement as HTMLElement | null;
    (visibleItems(node)[0] ?? node).focus();

    function onKeydown(e: KeyboardEvent) {
      if (e.key === "Escape") {
        e.preventDefault();
        e.stopPropagation();
        onClose();
        return;
      }
      if (e.key !== "Tab") return;
      const items = visibleItems(node);
      if (items.length === 0) return;
      const first = items[0];
      const last = items[items.length - 1];
      const active = document.activeElement;
      const inside = node.contains(active);
      if (e.shiftKey && (!inside || active === first)) {
        e.preventDefault();
        last.focus();
      } else if (!e.shiftKey && (!inside || active === last)) {
        e.preventDefault();
        first.focus();
      }
    }

    node.addEventListener("keydown", onKeydown);
    return () => {
      node.removeEventListener("keydown", onKeydown);
      opener?.focus?.();
    };
  };
}
