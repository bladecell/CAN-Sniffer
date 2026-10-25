<script module lang="ts">
  let seq = 0;
  // Dialogs can stack (for example a login prompt over an import); the page unlocks when the last one closes.
  let scrollLocks = 0;
</script>

<script lang="ts">
  import type { Snippet } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { focusables, returnFocusTarget, trapTarget } from "$lib/modal";

  interface Props {
    /** One-way: the parent owns it. A dismiss calls onClose, and the parent decides whether to close. */
    open?: boolean;
    title: string;
    onClose?: () => void;
    variant?: "dialog" | "sheet";
    size?: "md" | "lg";
    dismissible?: boolean;
    children: Snippet;
    footer?: Snippet;
  }

  let {
    open = false,
    title,
    onClose,
    variant = "dialog",
    size = "md",
    dismissible = true,
    children,
    footer,
  }: Props = $props();

  const titleId = `modal-title-${++seq}`;
  let dialogEl = $state<HTMLDialogElement>();
  let pressedOnBackdrop = false;
  let opener: HTMLElement | null = null;

  // Native showModal() gives the top layer and inert page. Focus goes back to the opener on close,
  // or to <main> when the opener was removed while the dialog was open.
  $effect(() => {
    if (!dialogEl) return;
    if (open && !dialogEl.open) {
      // body means the trigger was already removed by the time this effect ran: no opener to return to.
      const active = document.activeElement as HTMLElement | null;
      opener = active && active !== document.body ? active : null;
      dialogEl.showModal();
    } else if (!open && dialogEl.open) {
      dialogEl.close();
      const main = document.querySelector<HTMLElement>("main");
      if (main && !main.hasAttribute("tabindex")) main.tabIndex = -1;
      const target = returnFocusTarget(opener, (el) => el.isConnected, main);
      opener = null;
      // Deferred: the browser's own focus restore runs after close() and would blur a synchronous focus.
      setTimeout(() => target?.focus(), 0);
    }
  });

  // Own class, not the shared no-scroll one: Dashboard clears no-scroll on its own schedule.
  $effect(() => {
    if (!open) return;
    scrollLocks++;
    document.body.classList.add("modal-lock");
    return () => {
      if (--scrollLocks === 0) document.body.classList.remove("modal-lock");
    };
  });

  function requestClose() {
    if (!dismissible) return;
    onClose?.();
  }

  function onKeydown(e: KeyboardEvent) {
    if (e.key !== "Tab" || !dialogEl) return;
    const items = focusables(dialogEl);
    const next = trapTarget(items.length, items.indexOf(document.activeElement as HTMLElement), e.shiftKey);
    if (next < 0) return;
    e.preventDefault();
    items[next].focus();
  }
</script>

<dialog
  bind:this={dialogEl}
  class="modal {variant}"
  data-size={size}
  aria-modal="true"
  aria-labelledby={titleId}
  oncancel={(e) => {
    // Esc reaches here only when no popover (SelectMenu) consumed it first.
    e.preventDefault();
    requestClose();
  }}
  onkeydown={onKeydown}
  onpointerdown={(e) => (pressedOnBackdrop = e.target === dialogEl)}
  onclick={(e) => {
    if (pressedOnBackdrop && e.target === dialogEl) requestClose();
  }}
>
  {#if variant === "sheet"}
    <div class="grab" aria-hidden="true"></div>
  {/if}
  <header>
    <h2 id={titleId}>{title}</h2>
    {#if dismissible}
      <button type="button" class="close" aria-label="Close" onclick={requestClose}>
        <Icon name="x" size={20} />
      </button>
    {/if}
  </header>
  {#if open}
    <div class="body">{@render children()}</div>
    {#if footer}
      <footer>{@render footer()}</footer>
    {/if}
  {/if}
</dialog>

<style>
  :global(body.modal-lock) {
    overflow: hidden;
    touch-action: none;
  }

  /* Pico styles every <dialog> as a full-screen overlay (min-width/height 100%, centred flex); reset those here. */
  dialog.modal {
    box-sizing: border-box;
    display: none;
    width: min(var(--modal-w), calc(100% - 32px));
    min-width: 0;
    height: auto;
    min-height: 0;
    max-width: none;
    max-height: calc(100dvh - 32px);
    margin: auto;
    padding: 0;
    border: 0;
    border-radius: var(--r-float);
    background: var(--float);
    color: var(--text);
    box-shadow: var(--shadow-float);
    -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
  }
  dialog.modal[open] {
    display: flex;
    flex-direction: column;
    align-items: stretch;
    justify-content: flex-start;
    animation: modal-in 0.4s var(--ease-out);
  }
  dialog.modal[data-size="md"] {
    --modal-w: 440px;
  }
  dialog.modal[data-size="lg"] {
    --modal-w: 560px;
  }
  dialog.modal.sheet {
    width: min(100%, 960px);
    max-height: 85dvh;
    margin: auto auto 0;
    border-radius: var(--r-float) var(--r-float) 0 0;
  }
  dialog.modal.sheet[open] {
    animation: sheet-in 0.5s var(--ease-out);
  }
  dialog.modal::backdrop {
    background: var(--scrim);
  }
  @supports not ((backdrop-filter: blur(1px)) or (-webkit-backdrop-filter: blur(1px))) {
    dialog.modal {
      background: var(--float-solid);
    }
  }

  .grab {
    flex: none;
    width: 40px;
    height: 4px;
    margin: 8px auto 0;
    border-radius: 4px;
    background: var(--muted);
    opacity: 0.5;
  }
  header {
    flex: none;
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
    padding: 18px 14px 6px 22px;
  }
  .sheet > header {
    padding: 8px 12px 6px 20px;
  }
  dialog.modal h2 {
    margin: 0;
    color: var(--text);
    font: 700 1.15rem/1.3 var(--font);
  }
  .close {
    flex: none;
    display: grid;
    place-items: center;
    width: 44px;
    height: 44px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: transparent;
    color: var(--muted);
    cursor: pointer;
  }
  .close:hover {
    background: var(--hov);
  }
  .body {
    flex: 1 1 auto;
    min-height: 0;
    overflow-y: auto;
    overscroll-behavior: contain;
    padding: 4px 22px 22px;
  }
  .sheet > .body {
    padding-bottom: calc(22px + env(safe-area-inset-bottom));
  }
  footer {
    flex: none;
    display: flex;
    flex-wrap: wrap;
    justify-content: flex-end;
    gap: 10px;
    padding: 0 22px 22px;
  }

  /* Form content for every modal: fills, no borders (DESIGN.md Controls). Used by modal children. */
  dialog.modal :global(label) {
    display: block;
    margin: 0 0 6px;
    color: var(--muted);
    font-size: 0.85rem;
    font-weight: 700;
  }
  dialog.modal :global(input:not([type="checkbox"]):not([type="radio"]):not([type="color"])) {
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    margin: 0 0 14px;
    padding: 0 14px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
    box-shadow: none;
  }
  dialog.modal :global(input::placeholder) {
    color: var(--ctl-sub);
  }
  dialog.modal :global(.btn) {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    box-sizing: border-box;
    width: auto;
    min-height: var(--ctl-h);
    margin: 0;
    padding: 0 22px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: 700 1rem/1 var(--font);
    cursor: pointer;
    transition: background-color 0.15s;
  }
  dialog.modal :global(.btn:hover) {
    background: var(--ctl-h2);
  }
  dialog.modal :global(.btn:active) {
    filter: brightness(0.94);
  }
  dialog.modal :global(.btn.pri) {
    background: var(--accent);
    color: var(--on-accent);
    font-weight: 800;
  }
  dialog.modal :global(.btn.pri:hover) {
    filter: brightness(1.06);
  }
  dialog.modal :global(.btn:disabled) {
    opacity: 0.55;
    cursor: not-allowed;
  }

  @keyframes modal-in {
    from {
      opacity: 0;
      transform: scale(0.97);
    }
  }
  @keyframes sheet-in {
    from {
      opacity: 0;
      transform: translateY(32px);
    }
  }
</style>
