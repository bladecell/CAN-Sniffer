<script module lang="ts">
  let seq = 0;
</script>

<script lang="ts">
  import { tick, type Snippet } from "svelte";

  interface Props {
    /** Button content. Its visible text is its accessible name, so it carries no aria-label. */
    trigger: Snippet;
    /** Panel content. `close` shuts the panel and returns focus to the button. */
    children: Snippet<[() => void]>;
    class?: string;
    /** Names the panel (role=dialog). Not applied to the trigger. */
    ariaLabel?: string;
    /** Extra class on the panel, e.g. the wider filter panel. */
    panelClass?: string;
  }

  let { trigger, children, class: className = "", ariaLabel, panelClass = "" }: Props = $props();

  const panelId = `float-panel-${++seq}`;
  const MARGIN = 8;

  let open = $state(false);
  let rootEl = $state<HTMLDivElement>();
  let triggerEl = $state<HTMLButtonElement>();
  let panelEl = $state<HTMLDivElement>();
  let style = $state("");

  /** Fixed to the viewport beside the trigger. Flips above when there is no room below. */
  function place() {
    if (!triggerEl || !panelEl) return;
    const r = triggerEl.getBoundingClientRect();
    const ph = panelEl.offsetHeight;
    const pw = panelEl.offsetWidth;
    let top = r.bottom + 6;
    if (top + ph > window.innerHeight - MARGIN && r.top - 6 - ph > MARGIN) top = r.top - 6 - ph;
    const left = Math.min(Math.max(MARGIN, r.left), Math.max(MARGIN, window.innerWidth - pw - MARGIN));
    style = `top:${Math.max(MARGIN, top)}px;left:${left}px;min-width:${Math.max(r.width, 160)}px;`;
  }

  async function openPanel() {
    open = true;
    await tick();
    if (!panelEl) return;
    if (!panelEl.matches(":popover-open")) panelEl.showPopover();
    place();
    panelEl.querySelector<HTMLElement>("button:not([disabled]), input")?.focus({ preventScroll: true });
  }

  function close(restoreFocus: boolean) {
    if (!open) return;
    open = false;
    if (restoreFocus) triggerEl?.focus();
  }

  function toggle() {
    if (open) close(true);
    else void openPanel();
  }

  function onPanelKeydown(e: KeyboardEvent) {
    // A nested SelectMenu closes itself first and marks the event handled.
    if (e.key === "Escape" && !e.defaultPrevented) {
      e.preventDefault();
      close(true);
    }
  }

  function onFocusOut(e: FocusEvent) {
    if (!rootEl?.contains(e.relatedTarget as Node | null)) close(false);
  }

  function onWindowPointerDown(e: PointerEvent) {
    if (open && !rootEl?.contains(e.target as Node)) close(false);
  }

  // Follow the trigger when the page or an enclosing scroll container moves.
  $effect(() => {
    if (!open) return;
    const onMove = () => place();
    window.addEventListener("scroll", onMove, { capture: true, passive: true });
    window.addEventListener("resize", onMove);
    return () => {
      window.removeEventListener("scroll", onMove, { capture: true });
      window.removeEventListener("resize", onMove);
    };
  });
</script>

<svelte:window onpointerdown={onWindowPointerDown} />

<div class="float-panel" bind:this={rootEl} onfocusout={onFocusOut}>
  <button
    bind:this={triggerEl}
    type="button"
    class={className}
    aria-haspopup="dialog"
    aria-expanded={open}
    aria-controls={open ? panelId : undefined}
    onclick={toggle}
  >
    {@render trigger()}
  </button>

  {#if open}
    <div
      bind:this={panelEl}
      id={panelId}
      popover="manual"
      class="tel-float panel {panelClass}"
      role="dialog"
      aria-label={ariaLabel}
      tabindex="-1"
      {style}
      onkeydown={onPanelKeydown}
    >
      {@render children(() => close(true))}
    </div>
  {/if}
</div>

<style>
  .float-panel {
    position: relative;
    display: inline-flex;
  }
  .panel {
    position: fixed;
    inset: auto;
    margin: 0;
    box-sizing: border-box;
    padding: 12px;
    width: max-content;
    max-width: min(420px, calc(100vw - 16px));
    max-height: min(600px, calc(100dvh - 24px));
    overflow: auto;
    outline: none;
    z-index: 100;
    animation: panel-in 0.26s var(--ease-out);
  }
  @keyframes panel-in {
    from {
      opacity: 0;
      transform: translateY(-6px) scale(0.98);
    }
  }
  @media (prefers-reduced-motion: reduce) {
    .panel {
      animation: none;
    }
  }
</style>
