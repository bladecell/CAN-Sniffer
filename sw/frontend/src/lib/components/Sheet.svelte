<script lang="ts">
  import type { Snippet } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { overlay } from "$lib/overlay";
  import { measureHeight } from "$lib/layers";
  import { clampSheetHeight, defaultSheetHeight, stepSheetHeight } from "$lib/editor/sheetHeight";
  import "./overlay.css";

  interface Props {
    /** One-way: the parent owns it. A dismiss calls onClose, and the parent decides whether to close. */
    open?: boolean;
    title: string;
    /** One line under the title (mockup: "Fill in each section, then save"). */
    subtitle?: string;
    onClose?: () => void;
    dismissible?: boolean;
    /** Opt-in: the grab handle drags (mouse, touch) or takes Arrow Up/Down to set the height. Starts at 65% of the screen on each open. */
    resizable?: boolean;
    children: Snippet;
    footer?: Snippet;
    /** Rendered inside the header's float layer, under the title (the editor's section tabs). One layer with the header. */
    head?: Snippet;
    /** The body scrolls (default). Set false when the content scrolls itself, as the phone PID sidebar does. */
    scrolls?: boolean;
    /** A 390px phone frame, centred on wider screens (the mockup's phone view). */
    narrow?: boolean;
  }

  let { open = false, title, subtitle, onClose, dismissible = true, resizable = false, children, footer, head, scrolls = true, narrow = false }: Props = $props();
  const uid = $props.id();
  const titleId = `${uid}-title`;

  let height = $state<number | undefined>(undefined);
  let drag: { y: number; h: number } | null = null;
  const pct = $derived(height === undefined ? 0 : Math.round((height / window.innerHeight) * 100));

  $effect(() => {
    if (open && resizable) height = defaultSheetHeight(window.innerHeight);
  });

  function onGrabDown(e: PointerEvent) {
    if (height === undefined) return;
    drag = { y: e.clientY, h: height };
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
  }
  function onGrabMove(e: PointerEvent) {
    if (!drag) return;
    height = clampSheetHeight(drag.h + (drag.y - e.clientY), window.innerHeight);
  }
  function onGrabUp() {
    drag = null;
  }
  function onGrabKey(e: KeyboardEvent) {
    if (height === undefined || (e.key !== "ArrowUp" && e.key !== "ArrowDown")) return;
    e.preventDefault();
    height = stepSheetHeight(height, e.key === "ArrowUp" ? "up" : "down", window.innerHeight);
  }
</script>

<!-- The dialog itself is neither a scroller nor blurred: its layers sit beside the body (layers.ts). -->
<dialog
  use:overlay={{ open, dismissible, onClose }}
  class="overlay sheet"
  class:resizable
  class:flat={!scrolls}
  class:narrow
  style:height={resizable && height !== undefined ? `${height}px` : undefined}
  aria-labelledby={titleId}
>
  <!-- One float layer: grabber, title, close and tabs share one background, over the top of the sheet. -->
  <div class="sheet-top float-layer" {@attach measureHeight("--sheet-head")}>
    {#if resizable}
      <!-- svelte-ignore a11y_no_noninteractive_tabindex, a11y_no_noninteractive_element_interactions -->
      <div
        class="grab handle"
        role="separator"
        tabindex="0"
        aria-orientation="horizontal"
        aria-label="Sheet height"
        aria-valuemin="30"
        aria-valuemax="85"
        aria-valuenow={pct}
        aria-valuetext={`${pct}% of screen height`}
        onpointerdown={onGrabDown}
        onpointermove={onGrabMove}
        onpointerup={onGrabUp}
        onpointercancel={onGrabUp}
        onkeydown={onGrabKey}
      ></div>
    {:else}
      <div class="grab" aria-hidden="true"></div>
    {/if}
    <header>
      <div class="titles">
        <h2 id={titleId}>{title}</h2>
        {#if subtitle}<p>{subtitle}</p>{/if}
      </div>
      {#if dismissible}
        <button type="button" class="close" aria-label="Close" onclick={() => onClose?.()}>
          <Icon name="x" size={22} />
        </button>
      {/if}
      {#if head}
        <div class="sheet-head">{@render head()}</div>
      {/if}
    </header>
  </div>
  {#if open}
    <div class="body">{@render children()}</div>
    {#if footer}
      <footer {@attach measureHeight("--sheet-foot")}>{@render footer()}</footer>
    {/if}
  {/if}
</dialog>

<style>
  /* Top block: grabber, title, close and tabs in one float layer, over the top of the sheet. */
  .sheet-top {
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    z-index: 2;
    border-radius: var(--r-float) var(--r-float) 0 0;
    box-shadow: none;
  }
  .grab {
    position: relative;
    display: block;
    flex: none;
    width: 100%;
    height: 28px;
    margin: 0;
    padding: 0;
    border: 0;
    background: none;
  }
  .grab::before {
    content: "";
    position: absolute;
    left: 50%;
    top: 11px;
    width: 48px;
    height: 5px;
    margin-left: -24px;
    border-radius: 3px;
    background: var(--muted);
    opacity: 0.6;
  }
  .grab.handle {
    touch-action: none;
    cursor: ns-resize;
  }
  header {
    flex-wrap: wrap;
    gap: 0 12px;
    align-items: flex-start;
    padding: 0 20px 0 24px;
    background: none;
    box-shadow: none;
  }
  .titles {
    flex: 1 1 auto;
    min-width: 0;
    display: grid;
  }
  .titles h2 {
    margin: 0;
    font-size: 1.3rem;
    line-height: 1.2;
    font-weight: 800;
  }
  .titles p {
    margin: 0;
    font-size: 0.875rem;
    color: var(--muted);
  }
  .close {
    flex: none;
    width: 44px;
    height: 44px;
    margin: 0;
    border-radius: 50%;
    background: var(--ctl);
  }
  .sheet-head {
    flex: 0 0 100%;
    padding: 8px 0;
  }
  /* Phone frame: title, close and tabs share a 12px inset, as in the mockup. */
  .narrow header {
    padding: 0 12px;
  }
  .narrow .sheet-head {
    padding: 8px 0 0;
  }
  /* The sheet scrolls without a visible scrollbar. */
  :global(dialog.sheet .body) {
    scrollbar-width: none;
  }
  :global(dialog.sheet .body::-webkit-scrollbar) {
    display: none;
  }
</style>
