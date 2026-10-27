<script lang="ts">
  import type { Snippet } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { overlay } from "$lib/overlay";
  import { clampSheetHeight, defaultSheetHeight, stepSheetHeight } from "$lib/editor/sheetHeight";
  import "./overlay.css";

  interface Props {
    /** One-way: the parent owns it. A dismiss calls onClose, and the parent decides whether to close. */
    open?: boolean;
    title: string;
    onClose?: () => void;
    dismissible?: boolean;
    /** Opt-in: the grab handle drags (mouse, touch) or takes Arrow Up/Down to set the height. Starts at 65% of the screen on each open. */
    resizable?: boolean;
    children: Snippet;
    footer?: Snippet;
  }

  let { open = false, title, onClose, dismissible = true, resizable = false, children, footer }: Props = $props();
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

<dialog
  use:overlay={{ open, dismissible, onClose }}
  class="overlay sheet"
  class:resizable
  style:height={resizable && height !== undefined ? `${height}px` : undefined}
  aria-labelledby={titleId}
>
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
  <!-- The header sticks to the top of the scrolling sheet. Sticky rows under it (editor tabs) read --sheet-head. -->
  <header
    class="float-layer"
    {@attach (el) => {
      const dialog = el.closest("dialog");
      const sync = () => dialog?.style.setProperty("--sheet-head", `${el.offsetHeight}px`);
      const ro = new ResizeObserver(sync);
      ro.observe(el);
      sync();
      return () => ro.disconnect();
    }}
  >
    <h2 id={titleId}>{title}</h2>
    {#if dismissible}
      <button type="button" class="close" aria-label="Close" onclick={() => onClose?.()}>
        <Icon name="x" size={20} />
      </button>
    {/if}
  </header>
  {#if open}
    <div class="body">{@render children()}</div>
    {#if footer}
      <footer class="float-layer">{@render footer()}</footer>
    {/if}
  {/if}
</dialog>
