<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { closeOnOutsideScroll } from "$lib/popoverAnchor";
  import { portal } from "$lib/layers";

  interface Props {
    /** The hint text (from sidebarHints.ts). */
    text: string;
    /** Sits in a text line (next to a field name) instead of the sidebar's top-right corner. */
    inline?: boolean;
    label?: string;
  }

  let { text, inline = false, label = "How this page works" }: Props = $props();

  const uid = $props.id();
  const popId = `${uid}-hint`;
  let trigger: HTMLButtonElement | undefined = $state();
  let pop: HTMLDivElement | undefined = $state();
  let pointer = "mouse";
  let stopWatch: (() => void) | undefined;

  /** Right-aligned under the button; flips above when there is no room below. */
  function place() {
    if (!trigger || !pop) return;
    const r = trigger.getBoundingClientRect();
    const width = Math.min(264, window.innerWidth - 16);
    const height = pop.offsetHeight;
    const below = r.bottom + 8;
    pop.style.width = `${width}px`;
    pop.style.left = `${Math.max(8, Math.min(r.right - width, window.innerWidth - width - 8))}px`;
    pop.style.top = `${below + height > window.innerHeight - 8 ? Math.max(8, r.top - height - 8) : below}px`;
  }

  function show() {
    if (pop && !pop.matches(":popover-open")) pop.showPopover();
  }
  function hide() {
    if (pop?.matches(":popover-open")) pop.hidePopover();
  }

  function onToggle(e: Event) {
    if ((e as ToggleEvent).newState === "open") {
      place();
      stopWatch = closeOnOutsideScroll(pop, hide);
    } else {
      stopWatch?.();
      stopWatch = undefined;
    }
  }

  // Mouse: hover opens it. Touch and pen: a tap toggles it. Keyboard: focus opens it, Enter and Space toggle it.
  function onDown(e: PointerEvent) {
    pointer = e.pointerType;
  }
  function onClick(e: MouseEvent) {
    if (pointer === "mouse" && e.detail > 0) return;
    if (pop?.matches(":popover-open")) hide();
    else show();
  }
  function onFocus(e: FocusEvent) {
    if ((e.target as HTMLElement).matches(":focus-visible")) show();
  }
</script>

<button
  type="button"
  bind:this={trigger}
  class="hint"
  class:inline
  aria-label={label}
  aria-controls={popId}
  aria-describedby={popId}
  aria-expanded="false"
  onpointerdown={onDown}
  onpointerenter={(e) => e.pointerType === "mouse" && show()}
  onpointerleave={(e) => e.pointerType === "mouse" && hide()}
  onclick={onClick}
  onfocus={onFocus}
  onblur={hide}
>
  <Icon name="circle-info" size={inline ? 16 : 20} />
</button>
<!-- Portalled out of the blurred header: a blurred ancestor would drop this popover's blur (lib/layers.ts). -->
<div bind:this={pop} use:portal id={popId} class="pop" popover="auto" role="tooltip" ontoggle={onToggle}>{text}</div>

<style>
  .hint {
    position: absolute;
    top: 6px;
    right: 6px;
    display: grid;
    place-items: center;
    width: 44px;
    height: 44px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: 12px;
    background: none;
    color: var(--muted);
    cursor: pointer;
  }
  .hint.inline {
    position: static;
    flex: none;
    width: 24px;
    height: 24px;
    border-radius: 8px;
  }
  @media (hover: hover) {
    .hint:hover {
      color: var(--accent-text);
    }
  }
  /* Mouse and tap focus show nothing; keyboard focus is a background step. */
  .hint:focus {
    outline: none;
    box-shadow: none;
  }
  .hint:focus-visible {
    background: var(--hov);
  }
  .pop {
    inset: auto;
    box-sizing: border-box;
    width: min(264px, calc(100vw - 16px));
    margin: 0;
    padding: 12px 14px;
    border: 0;
    border-radius: var(--r-float);
    background: var(--float);
    -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
    box-shadow: var(--shadow-float);
    color: var(--text);
    font-size: 0.875rem;
    font-weight: 600;
    line-height: 1.45;
    outline: none;
  }
  @supports not ((backdrop-filter: blur(1px)) or (-webkit-backdrop-filter: blur(1px))) {
    .pop {
      background: var(--float-solid);
    }
  }
</style>
