<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { closeOnOutsideScroll, fieldBox } from "$lib/popoverAnchor";
  import { portal } from "$lib/layers";

  /** One tile. A tile shows an icon or a symbol on top and its label below. */
  export interface GridOption {
    value: string;
    label: string;
    icon?: string;
    sym?: string;
  }

  /** Shared grid popover for the editor: the Icon and Unit fields. The field shows the chosen tile. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: string;
    options: GridOption[];
    /** Tiles per row. Icons use 4, short unit symbols use 6. */
    columns?: number;
    /** Show each tile's label under it. Off for icons, which keep the label as their accessible name only. */
    showLabels?: boolean;
  }

  let { id, labelId, describedBy, value = $bindable(), options, columns = 4, showLabels = true }: Props = $props();

  const selected = $derived(options.find((o) => o.value === value));
  let trigger: HTMLButtonElement | undefined = $state();
  let pop: HTMLDivElement | undefined = $state();
  const popId = $derived(`${id}-pop`);

  let stopWatch: (() => void) | undefined;

  /** Fixed, aligned to the field's left edge: under the control, or above the field's label when there is no room below. */
  function place() {
    if (!trigger || !pop) return;
    const r = trigger.getBoundingClientRect();
    const field = fieldBox(trigger);
    const width = Math.min(304, window.innerWidth - 16);
    const height = pop.offsetHeight;
    const below = r.bottom + 8;
    pop.style.width = `${width}px`;
    pop.style.left = `${Math.min(Math.max(8, field.left), window.innerWidth - width - 8)}px`;
    pop.style.top = `${below + height > window.innerHeight - 8 ? Math.max(8, field.top - height - 8) : below}px`;
  }

  /** Scrolling the page or the sheet closes the popover: it never drifts from its field. */
  function onToggle(e: Event) {
    if ((e as ToggleEvent).newState === "open") {
      place();
      stopWatch = closeOnOutsideScroll(pop, () => pop?.hidePopover());
    } else {
      stopWatch?.();
      stopWatch = undefined;
      trigger?.focus();
    }
  }

  function choose(next: string) {
    value = next;
    pop?.hidePopover();
  }
</script>

<button
  type="button"
  bind:this={trigger}
  {id}
  class="trig"
  aria-labelledby={`${labelId} ${id}-v`}
  aria-describedby={describedBy}
  aria-haspopup="dialog"
  aria-controls={popId}
  onclick={() => (pop?.matches(":popover-open") ? pop.hidePopover() : pop?.showPopover())}
>
  {#if selected?.icon && value}
    <span class="ico"><Icon name={selected.icon} size={22} /></span>
  {/if}
  <span id={`${id}-v`} class:none={!value}>{selected ? selected.label || selected.sym : value || "None"}</span>
  <Icon name="chevron-down" size={16} class="chev" />
</button>

<!-- Portalled out of the field and the scrolling sheet body (lib/layers.ts): a popover inside either would lose its blur. -->
<div bind:this={pop} use:portal id={popId} class="pop" popover="auto" role="dialog" aria-label={options.length ? "Choose" : "Options"} ontoggle={onToggle}>
  <div class="grid" role="radiogroup" style="--columns: {columns};">
    {#each options as opt (opt.value)}
      <button
        type="button"
        role="radio"
        class="opt"
        aria-checked={value === opt.value}
        class:on={value === opt.value}
        aria-label={opt.label || undefined}
        title={opt.label || undefined}
        onclick={() => choose(opt.value)}
      >
        {#if opt.icon}
          <Icon name={opt.icon} size={26} />
        {:else if opt.sym}
          <span class="sym" class:long={opt.sym.length > 7}>{opt.sym}</span>
        {/if}
        {#if showLabels && opt.label}<span class="lab">{opt.label}</span>{/if}
      </button>
    {/each}
  </div>
</div>

<style>
  .trig {
    display: flex;
    align-items: center;
    gap: 10px;
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    padding: 0 14px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
    font-weight: 600;
    text-align: left;
    cursor: pointer;
  }
  @media (hover: hover) {
    .trig:hover {
      background: var(--ctl-h2);
    }
  }
  /* Pico draws a ring on focused buttons. Focus is a background step only. */
  .trig:focus,
  .trig:focus-visible {
    box-shadow: none;
  }
  /* Keyboard focus is a background step, not a ring. */
  .trig:focus-visible,
  .opt:focus-visible {
    outline: none;
    background: var(--ctl-h2);
  }
  .trig :global(.chev) {
    margin-left: auto;
    flex: none;
  }
  .ico {
    display: grid;
    place-items: center;
    width: 28px;
    height: 28px;
    flex: none;
  }
  .sym {
    font-weight: 600;
    font-size: 1.05rem;
    white-space: nowrap;
  }
  /* Symbols are regular weight and never wrap: a unit is one short word. */
  .opt .sym {
    max-width: 100%;
    font-size: 1.05rem;
    font-weight: 600;
    line-height: 1.1;
    white-space: nowrap;
    text-align: center;
  }
  /* Long symbols such as "grams/sec" shrink to fit the tile instead of breaking inside a word. */
  .opt .sym.long {
    font-size: 0.8rem;
  }
  .none {
    color: var(--ctl-sub);
  }
  /* The UA popover style centres with inset 0 and margin auto, which stretches it. Placed by place(), so reset both. */
  .pop {
    inset: auto;
    box-sizing: border-box;
    width: min(304px, calc(100vw - 16px));
    margin: 0;
    padding: 8px;
    overflow: visible;
    border: 0;
    border-radius: var(--r-float);
    background: var(--float);
    -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
    box-shadow: var(--shadow-float);
    color: var(--text);
  }
  /* Field.svelte resets its controls (margin, slider height). A popover is portalled out of the field, so it resets its own. */
  .pop :global(button) {
    margin: 0 !important;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(var(--columns), minmax(0, 1fr));
    gap: 4px;
  }
  .opt {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 4px;
    /* One tile height for both grids; icon and unit tiles differ only in content. */
    min-height: 56px;
    padding: 6px 2px;
    border: 0;
    border-radius: 14px;
    background: transparent;
    color: var(--text);
    font: inherit;
    font-size: 0.8125rem;
    line-height: 1.2;
    text-align: center;
    cursor: pointer;
  }
  @media (hover: hover) {
    .opt:hover {
      background: var(--hov);
    }
  }
  .opt.on {
    background: color-mix(in oklch, var(--accent) 18%, transparent);
    color: var(--accent-text);
    font-weight: 800;
  }
</style>
