<script lang="ts">
  import type { Snippet } from "svelte";
  import { slide } from "svelte/transition";
  import { accordion } from "$lib/components/accordion";
  import { dragHandle, dragHandleZone, type DndEvent } from "svelte-dnd-action";
  import Icon from "$lib/Icon.svelte";
  import { reduced } from "./flip";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import type { SelectMenuOption } from "$lib/selectMenu";

  export interface ItemRow {
    /** Stable key, unique within the list. */
    id: string;
    name: string;
    unit: string;
    /** PID colour swatch (CSS colour). */
    color?: string;
    /** A second line under the row, in monospace (a CAN frame's data). */
    detail?: string;
    /** Show the name in monospace (a CAN id). */
    mono?: boolean;
  }

  interface Props {
    title: string;
    rows: ItemRow[];
    /** Choices for the add select; disable the ones already used. Without `onAdd` the add row is not shown. */
    options?: SelectMenuOption<string>[];
    max: number;
    /** Rows below this count cannot be removed. */
    min?: number;
    emptyHint?: string;
    onAdd?: (key: string) => void;
    onRemove: (i: number) => void;
    onMove: (i: number, d: number) => void;
    /** Called when a drag ends, with the row ids in their new order. */
    onReorder: (ids: string[]) => void;
    /** Accordion rows: tapping a row's name opens `body` under it. Without `body` rows do not open. */
    body?: Snippet<[ItemRow, number]>;
    openId?: string | null;
    onToggle?: (id: string) => void;
  }

  let { title, rows, options = [], max, min = 0, emptyHint = "No items yet.", onAdd, onRemove, onMove, onReorder, body, openId = null, onToggle }: Props = $props();

  let list = $state<HTMLUListElement>();
  // The order shown while a drag is in flight; null means the rows prop.
  let live = $state<ItemRow[] | null>(null);
  const shown = $derived(live ?? rows);
  const FLIP_MS = reduced() ? 0 : 150;
  // Opening and closing a row eases its height; the detail line and the body swap with the same slide, so the row does not jump.

  const gripKey = (e: KeyboardEvent, i: number) => {
    const d = e.key === "ArrowUp" ? -1 : e.key === "ArrowDown" ? 1 : 0;
    if (!d) return;
    e.preventDefault();
    if (i + d < 0 || i + d >= rows.length) return;
    onMove(i, d);
    queueMicrotask(() => (list?.children[i + d]?.querySelector(".db-grip") as HTMLElement | null)?.focus());
  };

</script>

<div class="db-field db-span">
  <h3 class="db-h3">{title} <span class="tag">{rows.length} of {max}</span></h3>
  {#if rows.length === 0}
    <div class="empty-state">{emptyHint}</div>
  {:else}
    <ul
      class="db-items"
      aria-label="{title}, in display order"
      bind:this={list}
      use:dragHandleZone={{ items: shown, flipDurationMs: FLIP_MS, type: `items-${title}`, dropTargetStyle: {} }}
      onconsider={(e: CustomEvent<DndEvent<ItemRow>>) => (live = e.detail.items)}
      onfinalize={(e: CustomEvent<DndEvent<ItemRow>>) => {
        onReorder(e.detail.items.map((r) => r.id));
        live = null;
      }}
    >
      {#each shown as r, i (r.id)}
        <li class="db-item">
          <button
            type="button"
            class="db-ib db-grip"
            aria-label="Move {r.name}, drag or press the up and down arrow keys"
            title="Drag, or use the arrow keys"
            use:dragHandle
            onkeydown={(e) => gripKey(e, i)}
          >
            <svg viewBox="0 0 24 24" width="20" height="20" fill="currentColor" aria-hidden="true"><circle cx="9" cy="6" r="1.8" /><circle cx="15" cy="6" r="1.8" /><circle cx="9" cy="12" r="1.8" /><circle cx="15" cy="12" r="1.8" /><circle cx="9" cy="18" r="1.8" /><circle cx="15" cy="18" r="1.8" /></svg>
          </button>
          {#snippet label()}
            {#if r.color}<i class="db-sw" style:background={r.color}></i>{/if}
            <span class="db-nm ellipsis" class:mono={r.mono} title={r.name}>{r.name}</span>
            {#if r.unit}<span class="tag">{r.unit}</span>{/if}
          {/snippet}
          {#if body && onToggle}
            <button type="button" class="db-mn db-open" aria-expanded={openId === r.id} aria-controls="db-body-{r.id}" onclick={() => onToggle(r.id)}>
              {@render label()}
            </button>
          {:else}
            <div class="db-mn">{@render label()}</div>
          {/if}
          <div class="db-ac">
            <button type="button" class="db-ib" aria-label="Move {r.name} up" aria-disabled={i === 0} onclick={() => i > 0 && onMove(i, -1)}>
              <Icon name="chevron-down" size={20} class="up" />
            </button>
            <button type="button" class="db-ib" aria-label="Move {r.name} down" aria-disabled={i === rows.length - 1} onclick={() => i < rows.length - 1 && onMove(i, 1)}>
              <Icon name="chevron-down" size={20} />
            </button>
            <button type="button" class="db-ib db-danger" aria-label="Remove {r.name}" aria-disabled={rows.length <= min} onclick={() => rows.length > min && onRemove(i)}>
              <Icon name="trash" size={20} />
            </button>
          </div>
          {#if body && openId === r.id}
            <div class="db-body" id="db-body-{r.id}" transition:slide={accordion()}>{@render body(r, i)}</div>
          {:else if r.detail}
            <div class="db-sub" transition:slide={accordion()}>{r.detail}</div>
          {/if}
        </li>
      {/each}
    </ul>
  {/if}
  {#if onAdd}
  <div class="db-add-row">
    <div class="db-add-pick">
      {#key rows.length}
        <SelectMenu id="db-add" ariaLabel="Add item" placeholder={rows.length >= max ? `Up to ${max} items` : "+ Add item"} {options} onChange={onAdd} disabled={rows.length >= max} />
      {/key}
    </div>
    <span class="db-hint">{rows.length >= max ? `Up to ${max} items` : `${rows.length} of ${max} items`}</span>
  </div>
  {/if}
</div>

<style>
  .db-span {
    grid-column: 1 / -1;
    min-width: 0;
  }
  .db-h3 {
    display: flex;
    align-items: center;
    gap: 8px;
    margin: 0;
    font-size: 1rem;
    font-weight: 800;
  }
  .tag {
    flex: none;
  }
  .db-items {
    display: grid;
    grid-auto-rows: min-content;
    align-content: start;
    gap: 6px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .db-item {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
    padding: 6px 8px 6px 4px;
    border-radius: var(--r-track);
    background: var(--row);
  }
  .db-sub {
    flex: 1 0 100%;
    padding: 0 8px 6px 52px;
    color: var(--muted);
    font-family: var(--mono);
    font-size: 0.85rem;
    font-weight: 600;
    overflow-wrap: anywhere;
  }
  .db-grip {
    background: none;
    cursor: grab;
    touch-action: none;
  }
  .db-mn {
    flex: 1 1 0;
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 7px;
    min-width: 0;
    font-size: 0.95rem;
    font-weight: 800;
  }
  .db-nm {
    min-width: 0;
  }
  .db-open {
    flex-wrap: nowrap;
    min-height: 44px;
    margin: 0;
    padding: 0;
    border: 0;
    background: none;
    color: inherit;
    font: inherit;
    font-size: 0.95rem;
    font-weight: 800;
    text-align: left;
    cursor: pointer;
  }
  .db-open:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .db-open .db-nm {
    text-decoration: underline dotted;
    text-underline-offset: 4px;
  }
  .db-open[aria-expanded="true"] .db-nm {
    color: var(--accent-text);
    text-decoration-style: solid;
  }
  .db-body {
    flex: 1 0 100%;
    padding: 0 8px 8px;
  }
  .db-sw {
    flex: none;
    width: 10px;
    height: 10px;
    border-radius: 50%;
  }
  .db-ac {
    display: flex;
    flex: none;
    align-items: center;
    gap: 4px;
  }
  .db-ib {
    display: grid;
    flex: none;
    place-items: center;
    box-sizing: border-box;
    width: 44px;
    height: 44px;
    min-width: 0;
    min-height: 0;
    aspect-ratio: 1;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: calc(var(--r-ctl) - 2px);
    background: none;
    color: var(--ctl-text);
    cursor: pointer;
  }
  .db-ib:hover {
    background: var(--ctl);
  }
  .db-ib:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .db-ib[aria-disabled="true"] {
    opacity: 0.5;
    cursor: not-allowed;
  }
  .db-danger {
    color: var(--ctl-crit);
  }
  .db-add-row {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 12px;
  }
  .db-add-pick {
    flex: 0 1 220px;
  }
</style>
