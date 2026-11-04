<script lang="ts">
  import { dragHandleZone, type DndEvent } from "svelte-dnd-action";
  import { dashboardDocStore } from "$lib/dashboardDocStore.svelte";
  import type { Card, Section } from "$lib/dashboardModel";
  import { type SectionRef } from "$lib/dashboardGrid";
  import Icon from "$lib/Icon.svelte";
  import DashCard from "./DashCard.svelte";
  import { playCards, snapshotCards } from "./flip";

  interface Props {
    section: Section;
    index: number;
    count: number;
    sections: SectionRef[];
    editing: boolean;
    phone: boolean;
    /** Called when a drag ends in this zone with the zone's final card list. The page commits it. */
    onDrop: (sectionId: string, items: Card[]) => void;
    /** Opens the add-card sheet for this section. */
    onAdd: (sectionId: string) => void;
    onEdit: (cardId: string) => void;
    onSize: (cardId: string) => void;
    onError: (message: string) => void;
  }

  let { section, index, count, sections, editing, phone, onDrop, onAdd, onEdit, onSize, onError }: Props = $props();

  // Reordering animates the other cards aside; reduced motion turns it off.
  const reduceMotion = typeof window !== "undefined" && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const FLIP_MS = reduceMotion ? 0 : 150;
  // The list the zone shows while a drag is in flight. Null means the store's list.
  let live = $state<Card[] | null>(null);
  const items = $derived(live ?? section.cards);

  // Resizing or moving a card animates it and the cards around it to their new places. A drag has its own animation.
  let grid = $state<HTMLElement>();
  let before: ReturnType<typeof snapshotCards> | null = null;
  let afterDrop = false;
  const layoutKey = $derived(items.map((c) => `${c.id}:${c.w}x${c.h}`).join(","));
  $effect.pre(() => {
    void layoutKey;
    before = grid && live === null && !afterDrop ? snapshotCards(grid) : null;
  });
  $effect(() => {
    void layoutKey;
    if (grid && before) playCards(grid, before);
    before = null;
    afterDrop = false;
  });
  const sectionLabel = $derived(section.title || `Section ${index + 1}`);
  let confirmDelete = $state(false);

  function rename(title: string) {
    const err = dashboardDocStore.renameSection(section.id, title);
    if (err) onError(err);
  }
  function shift(delta: number) {
    const err = dashboardDocStore.moveSection(section.id, index + delta);
    if (err) onError(err);
  }
  function remove() {
    if (section.cards.length > 0 && !confirmDelete) {
      confirmDelete = true;
      return;
    }
    confirmDelete = false;
    const err = dashboardDocStore.removeSection(section.id);
    if (err) onError(err);
  }
</script>

<section class="db-section" class:editing aria-label={sectionLabel}>
  {#if editing}
    <div class="db-sec-edit">
      <input
        class="db-input db-sec-title"
        type="text"
        maxlength="60"
        placeholder="Section title (optional)"
        aria-label="Title for section {index + 1}"
        value={section.title}
        onchange={(e) => rename(e.currentTarget.value)}
      />
      <button type="button" class="db-icon" aria-label="Move section {index + 1} up" aria-disabled={index === 0} onclick={() => index > 0 && shift(-1)}>
        <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true"><path d="M6 15l6-6 6 6" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" /></svg>
      </button>
      <button type="button" class="db-icon" aria-label="Move section {index + 1} down" aria-disabled={index >= count - 1} onclick={() => index < count - 1 && shift(1)}>
        <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true"><path d="M6 9l6 6 6-6" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" /></svg>
      </button>
      <button type="button" class="db-icon db-icon-danger" aria-label="Delete section {index + 1}" onclick={remove}>
        <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true"><path d="M5 7h14M10 7V5h4v2M8 7l1 12h6l1-12" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" /></svg>
      </button>
    </div>
    {#if confirmDelete}
      <div class="db-sec-confirm" role="alert">
        <span>This removes {section.cards.length} {section.cards.length === 1 ? "card" : "cards"} with it.</span>
        <button type="button" class="rv-btn sm" onclick={() => (confirmDelete = false)}>Keep</button>
        <button type="button" class="rv-btn sm fill-crit" onclick={remove}>Delete section</button>
      </div>
    {/if}
  {:else if section.title}
    <h2 class="db-sec-title-view">{section.title}</h2>
  {/if}

  <div
    bind:this={grid}
    class="db-grid"
    class:db-drop={editing}
    use:dragHandleZone={{ items, flipDurationMs: FLIP_MS, type: "cards", dragDisabled: !editing }}
    onconsider={(e: CustomEvent<DndEvent<Card>>) => (live = e.detail.items)}
    onfinalize={(e: CustomEvent<DndEvent<Card>>) => {
      afterDrop = true;
      onDrop(section.id, e.detail.items);
      live = null;
      setTimeout(() => (afterDrop = false), 0);
    }}
  >
    {#each items as card (card.id)}
      <DashCard
        {card}
        {editing}
        {phone}
        sectionId={section.id}
        index={section.cards.findIndex((c) => c.id === card.id)}
        sectionIndex={index}
        {sections}
        {onEdit}
        {onSize}
        {onError}
      />
    {/each}
  </div>
  {#if !editing && section.cards.length === 0}
    <p class="db-empty">No cards in this section yet.</p>
  {/if}
  {#if editing}
    <button type="button" class="db-sec-add" onclick={() => onAdd(section.id)}>
      <Icon name="plus" size={18} />Add card
    </button>
  {/if}
</section>

<style>
  .db-section {
    display: grid;
    gap: 10px;
    min-width: 0;
    align-content: start;
  }
  /* Edit mode frames each section as a panel (mockup); view mode shows the cards on the page itself. */
  .db-section.editing {
    padding: 12px;
    border-radius: 28px;
    background: var(--row);
  }
  .db-sec-edit {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
  }
  .db-sec-title {
    flex: 1 1 auto;
    min-width: 0;
  }
  .db-icon-danger {
    color: var(--crit);
  }
  .db-sec-confirm {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
    color: var(--text);
    font-size: 0.9rem;
  }
  .db-sec-title-view {
    margin: 0 4px;
    color: var(--text);
    font-size: 1rem;
    font-weight: 800;
  }
  /* Edit mode shows the six columns the cards snap to (mockup). */
  .db-drop {
    border-radius: 18px;
    background-image: repeating-linear-gradient(
      to right,
      color-mix(in srgb, var(--text) 6%, transparent) 0 calc((100% - 60px) / 6),
      transparent calc((100% - 60px) / 6) calc((100% - 60px) / 6 + 12px)
    );
  }
  .db-sec-add {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    min-height: 48px;
    width: 100%;
    border: 0;
    border-radius: 18px;
    background: var(--ctl);
    color: var(--ctl-text);
    font: 800 0.95rem/1 var(--font);
    cursor: pointer;
  }
  .db-sec-add:hover {
    background: var(--ctl-h2);
  }
  .db-empty {
    margin: 0;
    padding: 0 8px;
    color: var(--muted);
    font-size: 0.9rem;
  }
</style>
