<script lang="ts">
  import { tick } from "svelte";
  import { tabTarget } from "$lib/tabNav";
  import Icon from "$lib/Icon.svelte";

  interface Props {
    dashboards: { id: string; name: string }[];
    activeId: string;
    onSelect: (id: string) => void;
    /** Opens the name prompt for a new dashboard. The page decides whether the cap allows it. */
    onAdd: () => void;
    /** Edit mode: the selected tab shows a pencil that opens the edit dialog. */
    editing?: boolean;
    onEdit?: () => void;
  }

  let { dashboards, activeId, onSelect, onAdd, editing = false, onEdit }: Props = $props();
  let strip = $state<HTMLElement>();

  // Keep the selected tab in view when it changes, including after a rename or a new tab.
  $effect(() => {
    void activeId;
    void dashboards.length;
    tick().then(() => strip?.querySelector<HTMLElement>(`[data-tab="${activeId}"]`)?.scrollIntoView({ inline: "nearest", block: "nearest" }));
  });

  // Tabs follow the WAI-ARIA tabs pattern: arrows move and select, Home and End jump.
  function onKey(e: KeyboardEvent, id: string) {
    const next = tabTarget(dashboards.map((d) => d.id), id, e.key);
    if (next === undefined) return;
    e.preventDefault();
    onSelect(next);
    strip?.querySelector<HTMLElement>(`[data-tab="${next}"]`)?.focus();
  }
</script>

<div class="db-tabs-row">
  <div class="db-tabs" role="tablist" aria-label="Dashboards" bind:this={strip}>
    {#each dashboards as d (d.id)}
      {#if editing && d.id === activeId}
        <div class="db-tab-cur">
          <button
            type="button"
            role="tab"
            class="db-tab"
            data-tab={d.id}
            aria-selected="true"
            tabindex="0"
            onclick={() => onSelect(d.id)}
            onkeydown={(e) => onKey(e, d.id)}>{d.name}</button
          >
          <button type="button" class="db-tab-pencil" aria-label="Edit dashboard" title="Edit dashboard" onclick={onEdit}>
            <Icon name="edit" size={18} />
          </button>
        </div>
      {:else}
        <button
          type="button"
          role="tab"
          class="db-tab"
          data-tab={d.id}
          aria-selected={d.id === activeId}
          tabindex={d.id === activeId ? 0 : -1}
          onclick={() => onSelect(d.id)}
          onkeydown={(e) => onKey(e, d.id)}>{d.name}</button
        >
      {/if}
    {/each}
  </div>
  {#if editing}
    <button type="button" class="db-add" aria-label="New dashboard" title="New dashboard" onclick={onAdd}>
      <Icon name="plus" size={22} />
    </button>
  {/if}
</div>

<style>
  .db-tabs-row {
    display: flex;
    align-items: stretch;
    height: 52px;
    gap: 8px;
    min-width: 0;
  }
  .db-tabs {
    box-sizing: border-box;
    align-items: center;
    flex: 1 1 auto;
    display: flex;
    gap: 2px;
    min-width: 0;
    padding: 3px 5px;
    scroll-padding-inline: 8px;
    border-radius: 18px;
    background: var(--ctl);
    overflow-x: auto;
    scrollbar-width: none;
    -webkit-overflow-scrolling: touch;
  }
  .db-tabs::-webkit-scrollbar {
    display: none;
  }
  .db-tab {
    position: relative;
    flex: none;
    height: 44px;
    margin: 0;
    padding: 0 16px;
    border: 0;
    border-radius: 15px;
    background: none;
    color: var(--ctl-text);
    font: 700 0.95rem/1 var(--font);
    white-space: nowrap;
    cursor: pointer;
  }
  .db-tab[aria-selected="true"] {
    font-weight: 800;
    background: var(--float-solid);
  }
  .db-tab[aria-selected="true"]::after {
    content: "";
    position: absolute;
    left: 22%;
    right: 22%;
    bottom: 3px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  /* Edit mode: the selected tab is one pill holding the name and the pencil, so the pill and underline move to the wrapper. */
  .db-tab-cur {
    position: relative;
    display: flex;
    align-items: center;
    flex: none;
    height: 44px;
    border-radius: 15px;
    background: var(--float-solid);
  }
  .db-tab-cur::after {
    content: "";
    position: absolute;
    left: 22%;
    right: 22%;
    bottom: 3px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .db-tab-cur > .db-tab[aria-selected="true"] {
    padding-right: 4px;
    background: none;
    color: var(--text);
  }
  .db-tab-cur > .db-tab[aria-selected="true"]::after {
    content: none;
  }
  .db-tab-pencil {
    flex: none;
    display: grid;
    place-items: center;
    width: 44px;
    height: 44px;
    margin: 0 4px 0 0;
    padding: 0;
    border: 0;
    border-radius: 15px;
    background: none;
    color: var(--text);
    cursor: pointer;
  }
  .db-tab-pencil:hover,
  .db-tab-pencil:active,
  .db-tab-pencil:focus {
    background: none;
    box-shadow: none;
  }
  .db-tab-pencil:focus-visible {
    outline: 2px solid var(--accent);
    outline-offset: -2px;
  }
  .db-add {
    flex: none;
    display: grid;
    place-items: center;
    width: 52px;
    min-height: 0;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: 16px;
    background: var(--ctl);
    color: var(--ctl-text);
    cursor: pointer;
  }
  .db-add:hover {
    background: var(--ctl-h2);
  }
</style>
