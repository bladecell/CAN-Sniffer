<script lang="ts">
  import { tick } from "svelte";
  import { nextOption } from "$lib/appearance";
  import Icon from "$lib/Icon.svelte";

  interface Props {
    dashboards: { id: string; name: string }[];
    activeId: string;
    onSelect: (id: string) => void;
    /** Opens the name prompt for a new dashboard. The page decides whether the cap allows it. */
    onAdd: () => void;
  }

  let { dashboards, activeId, onSelect, onAdd }: Props = $props();
  let strip = $state<HTMLElement>();

  // Keep the selected tab in view when it changes, including after a rename or a new tab.
  $effect(() => {
    void activeId;
    void dashboards.length;
    tick().then(() => strip?.querySelector<HTMLElement>('[aria-selected="true"]')?.scrollIntoView({ inline: "nearest", block: "nearest" }));
  });

  // Tabs follow the WAI-ARIA tabs pattern: arrows move and select, Home and End jump.
  function onKey(e: KeyboardEvent, id: string) {
    const ids = dashboards.map((d) => d.id);
    let next: string | undefined;
    if (e.key === "Home") next = ids[0];
    else if (e.key === "End") next = ids[ids.length - 1];
    else next = nextOption(ids, id, e.key);
    if (next === undefined) return;
    e.preventDefault();
    onSelect(next);
    strip?.querySelector<HTMLElement>(`[data-tab="${next}"]`)?.focus();
  }
</script>

<div class="db-tabs-row">
  <div class="db-tabs" role="tablist" aria-label="Dashboards" bind:this={strip}>
    {#each dashboards as d (d.id)}
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
    {/each}
  </div>
  <button type="button" class="db-add" aria-label="New dashboard" title="New dashboard" onclick={onAdd}>
    <Icon name="plus" size={22} />
  </button>
</div>

<style>
  .db-tabs-row {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .db-tabs {
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
    min-height: 44px;
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
  .db-add {
    flex: none;
    display: grid;
    place-items: center;
    width: 48px;
    min-height: 48px;
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
