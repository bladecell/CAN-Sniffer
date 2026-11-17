<script lang="ts">
  import type { IdRow } from "$lib/canBus/canBusStore.svelte";
  import { ACTIVE_MS } from "$lib/canBus/canModel";
  import { formatId, idColorIndex } from "$lib/canBus/canIds";
  import { hzText } from "$lib/canBus/canFormat";

  interface Props {
    ids: IdRow[];
    now: number;
    selected: number | null;
    onselect: (key: number) => void;
    ondetails: (key: number) => void;
  }

  let { ids, now, selected, onselect, ondetails }: Props = $props();

  let text = $state("");
  const needle = $derived(text.replace(/\s/g, "").toUpperCase());
  const shown = $derived(needle ? ids.filter((r) => formatId(r.id, r.ext).includes(needle)) : ids);
  const active = $derived(shown.filter((r) => now - r.seenMs < ACTIVE_MS));
  const silent = $derived(shown.filter((r) => now - r.seenMs >= ACTIVE_MS));
  const silentTotal = $derived(ids.filter((r) => now - r.seenMs >= ACTIVE_MS).length);
</script>

<div class="cl">
  <div class="cl-top">
    <div>
      <h2>IDs</h2>
      <p class="muted">{ids.length} seen · {silentTotal} silent. Click selects, double-click opens details.</p>
    </div>
    <button class="btn sm" disabled={selected === null} onclick={() => selected !== null && ondetails(selected)}>Details</button>
  </div>
  <input class="in mono" placeholder="Filter, e.g. 7E8" aria-label="Filter IDs" bind:value={text} />

  <div class="cl-list">
    {#if ids.length === 0}
      <p class="muted">No frames yet.</p>
    {:else if shown.length === 0}
      <p class="muted">No ID matches.</p>
    {/if}

    {#if active.length > 0}
      <h3 class="cl-grp">Active <span>{active.length}</span></h3>
    {/if}
    {#each active as r (r.key)}
      {@render idRow(r)}
    {/each}

    {#if silent.length > 0}
      <h3 class="cl-grp">Silent <span>{silent.length}</span></h3>
    {/if}
    {#each silent as r (r.key)}
      {@render idRow(r, true)}
    {/each}
  </div>
</div>

{#snippet idRow(r: IdRow, quiet = false)}
  <button
    class="cl-row"
    class:quiet
    aria-current={selected === r.key ? "true" : undefined}
    style:--c="var(--c{idColorIndex(r.key)})"
    onclick={() => onselect(r.key)}
    ondblclick={() => ondetails(r.key)}
  >
    <i aria-hidden="true"></i>
    <span class="cl-id">
      <b class="mono">{formatId(r.id, r.ext)}</b>
      <small>{r.name || (r.ext ? "extended" : "standard")}</small>
    </span>
    <em>{quiet ? "silent" : hzText(r.hz)}</em>
  </button>
{/snippet}

<style>
  .cl {
    display: grid;
    gap: 12px;
    min-width: 0;
  }
  .cl-top {
    display: flex;
    align-items: flex-start;
    justify-content: space-between;
    gap: 10px;
  }
  .cl-top h2 {
    margin: 0;
  }
  .cl-top p {
    margin: 2px 0 0;
    font-size: 0.88rem;
  }
  .cl-list {
    display: grid;
    gap: 6px;
    align-content: start;
    max-height: min(56vh, 520px);
    overflow: auto;
    padding-right: 2px;
  }
  .cl-grp {
    display: flex;
    gap: 8px;
    margin: 6px 2px 2px;
    font-size: 0.85rem;
    font-weight: 700;
  }
  .cl-grp span {
    margin-left: auto;
    color: var(--muted);
  }
  .cl-row {
    position: relative;
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 12px;
    width: 100%;
    min-height: 48px;
    padding: 8px 14px 8px 10px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--row);
    color: var(--text);
    text-align: left;
    font: inherit;
  }
  .cl-row i {
    align-self: stretch;
    width: 4px;
    flex: none;
    border-radius: 4px;
    background: var(--c);
  }
  .cl-row.quiet i {
    opacity: 0.4;
  }
  .cl-id {
    flex: 1;
    display: grid;
    min-width: 0;
  }
  .cl-id b {
    font-size: 0.95rem;
  }
  .cl-id small {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 600;
  }
  .cl-row em {
    flex-shrink: 0;
    white-space: nowrap;
    font-style: normal;
    font-variant-numeric: tabular-nums;
    font-weight: 700;
    color: var(--muted);
  }
  .cl-row.quiet b {
    color: var(--muted);
  }
  .cl-row[aria-current='true'] {
    background: color-mix(in srgb, var(--accent) 24%, var(--hov));
  }
  .cl-row[aria-current='true'] b {
    color: var(--accent-text);
  }
  @media (hover: hover) {
    .cl-row:hover {
      background: var(--hov);
    }
  }
</style>
