<script lang="ts">
  import type { LogRow, IdView } from "$lib/canBus/canModel";
  import type { TableMode } from "$lib/canBus/canBusStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import { relTime } from "$lib/canBus/canFormat";
  import { formatId, idColorIndex, idKey } from "$lib/canBus/canIds";

  type Row = LogRow | IdView;

  interface Props {
    mode: TableMode;
    rows: LogRow[];
    byId: IdView[];
    startUs: number | null;
    selected: number | null;
    onselect: (key: number) => void;
    ondetails: (key: number) => void;
    onmode: (mode: TableMode) => void;
    onclear: () => void;
  }

  let { mode, rows, byId, startUs, selected, onselect, ondetails, onmode, onclear }: Props = $props();

  const count = $derived(mode === "log" ? rows.length : byId.length);
</script>

<section class="card ft" aria-label="Frames">
  <div class="ft-head">
    <div class="seg" role="radiogroup" aria-label="Table">
      <button role="radio" aria-checked={mode === "log"} onclick={() => onmode("log")}>Log</button>
      <button role="radio" aria-checked={mode === "byId"} onclick={() => onmode("byId")}>By ID</button>
    </div>
    <div class="ft-tools">
      <span class="muted">{count} shown</span>
      <button class="btn sm" disabled={count === 0} onclick={onclear}>
        <Icon name="trash" size={16} />
        <span>Clear</span>
      </button>
    </div>
  </div>

  {#if count === 0}
    <p class="muted ft-empty">No frames. Press Start to log the bus.</p>
  {:else}
    <div class="ft-scroll">
      <table class="ft-tbl">
        <thead>
          <tr>
            <th scope="col">Time (s)</th>
            <th scope="col">Direction</th>
            <th scope="col">ID</th>
            <th scope="col" class="hide-s">DLC</th>
            <th scope="col">Data</th>
            <th scope="col">Parsed</th>
          </tr>
        </thead>
        <tbody>
          {#if mode === "log"}
            {#each rows as row (row)}
              {@render line(row)}
            {/each}
          {:else}
            {#each byId as row (row.key)}
              {@render line(row)}
            {/each}
          {/if}
        </tbody>
      </table>
    </div>
  {/if}
</section>

{#snippet line(row: Row)}
  {@const key = idKey(row.id, row.ext)}
  <tr
    tabindex="0"
    class:sel={selected === key}
    onclick={() => onselect(key)}
    ondblclick={() => ondetails(key)}
    onkeydown={(e) => {
      if (e.key === "Enter") ondetails(key);
    }}
  >
    <td class="mono muted">{relTime(row.t, startUs)}</td>
    <td class="dir" class:tx={row.tx}>{row.tx ? "TX" : "RX"}</td>
    <td class="mono id" style:color="var(--c{idColorIndex(key)})">{formatId(row.id, row.ext)}</td>
    <td class="hide-s">{row.data.length}</td>
    <td class="mono data">
      {#if row.rtr}
        <span class="muted">remote request</span>
      {:else}
        {#each Array.from(row.data) as b, i (i)}
          <span class="byte" class:chg={(row.changed & (1 << i)) !== 0} class:z={b === 0}>{b.toString(16).toUpperCase().padStart(2, "0")}</span>
        {/each}
      {/if}
    </td>
    <td class="mono parsed" class:muted={!row.parsed}>{row.parsed || "-"}</td>
  </tr>
{/snippet}

<style>
  .ft {
    padding: 8px 12px;
    min-width: 0;
  }
  .ft-head {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
    padding: 4px 4px 8px;
  }
  .ft-tools {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  .ft-tools .btn {
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  .ft-empty {
    margin: 12px;
  }
  .ft-scroll {
    overflow: auto;
    max-height: 60vh;
  }
  .ft-tbl {
    width: 100%;
    border-collapse: separate;
    border-spacing: 0 4px;
    font-size: 0.92rem;
  }
  .ft-tbl th {
    position: sticky;
    top: 0;
    padding: 6px 12px;
    text-align: left;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    font-size: 0.75rem;
    font-weight: 800;
    color: var(--muted);
    background: var(--card);
    white-space: nowrap;
  }
  .ft-tbl td {
    padding: 9px 12px;
    background: var(--row);
    white-space: nowrap;
    font-variant-numeric: tabular-nums;
  }
  .ft-tbl td:first-child {
    border-radius: var(--r-ctl) 0 0 var(--r-ctl);
    position: relative;
  }
  .ft-tbl td:last-child {
    border-radius: 0 var(--r-ctl) var(--r-ctl) 0;
  }
  .ft-tbl tbody tr {
    cursor: pointer;
  }
  .ft-tbl tbody tr:hover td {
    background: var(--hov);
  }
  .ft-tbl tr.sel td,
  .ft-tbl tr.sel:hover td {
    background: color-mix(in srgb, var(--accent) 24%, var(--hov));
  }
  .id {
    font-weight: 800;
    letter-spacing: 0.02em;
  }
  .byte {
    display: inline-block;
    min-width: 2.1ch;
    padding: 1px 3px;
    border-radius: 6px;
    text-align: center;
  }
  .byte + .byte {
    margin-left: 0.25em;
  }
  .byte.z {
    color: var(--muted);
  }
  .byte.chg {
    background: color-mix(in srgb, var(--accent) 30%, transparent);
    color: var(--accent-text);
    font-weight: 800;
  }
  .parsed:not(.muted) {
    color: var(--accent-text);
  }
  @media (max-width: 859px) {
    .dir {
    font-weight: 600;
    color: var(--ok);
  }
  .dir.tx {
    color: var(--accent-text);
  }
  .hide-s {
      display: none;
    }
  }
</style>
