<script lang="ts">
  import type { LogRow, IdView } from "$lib/canBus/canModel";
  import type { TableMode } from "$lib/canBus/canBusStore.svelte";
  import { relTime } from "$lib/canBus/canFormat";
  import { formatId, idKey } from "$lib/canBus/canIds";

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
  }

  let { mode, rows, byId, startUs, selected, onselect, ondetails, onmode }: Props = $props();

  const count = $derived(mode === "log" ? rows.length : byId.length);
</script>

<section class="card ft" aria-label="Frames">
  <div class="ft-head">
    <div class="seg" role="radiogroup" aria-label="Table">
      <button role="radio" aria-checked={mode === "log"} onclick={() => onmode("log")}>Log</button>
      <button role="radio" aria-checked={mode === "byId"} onclick={() => onmode("byId")}>By ID</button>
    </div>
    <span class="muted">{count} shown</span>
  </div>

  {#if count === 0}
    <p class="muted ft-empty">No frames yet. The table fills while the bus talks.</p>
  {:else}
    <div class="ft-scroll">
      <table class="ft-tbl">
        <thead>
          <tr>
            <th scope="col">Time (s)</th>
            <th scope="col">ID</th>
            <th scope="col" class="hide-s">DLC</th>
            <th scope="col">Data</th>
            <th scope="col">Parsed</th>
            <th scope="col" class="hide-s">Source</th>
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
    <td class="num muted">{relTime(row.t, startUs)}</td>
    <td class="mono id">{formatId(row.id, row.ext)}</td>
    <td class="hide-s num">{row.data.length}</td>
    <td class="mono data">
      {#if row.rtr}
        <span class="muted">remote request</span>
      {:else}
        {#each Array.from(row.data) as b, i (i)}
          <span class:chg={(row.changed & (1 << i)) !== 0}>{b.toString(16).toUpperCase().padStart(2, "0")}</span>
        {/each}
      {/if}
    </td>
    <td class="t-acc mono">{row.parsed}</td>
    <td class="hide-s muted">{row.tx ? "TX" : "RX"}</td>
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
    padding: 6px 10px;
    text-align: left;
    font-size: 0.85rem;
    font-weight: 700;
    color: var(--muted);
    background: var(--card);
    white-space: nowrap;
  }
  .ft-tbl td {
    padding: 6px 10px;
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
  .ft-tbl tr.sel td:first-child::before {
    content: '';
    position: absolute;
    left: 0;
    top: 24%;
    bottom: 24%;
    width: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .num {
    text-align: right;
  }
  .id {
    font-weight: 700;
  }
  .data span {
    display: inline-block;
    min-width: 2ch;
  }
  .data span + span {
    margin-left: 0.4em;
  }
  .data .chg {
    color: var(--accent-text);
    font-weight: 800;
  }
  @media (max-width: 859px) {
    .hide-s {
      display: none;
    }
  }
</style>
