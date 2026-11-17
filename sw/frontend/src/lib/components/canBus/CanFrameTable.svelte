<script lang="ts">
  import type { LogRow, IdView } from "$lib/canBus/canModel";
  import type { TableMode } from "$lib/canBus/canBusStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import { relTime } from "$lib/canBus/canFormat";
  import { measureHeight, trackUnder } from "$lib/layers";
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

  // The bar sits beside the scroller (Gecko drops the blur inside one), so mirror its horizontal scroll and scrollbar width.
  function syncBar(el: HTMLElement) {
    const host = el.parentElement;
    const sync = () => {
      host?.style.setProperty("--sl", `${-el.scrollLeft}px`);
      host?.style.setProperty("--sbw", `${el.offsetWidth - el.clientWidth}px`);
    };
    el.addEventListener("scroll", sync, { passive: true });
    const ro = new ResizeObserver(sync);
    ro.observe(el);
    sync();
    return () => {
      el.removeEventListener("scroll", sync);
      ro.disconnect();
    };
  }

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
    <div class="ft-body">
      <div class="ft-bar float-layer rest-top" aria-hidden="true" {@attach measureHeight("--hdr-h")}>
        <div class="ft-clip">
        <div class="ft-cols">
          <span>Time (s)</span>
          <span>Direction</span>
          <span>ID</span>
          <span class="hide-s">DLC</span>
          <span>Data</span>
          <span>Parsed</span>
        </div>
        </div>
      </div>
      <div class="ft-scroll" {@attach trackUnder} {@attach syncBar}>
      <table class="ft-tbl">
        <colgroup>
          <col class="w-t" />
          <col class="w-d" />
          <col class="w-i" />
          <col class="w-l hide-s" />
          <col class="w-b" />
          <col />
        </colgroup>
        <thead class="sr-only">
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
  .ft-body {
    --w-t: 84px;
    --w-d: 92px;
    --w-i: 92px;
    --w-l: 56px;
    --w-b: 300px;
    --min: calc(var(--w-t) + var(--w-d) + var(--w-i) + var(--w-l) + var(--w-b) + 140px);
    position: relative;
  }
  .ft-scroll {
    overflow: auto;
    max-height: 60vh;
    border-radius: var(--r-ctl);
    padding-top: var(--hdr-h, 30px);
  }
  .ft-bar {
    position: absolute;
    top: 0;
    left: 0;
    right: var(--sbw, 0px);
    z-index: 2;
    border-radius: var(--r-ctl);
    box-shadow: none;
    pointer-events: none;
  }
  .ft-clip {
    overflow: hidden;
    border-radius: inherit;
  }
  .ft-cols {
    display: grid;
    grid-template-columns: var(--w-t) var(--w-d) var(--w-i) var(--w-l) var(--w-b) 1fr;
    min-width: var(--min);
    margin-left: var(--sl, 0px);
  }
  .ft-cols span {
    padding: 6px 12px;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    font-size: 0.75rem;
    font-weight: 800;
    color: var(--muted);
    white-space: nowrap;
  }
  .ft-tbl {
    width: 100%;
    min-width: var(--min);
    table-layout: fixed;
    border-collapse: separate;
    border-spacing: 0 4px;
    font-size: 0.92rem;
  }
  .w-t {
    width: var(--w-t);
  }
  .w-d {
    width: var(--w-d);
  }
  .w-i {
    width: var(--w-i);
  }
  .w-l {
    width: var(--w-l);
  }
  .w-b {
    width: var(--w-b);
  }
  .sr-only {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip-path: inset(50%);
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
  .dir {
    font-weight: 600;
    color: var(--ok);
  }
  .dir.tx {
    color: var(--accent-text);
  }
  @media (max-width: 859px) {
    .hide-s {
      display: none;
    }
    .ft-body {
      --min: calc(var(--w-t) + var(--w-d) + var(--w-i) + var(--w-b) + 140px);
    }
    .ft-cols {
      grid-template-columns: var(--w-t) var(--w-d) var(--w-i) var(--w-b) 1fr;
    }
  }
</style>
