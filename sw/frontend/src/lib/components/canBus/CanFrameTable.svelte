<script lang="ts">
  import type { LogRow } from "$lib/canBus/canModel";
  import { relTime } from "$lib/canBus/canFormat";
  import { formatId, hexBytes, idKey } from "$lib/canBus/canIds";

  interface Props {
    rows: LogRow[];
    startUs: number | null;
    selected: number | null;
    onselect: (key: number) => void;
  }

  let { rows, startUs, selected, onselect }: Props = $props();
</script>

<section class="card ft" aria-label="Frames">
  {#if rows.length === 0}
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
          {#each rows as row (row)}
            {@const key = idKey(row.id, row.ext)}
            <tr
              tabindex="0"
              class:sel={selected === key}
              onclick={() => onselect(key)}
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
          {/each}
        </tbody>
      </table>
    </div>
  {/if}
</section>

<style>
  .ft {
    padding: 8px 12px;
    min-width: 0;
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
