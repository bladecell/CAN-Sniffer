<script lang="ts">
  import type { IdDetail } from "$lib/canBus/canBusStore.svelte";
  import { hzText, relTime } from "$lib/canBus/canFormat";
  import { formatId } from "$lib/canBus/canIds";

  interface Props {
    detail: IdDetail | null;
    startUs: number | null;
  }

  let { detail, startUs }: Props = $props();
  const hex = (v: number) => v.toString(16).toUpperCase().padStart(2, "0");
</script>

<div class="id-pane">
  {#if detail === null}
    <h2>ID details</h2>
    <p class="muted">Select an ID in the IDs list, then Details, or double-click it.</p>
  {:else}
    <h2>ID <span class="idn">{formatId(detail.id, detail.ext)}</span></h2>
    <p class="muted">{detail.name || (detail.ext ? "Extended id" : "Standard id")}</p>

    <div class="bytes" aria-label="Last data bytes">
      {#each Array.from(detail.last.data) as b, i (i)}
        <div class="cell" class:chg={(detail.changed & (1 << i)) !== 0}>
          <small>B{i}</small>
          <b class="mono">{hex(b)}</b>
        </div>
      {/each}
    </div>

    <div class="chips">
      <span class="chip hz">{hzText(detail.hz)}</span>
      <span class="chip c-plain">DLC {detail.dlc}</span>
      <span class="chip c-plain">{detail.count} frames</span>
    </div>

    <h3>Recent values</h3>
    <ul class="hist mono">
      {#each detail.recent as f (f.t)}
        <li>
          <span class="muted">{relTime(f.t, startUs)}</span>
          {Array.from(f.data, hex).join(" ")}
        </li>
      {/each}
    </ul>
  {/if}
</div>

<style>
  .id-pane {
    display: grid;
    gap: 10px;
    min-width: 0;
  }
  .id-pane h2 {
    margin: 0;
  }
  .id-pane p {
    margin: 0;
  }
  .chips {
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
  }
  .bytes {
    display: grid;
    grid-template-columns: repeat(8, minmax(0, 1fr));
    gap: 6px;
    margin: 2px 0;
  }
  .cell {
    padding: 8px 0;
    text-align: center;
    border-radius: var(--r-ctl);
    background: var(--card2);
  }
  .cell small {
    display: block;
    color: var(--muted);
    font-size: 0.68rem;
    font-weight: 800;
  }
  .cell b {
    font-size: 0.9rem;
  }
  .cell.chg {
    background: color-mix(in srgb, var(--accent) 28%, transparent);
  }
  .cell.chg b {
    color: var(--accent-text);
  }
  .idn {
    font-family: var(--mono);
    font-weight: 800;
    letter-spacing: 0.02em;
    color: var(--accent-text);
  }
  .chip.hz {
    background: color-mix(in srgb, var(--accent) 24%, transparent);
    color: var(--accent-text);
  }
  h3 {
    margin: 10px 0 2px;
    font-size: 1rem;
  }
  .hist {
    display: grid;
    gap: 2px;
    margin: 0;
    padding: 0;
    list-style: none;
    font-size: 0.88rem;
  }
  .hist li {
    display: flex;
    gap: 12px;
    padding: 6px 0;
    border-top: 1px solid var(--hair);
  }
  .hist li:first-child {
    border-top: 0;
  }
</style>
