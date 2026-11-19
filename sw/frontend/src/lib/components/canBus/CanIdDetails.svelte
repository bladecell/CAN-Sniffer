<script lang="ts">
  import type { IdDetail } from "$lib/canBus/canBusStore.svelte";
  import { relTime } from "$lib/canBus/canFormat";
  import { formatId } from "$lib/canBus/canIds";
  import { changedMasks } from "$lib/canBus/canModel";
  import CanByteTile from "./CanByteTile.svelte";

  interface Props {
    detail: IdDetail | null;
    startUs: number | null;
    ruleExists: boolean;
    onrule: (detail: IdDetail) => void;
  }

  let { detail, startUs, ruleExists, onrule }: Props = $props();
  const bitAt = (mask: number, bit: number) => ((mask >> bit) & 1) === 1;
  const BITS = [7, 6, 5, 4, 3, 2, 1, 0];
  const rate = (hz: number) => (hz >= 10 ? String(Math.round(hz)) : hz.toFixed(1));
  // A CAN frame carries at most 8 bytes; the slice keeps a bad length from building more rows.
  const bytes = $derived(detail ? Array.from(detail.last.data).slice(0, 8) : []);
  // Newest first, each row with the bytes that changed against the next older frame.
  const recentRows = $derived.by(() => {
    if (!detail) return [];
    const masks = changedMasks(detail.recent);
    return detail.recent.map((f, i) => ({ t: f.t, data: f.data.subarray(0, 8), changed: masks[i] }));
  });
  const BYTE_IDX = [0, 1, 2, 3, 4, 5, 6, 7];
</script>

<div class="id-pane">
  {#if detail === null}
    <h2>ID details</h2>
    <p class="muted">Select an ID in the IDs list, then Details, or double-click it.</p>
  {:else}
    <div class="id-head">
      <div>
        <h2>ID <span class="idn">{formatId(detail.id, detail.ext)}</span></h2>
        <p class="muted">{detail.name || (detail.ext ? "Extended id" : "Standard id")} · DLC {detail.dlc}</p>
      </div>
      <button
        class="btn sm"
        title={ruleExists ? "Opens the rule for this ID in the Rules tab" : "Opens the Rules tab with a new rule for this ID"}
        onclick={() => onrule(detail)}
      >
        {ruleExists ? "Open rule" : "+ Create rule"}
      </button>
    </div>

    <div class="stats">
      <div><small>Rate</small><b>{rate(detail.hz)} <i>/s</i></b></div>
      <div><small>Frames</small><b>{detail.count}</b></div>
      <div><small>Changes</small><b>{detail.changes}</b></div>
    </div>

    <div class="bt" aria-label="Last data bytes">
      <span></span>
      {#each bytes as b, i (i)}
        <CanByteTile value={b} changed={(detail.changed & (1 << i)) !== 0} />
      {/each}
    </div>

    <div class="sub"><span>Bits</span><span>bit 7 to 0</span></div>
    <div class="bits" role="group" aria-label="Bits of the last frame">
      <span></span>
      {#each BITS as n (n)}<span class="h">{n}</span>{/each}
      {#each bytes as b, i (i)}
        <span class="r">B{i}</span>
        {#each BITS as n (n)}
          {@const recent = bitAt(detail.recentBits[i] ?? 0, n)}
          {@const ever = bitAt(detail.everBits[i] ?? 0, n)}
          <div class="k" class:now={recent} class:ev={ever && !recent} class:on={bitAt(b, n)}></div>
        {/each}
      {/each}
    </div>
    <div class="legend">
      <span><i class="on"></i>1</span>
      <span><i></i>0</span>
      <span><i class="now"></i>changing now</span>
      <span><i class="ev"></i>changed before</span>
    </div>

    <h3>Recent values</h3>
    <ul class="hist">
      {#each recentRows as r (r.t)}
        <li class="hrow">
          <span class="time">{relTime(r.t, startUs)}</span>
          {#each BYTE_IDX as i (i)}
            {#if i < r.data.length}<CanByteTile value={r.data[i]} changed={(r.changed & (1 << i)) !== 0} />{:else}<span></span>{/if}
          {/each}
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
  .id-head {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    gap: 10px;
  }
  .stats {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 8px;
  }
  .stats div {
    display: grid;
  }
  .stats small,
  .sub {
    font-size: 0.72rem;
    font-weight: 800;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: var(--muted);
  }
  .stats b {
    font-size: 1.25rem;
    font-variant-numeric: tabular-nums;
  }
  .stats b i {
    font-style: normal;
    font-size: 0.8rem;
    font-weight: 600;
    color: var(--muted);
  }
  .idn {
    font-family: var(--mono);
    font-weight: 800;
    letter-spacing: 0.02em;
    color: var(--accent-text);
  }
  .bt {
    --tile-h: 38px;
    display: grid;
    grid-template-columns: 26px repeat(8, minmax(0, 44px));
    justify-content: center;
    gap: 4px;
  }
  .sub {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    gap: 8px;
    margin-top: 4px;
  }
  .bits {
    display: grid;
    grid-template-columns: 26px repeat(8, minmax(0, 44px));
    justify-content: center;
    gap: 4px;
    align-items: center;
  }
  .bits .h,
  .bits .r {
    font-family: var(--mono);
    font-size: 0.72rem;
    font-weight: 800;
    color: var(--muted);
  }
  .bits .h {
    text-align: center;
  }
  .k {
    aspect-ratio: 1;
    min-height: 24px;
    border-radius: 6px;
    border: 2px solid var(--hair);
  }
  .k.on {
    background: color-mix(in srgb, var(--text) 50%, transparent);
  }
  .k.ev {
    border-color: var(--muted);
  }
  .k.now {
    border-color: var(--accent);
  }
  .k.now.on {
    background: var(--accent);
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 12px;
    font-size: 0.78rem;
    font-weight: 600;
    color: var(--muted);
  }
  .legend span {
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .legend i {
    width: 14px;
    height: 14px;
    border-radius: 4px;
    border: 2px solid var(--hair);
  }
  .legend i.on {
    background: color-mix(in srgb, var(--text) 50%, transparent);
  }
  .legend i.ev {
    border-color: var(--muted);
  }
  .legend i.now {
    border-color: var(--accent);
  }
  h3 {
    margin: 10px 0 2px;
    font-size: 1rem;
  }
  .hist {
    width: fit-content;
    max-width: 100%;
    display: grid;
    gap: 2px;
    margin: 0 auto;
    padding: 0;
    list-style: none;
  }
  .hrow {
    --tile-h: 34px;
    display: grid;
    grid-template-columns: 4.1rem repeat(8, minmax(0, 40px));
    justify-content: center;
    gap: 3px;
    align-items: center;
    padding: 5px 0;
    border-top: 1px solid var(--hair);
  }
  .hrow:first-child {
    border-top: 0;
  }
  .hrow .time {
    font-family: var(--mono);
    font-size: 0.74rem;
    white-space: nowrap;
    color: var(--muted);
  }
</style>
