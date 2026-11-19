<script lang="ts">
  import type { IdDetail } from "$lib/canBus/canBusStore.svelte";
  import { relTime } from "$lib/canBus/canFormat";
  import { formatId } from "$lib/canBus/canIds";

  interface Props {
    detail: IdDetail | null;
    startUs: number | null;
    ruleExists: boolean;
    onrule: (detail: IdDetail) => void;
  }

  let { detail, startUs, ruleExists, onrule }: Props = $props();
  const hex = (v: number) => v.toString(16).toUpperCase().padStart(2, "0");
  const bitAt = (mask: number, bit: number) => ((mask >> bit) & 1) === 1;
  const BITS = [7, 6, 5, 4, 3, 2, 1, 0];
  const rate = (hz: number) => (hz >= 10 ? String(Math.round(hz)) : hz.toFixed(1));
  // A CAN frame carries at most 8 bytes; the slice keeps a bad length from building more rows.
  const bytes = $derived(detail ? Array.from(detail.last.data).slice(0, 8) : []);
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
      {#each bytes as b, i (i)}
        <div class="b" class:z={b === 0} class:chg={(detail.changed & (1 << i)) !== 0}>{hex(b)}</div>
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
      <span><i class="now"></i>changing now</span>
      <span><i class="ev"></i>changed before</span>
      <span><i></i>never</span>
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
  /* Byte tile, shared in spirit with the table: tint and corner mark = changed on the last frame, dim = 00. */
  .bt {
    display: grid;
    grid-template-columns: repeat(8, minmax(0, 1fr));
    gap: 4px;
  }
  .b {
    position: relative;
    display: grid;
    place-items: center;
    height: 34px;
    min-width: 0;
    border-radius: 8px;
    background: var(--card2);
    font-family: var(--mono);
    font-weight: 700;
    font-size: 0.92rem;
    font-variant-numeric: tabular-nums;
  }
  .b.z {
    color: var(--muted);
    font-weight: 500;
    opacity: 0.7;
  }
  .b.chg {
    background: color-mix(in srgb, var(--accent) 26%, transparent);
    color: var(--text);
  }
  .b.chg::after {
    content: "";
    position: absolute;
    top: 0;
    right: 0;
    width: 9px;
    height: 9px;
    border-top-right-radius: 8px;
    background: linear-gradient(225deg, var(--accent) 50%, transparent 50%);
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
    grid-template-columns: 26px repeat(8, minmax(0, 1fr));
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
    aspect-ratio: 1.25;
    min-height: 24px;
    border-radius: 6px;
    border: 1px solid var(--hair);
  }
  .k.ev {
    background: color-mix(in srgb, var(--muted) 38%, transparent);
    border-color: transparent;
  }
  .k.now {
    background: var(--accent);
    border-color: transparent;
  }
  .k.on::after {
    content: "1";
    display: grid;
    place-items: center;
    height: 100%;
    font: 700 0.7rem var(--mono);
    color: var(--on-accent);
  }
  .k.ev.on::after {
    color: var(--text);
    opacity: 0.8;
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
    border: 1px solid var(--hair);
  }
  .legend i.ev {
    background: color-mix(in srgb, var(--muted) 38%, transparent);
    border-color: transparent;
  }
  .legend i.now {
    background: var(--accent);
    border-color: transparent;
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
