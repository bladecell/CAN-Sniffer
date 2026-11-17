<script lang="ts">
  import { canBus } from "$lib/canBus/canBusStore.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { hzText, loadText, sparkPoints, twaiLabel } from "$lib/canBus/canFormat";

  const fps = $derived(canBus.counters.fps);
  const load = $derived(canBus.counters.load);
  const dropped = $derived(canBus.counters.dropped);
  const droppedTotal = $derived(dropped ? dropped.driver + dropped.ring + dropped.socket : null);
  const node = $derived(canBus.bus?.node_status);
</script>

<div class="ct">
  <section class="card ct-tile" aria-label="Frames per second">
    <small>Frames / s</small>
    <b>{Math.round(fps)}</b>
    <svg class="ct-spark" viewBox="0 0 100 30" preserveAspectRatio="none" aria-hidden="true">
      <polyline points={sparkPoints(canBus.counters.fpsHistory)} />
    </svg>
  </section>
  <section class="card ct-tile" aria-label="Bus load">
    <small>Bus load</small>
    <b>{loadText(load)}</b>
    <div class="ct-meter" role="meter" aria-label="Bus load" aria-valuemin="0" aria-valuemax="100" aria-valuenow={load === null ? undefined : load / 10}>
      <i style:width="{load === null ? 0 : Math.min(100, load / 10)}%"></i>
    </div>
  </section>
  <section class="card ct-tile" aria-label="Dropped frames">
    <small>Dropped</small>
    <b>{droppedTotal ?? "–"}</b>
    <span class="muted">{dropped ? `driver ${dropped.driver} · ring ${dropped.ring} · socket ${dropped.socket}` : "No stream"}</span>
  </section>
  <section class="card ct-tile" aria-label="Bus state">
    <small>State</small>
    <b>{twaiLabel(node?.twai_error_state)}</b>
    <span class="muted">{node ? `TEC ${node.tx_error_count} · REC ${node.rx_error_count}` : connectionStore.wsCanStatus.state}</span>
  </section>
</div>

<style>
  .ct {
    display: grid;
    gap: var(--gap, 20px);
    grid-template-columns: repeat(4, minmax(0, 1fr));
  }
  .ct-tile {
    padding: 16px 20px;
    display: grid;
    gap: 2px;
    align-content: start;
    min-width: 0;
  }
  .ct-tile small {
    color: var(--muted);
    font-weight: 700;
  }
  .ct-tile b {
    font-size: 2.2rem;
    font-weight: 800;
    line-height: 1.15;
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
  }
  .ct-tile span {
    overflow-wrap: anywhere;
  }
  .ct-spark {
    width: 100%;
    height: 34px;
    margin-top: 6px;
  }
  .ct-spark polyline {
    fill: none;
    stroke: var(--accent);
    stroke-width: 2;
    vector-effect: non-scaling-stroke;
  }
  .ct-meter {
    height: 8px;
    margin-top: 10px;
    border-radius: 99px;
    background: var(--ctl);
    overflow: hidden;
  }
  .ct-meter i {
    display: block;
    height: 100%;
    background: var(--accent);
    border-radius: 99px;
  }
  @media (max-width: 859px) {
    .ct {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
  }
</style>
