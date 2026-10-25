<script lang="ts">
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import LayerChartLarge from "$lib/components/LayerChartLarge.svelte";
  import type { PidDefinition } from "$lib/types";

  interface Props {
    n: number;
    pids: number[];
    active: boolean;
    onActivate: () => void;
  }

  let { n, pids, active, onActivate }: Props = $props();

  let zoomed = $state(false);
  let chart = $state<{ reset: () => void; fitY: () => void }>();

  const defByPid = $derived(new Map(vehicleStore.pidDefinitions.map((d) => [d.pid, d])));

  function defOf(pid: number): PidDefinition | undefined {
    return defByPid.get(pid);
  }

  function colorOf(pid: number) {
    return safeHexColor(defOf(pid)?.color);
  }

  function liveValue(pid: number) {
    return connectionStore.pids.get(pid)?.value ?? "N/A";
  }
</script>

<div class="cell tel-bar-mark" role="group" aria-label="Chart {n}" aria-current={active ? "true" : undefined} onpointerdown={onActivate}>
  <div class="head">
    <button type="button" class="csel" aria-pressed={active} onclick={onActivate}>Chart {n}</button>
    <div class="legend">
      {#each pids as pid (pid)}
        {@const def = defOf(pid)}
        <span class="lg">
          <i style="--sw: {colorOf(pid)}"></i>
          <span class="lname">{def?.name || `0x${pid.toString(16).toUpperCase()}`}</span>
          <b class="num">{liveValue(pid)}</b>
          {#if def?.unit && typeof liveValue(pid) === "number"}<small>{def.unit}</small>{/if}
        </span>
      {/each}
    </div>
    {#if zoomed}
      <div class="tools">
        <button type="button" class="tel-btn sm" onclick={() => chart?.reset()}>Reset view</button>
        <button type="button" class="tel-btn sm" onclick={() => chart?.fitY()}>Fit Y</button>
      </div>
    {/if}
  </div>
  <div class="plot">
    <LayerChartLarge bind:this={chart} {pids} isActive={active} bind:zoomed />
  </div>
</div>

<style>
  .cell {
    position: relative;
    display: flex;
    flex-direction: column;
    min-width: 0;
    padding: 10px 12px 12px 16px;
    border-radius: var(--r-card);
    background: var(--card);
  }
  .head {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 4px 16px;
    padding: 0 0 4px 6px;
  }
  .csel {
    all: unset;
    box-sizing: border-box;
    display: inline-flex;
    align-items: center;
    min-height: 34px;
    padding: 0 6px;
    border-radius: 8px;
    cursor: pointer;
    color: var(--muted);
    font-weight: 800;
  }
  .csel[aria-pressed='true'] {
    color: var(--accent-text);
  }
  .csel:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  @media (pointer: coarse) {
    .csel {
      min-height: 44px;
    }
  }
  .legend {
    display: flex;
    flex: 1;
    flex-wrap: wrap;
    gap: 4px 18px;
    min-width: 0;
  }
  .lg {
    display: inline-flex;
    align-items: baseline;
    gap: 8px;
    min-width: 0;
    white-space: nowrap;
  }
  .lg i {
    align-self: center;
    width: 14px;
    height: 4px;
    border-radius: 2px;
    background: var(--sw);
    flex: none;
  }
  .lname {
    overflow: hidden;
    text-overflow: ellipsis;
    font-weight: 700;
  }
  .lg b {
    font-size: 1.5rem;
    font-weight: 800;
    line-height: 1.1;
    font-variant-numeric: tabular-nums;
  }
  .lg small {
    color: var(--muted);
  }
  .tools {
    display: flex;
    gap: 6px;
    margin-left: auto;
  }
  .plot {
    display: flex;
    flex-direction: column;
    min-width: 0;
  }
</style>
