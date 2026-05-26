<script lang="ts">
  import { canStore } from "$lib/canStore.svelte.js";
  import { onMount } from "svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { OverviewGridItem } from "$lib/types";

  interface Props {
    item: OverviewGridItem;
    [key: string]: any;
  }

  let { item, ...rest }: Props = $props();

  onMount(async () => {
    await canStore.requestVin();
    await canStore.requestDTC();
  });

  const vin = $derived(canStore.vin || "--- UNKNOWN ---");
  const battery = $derived(
    canStore.wsCanStatus?.battery_voltage.toFixed(1) || "0.0",
  );
  const isOnline = $derived(canStore.wsCanStatus?.state === "Connected");
  const state = $derived(canStore.wsCanStatus?.state || "Disconnected");
  const utilization = $derived(canStore.wsCanStatus?.utilization ?? 0);
  const dtcCount = $derived(canStore.totalDTCs ?? 0);

  const statusColor = $derived(
    isOnline ? "var(--pico-ins-color)" : "var(--pico-del-color)",
  );
</script>

{#snippet pidBadge(pidId: number)}
  {@const metric = usePidData(() => pidId)}
  <div
    class="pid-badge"
    style="background: color-mix(in srgb, {metric.isValid
      ? metric.color
      : '#6b7280'} 15%, transparent);"
  >
    <span class="pid-label">{metric.label}</span>
    <span class="pid-value"
      >{metric.displayValue}<small>{metric.unit}</small></span
    >
  </div>
{/snippet}

<article class="overview-card" style="--module-accent: {item.color}" {...rest}>
  <header class="card-header">
    <div class="titles">
      <div class="label">System Overview</div>
      <div class="vin-text">{vin}</div>
    </div>

    <div class="header-badges">
      <div class="badge util-badge">
        <span class="badge-label">Util</span>
        <span class="badge-value">{utilization}%</span>
      </div>
      <div class="badge status-badge" style:--status-color={statusColor}>
        <div class="status-dot"></div>
        <span class="badge-value">{state}</span>
      </div>
    </div>
  </header>

  <div class="card-body">
    <div class="stats-row">
      <div class="stat-item">
        <span class="label">Battery</span>
        <div class="value">
          <span class="number">{battery}</span>
          <span class="unit">V</span>
        </div>
      </div>
      <div class="stat-item">
        <span class="label">Diagnostics</span>
        <div class="value">
          <span class="number" class:error={dtcCount > 0}>{dtcCount}</span>
          <span class="unit">DTCs</span>
        </div>
      </div>
    </div>

    <div class="pid-footer">
      {#each item.pids as pidId}
        {@render pidBadge(pidId)}
      {/each}
    </div>
  </div>
</article>

<style>
  .overview-card {
    width: 100%;
    height: 100%;
    box-sizing: border-box;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    padding: 16px;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 12px;
    background: color-mix(
      in srgb,
      var(--module-accent) 8%,
      rgba(20, 20, 25, 0.8)
    );
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 16px;
    background: none;
    border: none;
  }

  .header-badges {
    display: flex;
    gap: 8px;
    align-items: center;
  }

  .badge {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 4px 8px;
    background: color-mix(
      in srgb,
      var(--pico-muted-border-color) 20%,
      transparent
    );
    border-radius: 6px;
    border: 1px solid var(--pico-muted-border-color);
  }

  .badge-label {
    font-size: 0.55rem;
    font-weight: 700;
    text-transform: uppercase;
    color: var(--pico-muted-color);
  }

  .badge-value {
    font-family: var(--pico-font-family-monospace);
    font-size: 0.7rem;
    font-weight: 800;
  }

  .status-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: var(--status-color);
    box-shadow: 0 0 8px var(--status-color);
    animation: pulse-glow 2s infinite alternate;
  }

  .titles {
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  .label {
    font-size: 0.7rem;
    text-transform: uppercase;
    color: var(--pico-muted-color);
    font-weight: 500;
  }

  .vin-text {
    font-family: var(--pico-font-family-monospace);
    font-size: 1.1rem;
    font-weight: 700;
    color: var(--pico-contrast);
  }

  .card-body {
    display: flex;
    flex-direction: column;
    gap: 20px;
    flex-grow: 1;
    justify-content: center;
  }

  .stats-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 16px;
  }

  .stat-item {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .value {
    display: flex;
    align-items: baseline;
    gap: 4px;
  }

  .number {
    font-size: 2rem;
    font-weight: 700;
    line-height: 1;
    font-family: var(--pico-font-family-monospace);
  }

  .number.error {
    color: var(--pico-del-color);
  }

  .unit {
    font-size: 0.9rem;
    color: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }

  .pid-footer {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 8px;
    margin-top: auto;
  }

  .pid-badge {
    padding: 8px;
    border-radius: 8px;
    text-align: center;
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  .pid-label {
    font-size: 0.55rem;
    text-transform: uppercase;
    color: var(--pico-muted-color);
    font-weight: 600;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .pid-value {
    font-family: var(--pico-font-family-monospace);
    font-size: 0.9rem;
    font-weight: 700;
  }

  .pid-value small {
    font-size: 0.6rem;
    margin-left: 2px;
    color: var(--pico-muted-color);
  }

  @keyframes pulse-glow {
    0% {
      opacity: 1;
      box-shadow: 0 0 1px var(--status-color);
    }
    50% {
      opacity: 0.85;
      box-shadow: 0 0 6px var(--status-color);
    }
    100% {
      opacity: 1;
      box-shadow: 0 0 8px var(--status-color);
    }
  }

  .overview-card:hover {
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
    border-color: color-mix(
      in srgb,
      var(--module-accent) 40%,
      var(--pico-muted-border-color)
    ) !important;
  }
</style>
