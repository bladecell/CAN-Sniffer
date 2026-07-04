<script lang="ts">
  import { canStore } from "$lib/canStore.svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { OverviewGridItem, DtcModeData } from "$lib/types";

  interface Props {
    item: OverviewGridItem;
    [key: string]: any;
  }

  let { item, ...rest }: Props = $props();

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

<article
  class="dashboard-card overview-card"
  style="--module-accent: {item.color}"
  {...rest}
>
  <header class="dashboard-card-header">
    <div class="dashboard-card-titles">
      <div class="dashboard-card-label">System Overview</div>
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

  <div class="dashboard-card-body card-body">
    <div class="stats-row">
      <div class="stat-item">
        <span class="dashboard-card-label">Battery</span>
        <div class="dashboard-card-value-group">
          <span class="dashboard-card-number">{battery}</span>
          <span class="dashboard-card-unit">V</span>
        </div>
      </div>
      <div class="stat-item">
        <span class="dashboard-card-label">Diagnostics</span>
        <div class="dashboard-card-value-group">
          <span class="dashboard-card-number" class:error={dtcCount > 0}
            >{dtcCount}</span
          >
          <span class="dashboard-card-unit">DTCs</span>
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
    justify-content: space-between;
    background: color-mix(
      in srgb,
      var(--module-accent) 8%,
      rgba(20, 20, 25, 0.8)
    );
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

  .vin-text {
    font-family: var(--pico-font-family-monospace);
    font-size: 1.1rem;
    font-weight: 700;
    color: var(--pico-contrast);
  }

  .card-body {
    gap: 20px;
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

  .error {
    color: var(--pico-del-color);
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
</style>
