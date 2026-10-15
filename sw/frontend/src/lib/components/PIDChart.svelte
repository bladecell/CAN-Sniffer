<script lang="ts">
  import type { HTMLAttributes } from "svelte/elements";
  import { onMount } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import { chartHistoryStore, MAX_HISTORY_SEC } from "$lib/chartHistoryStore";
  import type { PidGridItem } from "$lib/types";
  import SmallCardChart from "./SmallCardChart.svelte";

  interface Props extends HTMLAttributes<HTMLElement> {
    item: PidGridItem;
    update_interval_ms?: number;
  }

  let { item, update_interval_ms = 500, ...rest }: Props = $props();

  const metric = usePidData(() => item.pid);

  let chartData = $state.raw<{ date: Date; value: number | null }[]>([]);

  $effect(() => {
    if (item.pid === undefined) return;
    const unsubscribe = chartHistoryStore.subscribe(item.pid, (buffer) => {
      const last = buffer.lastTime();
      if (last === undefined) {
        chartData = [];
        return;
      }
      const { t, v } = buffer.snapshot(last - MAX_HISTORY_SEC * 1000);
      chartData = t.map((time, i) => ({ date: new Date(time), value: v[i] }));
    });

    return unsubscribe;
  });

  const xDomain = $derived(
    chartData.length > 0
      ? [
          new Date(chartData[chartData.length - 1].date.getTime() - MAX_HISTORY_SEC * 1000),
          chartData[chartData.length - 1].date,
        ]
      : undefined
  );
</script>

<article
  class="dashboard-card pid-chart-card"
  class:disabled={!metric.isSupported}
  style="--module-accent: {metric.color};"
  {...rest}
>
  <header class="dashboard-card-header">
    <div class="dashboard-card-icon">
      <Icon name={metric.icon} size={32} />
    </div>
    <div class="dashboard-card-titles">
      <div class="dashboard-card-label">{metric.label}</div>
      <div class="dashboard-card-subtitle">
        {metric.description}
      </div>
    </div>

    <div class="chart-badge">
      {metric.displayValue}<span class="unit">{metric.unit}</span>
    </div>
  </header>

  <SmallCardChart
    data={chartData}
    {xDomain}
    yDomain={[metric.min, metric.max]}
    color={metric.color ?? "#01AAFF"}
    gradientId="chart-area-gradient-{item.pid}"
  />
</article>

<style>
  .pid-chart-card {
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .chart-badge {
    font-size: 1.25rem;
    font-weight: 700;
    padding: 4px 12px;
    border-radius: 6px;
    font-family: var(--pico-font-family-monospace);
    display: flex;
    align-items: baseline;
    gap: 2px;
    color: var(--module-accent);
    background: color-mix(in srgb, var(--module-accent) 10%, transparent);
  }

  .chart-badge .unit {
    font-size: 0.8rem;
    font-weight: 500;
    color: var(--module-accent);
  }

</style>
