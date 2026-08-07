<script lang="ts">
  import { onMount } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import { chartHistoryStore } from "$lib/chartHistoryStore";
  import type { PidGridItem } from "$lib/types";
  import { Chart, Svg, Spline, Area } from "layerchart";
  import { scaleTime } from "d3-scale";

  interface Props {
    item: PidGridItem;
    update_interval_ms?: number;
    [key: string]: any;
  }

  let { item, update_interval_ms = 500, ...rest }: Props = $props();

  const metric = usePidData(() => item.pid);

  let chartData = $state<{ date: Date; value: number }[]>([]);

  $effect(() => {
    if (item.pid === undefined) return;
    const unsubscribe = chartHistoryStore.subscribe(item.pid, (tData, vData) => {
      chartData = tData.map((t, i) => ({ date: new Date(t), value: vData[i] }));
    });

    return () => {
      unsubscribe();
    };
  });
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

  <div class="chart-wrapper">
    {#if chartData.length > 0}
      <Chart
        data={chartData}
        x="date"
        xScale={scaleTime()}
        y="value"
        yDomain={[metric.min, metric.max]}
        padding={{ top: 10, bottom: 2, left: 0, right: 0 }}
      >
        <Svg>
          <defs>
            <linearGradient id="chart-area-gradient-{item.pid}" x1="0" x2="0" y1="0" y2="1">
              <stop offset="0%" stop-color={metric.color ?? "#01AAFF"} stop-opacity="0.4" />
              <stop offset="100%" stop-color={metric.color ?? "#01AAFF"} stop-opacity="0.0" />
            </linearGradient>
          </defs>
          <Area fill="url(#chart-area-gradient-{item.pid})" />
          <Spline
            stroke={metric.color ?? "#01AAFF"}
            strokeWidth={3}
            class="fill-none"
          />
        </Svg>
      </Chart>
    {/if}
  </div>
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

  .chart-wrapper {
    flex-grow: 1;
    width: calc(100% + 32px);
    margin-left: -16px;
    margin-right: -16px;
    position: relative;
    min-height: 0;
    min-width: 0;
    overflow: hidden;
  }
</style>
