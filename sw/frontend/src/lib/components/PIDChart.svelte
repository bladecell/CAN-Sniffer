<script lang="ts">
  import { canStore } from "$lib/canStore.svelte";
  import { onMount, untrack, tick, onDestroy } from "svelte";
  import uPlot from "uplot";
  import "uplot/dist/uPlot.min.css";
  import Icon from "$lib/Icon.svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import { chartHistoryStore } from "$lib/chartHistoryStore";
  import type { PidGridItem } from "$lib/types";

  interface Props {
    item: PidGridItem;
    update_interval_ms?: number;
    [key: string]: any;
  }

  let { item, update_interval_ms = 500, ...rest }: Props = $props();

  const metric = usePidData(() => item.pid);

  // --- STATE ---
  let chartRef = $state<HTMLDivElement>();
  let wrapperWidth = $state(300);
  let wrapperHeight = $state(120);
  let chartInstance: uPlot | null = null;

  // Plain standard arrays for performance
  let timeData: number[] = [];
  let valueData: number[] = [];
  const MAX_HISTORY_SEC = 60;

  function hexToRgba(hex: string, alpha: number): string {
    const cleanHex = hex.replace("#", "");
    const r = parseInt(cleanHex.substring(0, 2), 16) || 0;
    const g = parseInt(cleanHex.substring(2, 4), 16) || 0;
    const b = parseInt(cleanHex.substring(4, 6), 16) || 0;
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  // 1. Initialization and Re-configuration Effect
  $effect(() => {
    // Track dependencies we want to trigger a RE-INIT or UPDATE
    const color = metric.color;
    const min = metric.min;
    const max = metric.max;
    const pid = item.pid;
    const w = wrapperWidth;
    const h = wrapperHeight;

    if (!chartRef || w <= 0 || h <= 0) return;

    if (!chartInstance) {
      // First time initialization
      untrack(() => initChart(pid, color, min, max));
    } else {
      // Update existing instance
      untrack(() => updateChartConfig(color, min, max, w, h));
    }
  });

  function initChart(pid: number, color: string, min: number, max: number) {
    const data = canStore.pids.get(pid);
    if (data?.history?.length) {
      timeData = data.history.map((h: any) => h.timestamp);
      valueData = data.history.map((h: any) => h.value);
    }

    const opts: uPlot.Options = {
      width: wrapperWidth,
      height: wrapperHeight,
      cursor: { show: false },
      legend: { show: false },
      padding: [10, 0, 10, 0],
      scales: {
        x: { time: true },
        y: {
          range: (u, min, max) => [Number(metric.min), Number(metric.max)],
          auto: false,
        },
      },
      axes: [{ show: false }, { show: false }],
      series: [
        {},
        {
          stroke: () => metric.color,
          width: 3,
          paths: uPlot.paths.spline ? uPlot.paths.spline() : undefined,
          points: { show: false },
          spanGaps: true,
          fill: (u) => {
            const ctx = u.ctx;
            const gradient = ctx.createLinearGradient(0, 0, 0, u.bbox.height);
            gradient.addColorStop(0, hexToRgba(metric.color, 0.4));
            gradient.addColorStop(1, hexToRgba(metric.color, 0.0));
            return gradient;
          },
        },
      ],
    };

    chartInstance = new uPlot(opts, [timeData, valueData], chartRef!);
  }

  function updateChartConfig(
    color: string,
    min: number,
    max: number,
    w: number,
    h: number,
  ) {
    if (!chartInstance) return;

    // Update size
    chartInstance.setSize({ width: Math.floor(w), height: Math.floor(h) });

    // Update Scales
    chartInstance.setScale("y", { min, max });

    // Redraw to apply dynamic stroke and fill colors from metric.color
    chartInstance.redraw();
  }

  // 2. Data Subscription Management
  $effect(() => {
    let unsubscribe = chartHistoryStore.subscribe(item.pid, (tData, vData) => {
      timeData = tData;
      valueData = vData;

      if (chartInstance && timeData.length > 0) {
        chartInstance.setData([timeData, valueData], false);
        const cutoff = timeData[0];
        const latest = timeData[timeData.length - 1];
        chartInstance.setScale("x", { min: cutoff, max: latest });
      }
    });

    return () => {
      unsubscribe();
      chartInstance?.destroy();
      chartInstance = null;
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

  <div
    class="chart-wrapper"
    bind:clientWidth={wrapperWidth}
    bind:clientHeight={wrapperHeight}
  >
    <div bind:this={chartRef}></div>
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
