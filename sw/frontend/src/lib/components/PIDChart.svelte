<script lang="ts">
  import { canStore } from "$lib/canStore.svelte.js";
  import { onMount, untrack, tick, onDestroy } from "svelte";
  import uPlot from "uplot";
  import "uplot/dist/uPlot.min.css";
  import Icon from "$lib/Icon.svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
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
  onMount(() => {
    let unsubscribe = canStore.subscribe((update: any) => {
      if (update.pid !== item.pid) return;

      timeData.push(update.timestamp);
      valueData.push(update.value);

      const cutoff = update.timestamp - MAX_HISTORY_SEC * 1000;
      while (timeData.length > 0 && timeData[0] < cutoff) {
        timeData.shift();
        valueData.shift();
      }

      if (chartInstance) {
        chartInstance.setData([timeData, valueData], false);
        chartInstance.setScale("x", { min: cutoff, max: update.timestamp });
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
  class="pid-card"
  class:disabled={!metric.supported}
  style="background: color-mix(in srgb, {metric.color} 5%, transparent);"
  {...rest}
>
  <header class="card-header">
    <div
      class="icon"
      style="background: color-mix(in srgb, {metric.color} 20%, transparent);"
    >
      <Icon name={metric.icon} size={32} />
    </div>
    <div class="titles">
      <div class="label">{metric.label}</div>
      <div
        class="subtitle"
        style="color: color-mix(in srgb, {metric.color} 70%, transparent);"
      >
        {metric.description}
      </div>
    </div>

    <div
      class="badge"
      style="background: color-mix(in srgb, {metric.color} 10%, transparent); color: {metric.color};"
    >
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
  .pid-card {
    width: 100%;
    height: 100%;
    box-sizing: border-box;
    margin: 0;
    display: flex;
    flex-direction: column;
    padding: 16px;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 12px;
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
    overflow: hidden;
  }

  .pid-card:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  }

  .pid-card.disabled {
    opacity: 0.4;
    filter: grayscale(100%);
  }

  .pid-card.disabled:hover {
    transform: none;
    box-shadow: none;
  }

  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 8px;
    background: none;
    border: none;
  }

  .icon {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 48px;
    width: 48px;
    border-radius: 10px;
    transition: transform 0.2s ease;
  }

  .pid-card:hover .icon {
    transform: scale(1.05);
  }

  .titles {
    display: flex;
    flex-direction: column;
    gap: 4px;
    align-items: flex-start;
    flex: 1;
    padding: 0 12px;
  }

  .label {
    font-size: 0.75rem;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: var(--pico-muted-color);
    font-weight: 500;
  }

  .subtitle {
    font-size: 0.65rem;
    letter-spacing: 0.05em;
    font-family: var(--pico-font-family-monospace);
  }

  .badge {
    font-size: 1.25rem;
    font-weight: 700;
    padding: 4px 12px;
    border-radius: 6px;
    font-family: var(--pico-font-family-monospace);
    display: flex;
    align-items: baseline;
    gap: 2px;
  }

  .badge .unit {
    font-size: 0.8rem;
    font-weight: 500;
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
