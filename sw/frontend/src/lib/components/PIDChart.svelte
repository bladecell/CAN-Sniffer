<script lang="ts">
  import { canStore } from "$lib/canStore.svelte.js";
  import { onMount, untrack, tick } from "svelte";
  import uPlot from "uplot";
  import "uplot/dist/uPlot.min.css";
  import Icon from "$lib/Icon.svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";

  interface Props {
    pid: number | string;
    update_interval_ms?: number;
    moveStart?: ((e: PointerEvent) => void) | null;
    [key: string]: any;
  }

  let {
    pid,
    update_interval_ms = 500,
    moveStart = null,
    ...rest
  }: Props = $props();

  // Instantiate the library with our reactive prop getter closure
  const metric = usePidData(() => pid);

  // --- STATE ---
  let chartRef = $state<HTMLDivElement>();
  let wrapperWidth = $state(300);
  let wrapperHeight = $state(120);
  let chartInstance: uPlot | null = null;

  // Track size modifications reactively (keeps uPlot's fast size engine)
  $effect(() => {
    const w = wrapperWidth;
    const h = wrapperHeight;

    untrack(() => {
      if (chartInstance && w > 0 && h > 0) {
        chartInstance.setSize({
          width: Math.floor(w),
          height: Math.floor(h),
        });
      }
    });
  });

  // Plain standard arrays to bypass Svelte proxy overhead loop during push/shift mutations
  let timeData: number[] = [];
  let valueData: number[] = [];
  const MAX_HISTORY_SEC = 60;

  function hexToRgba(hex: string, alpha: number): string {
    hex = hex.replace("#", "");
    const r = parseInt(hex.substring(0, 2), 16);
    const g = parseInt(hex.substring(2, 4), 16);
    const b = parseInt(hex.substring(4, 6), 16);
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  onMount(() => {
    let unsubscribe = () => {};

    const setup = async () => {
      // Isolate library reads using untrack so we only capture baseline options on initialization
      const config = untrack(() => ({
        color: metric.color,
        min: metric.min,
        max: metric.max,
      }));

      // 1. HYDRATE (Load background historical frames)
      const data = canStore.pids.get(pid);
      if (data && data.history && data.history.length > 0) {
        timeData = data.history.map((h: any) => h.timestamp);
        valueData = data.history.map((h: any) => h.value);
      }

      await tick();
      if (!chartRef) return;

      const opts: uPlot.Options = {
        width: wrapperWidth,
        height: wrapperHeight,
        cursor: { show: false },
        legend: { show: false },
        padding: [10, 0, 10, 0],
        scales: {
          x: { time: true },
          y: {
            range: () => [Number(config.min), Number(config.max)],
            clean: true,
          },
        },
        axes: [{ show: false }, { show: false }],
        series: [
          {},
          {
            stroke: config.color,
            width: 3,
            paths: uPlot.paths.spline ? uPlot.paths.spline() : undefined,
            points: { show: false },
            spanGaps: true,
            fill: (u) => {
              const ctx = u.ctx;
              const gradient = ctx.createLinearGradient(0, 0, 0, u.bbox.height);
              gradient.addColorStop(0, hexToRgba(config.color, 0.4));
              gradient.addColorStop(1, hexToRgba(config.color, 0.0));
              return gradient;
            },
          },
        ],
        hooks: {
          drawSeries: [
            (u) => {
              // Read supported safely from proxy frame stream
              if (!metric.supported || u.data[1].length === 0) return;
              const ctx = u.ctx;
              const lastIdx = u.data[0].length - 1;
              const cx = u.valToPos(u.data[0][lastIdx], "x", true);
              const cy = u.valToPos(u.data[1][lastIdx], "y", true);
              ctx.beginPath();
              ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
              ctx.fillStyle = config.color;
              ctx.fill();
            },
          ],
        },
      };

      chartInstance = new uPlot(opts, [timeData, valueData], chartRef);

      // Continuous stream event subscription
      unsubscribe = canStore.subscribe((update: any) => {
        if (update.pid !== pid) return;

        timeData.push(update.timestamp);
        valueData.push(update.value);

        const cutoff = update.timestamp - MAX_HISTORY_SEC * 1000;
        while (timeData.length > 0 && timeData[0] < cutoff) {
          timeData.shift();
          valueData.shift();
        }

        if (chartInstance) {
          chartInstance.setData([timeData, valueData], false);

          chartInstance.setScale("x", {
            min: cutoff,
            max: update.timestamp,
          });
        }
      });
    };

    setup();

    return () => {
      unsubscribe();
      if (chartInstance) chartInstance.destroy();
    };
  });

  let longPressTimer: ReturnType<typeof setTimeout> | undefined;

  function handlePointerDown(
    e: PointerEvent,
    callback: (e: PointerEvent) => void,
  ): void {
    longPressTimer = setTimeout(() => callback(e), 300);
  }

  function handlePointerUp(): void {
    clearTimeout(longPressTimer);
  }
</script>

<article
  class="pid-card"
  class:disabled={!metric.supported}
  style="background: color-mix(in srgb, {metric.color} 5%, transparent);"
  {...rest}
>
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <header
    class="card-header"
    onpointerdown={moveStart ? (e) => handlePointerDown(e, moveStart) : null}
    onpointerup={handlePointerUp}
    onpointermove={handlePointerUp}
    style="cursor: {moveStart ? 'grab' : 'default'}; touch-action: pan-y;"
  >
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
  /* MATCHES YOUR EXISTING CARD STYLE */
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

  /* HEADER SECTION */
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
  }
</style>
