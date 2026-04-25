<script>
  import { canStore } from "$lib/canStore.svelte.js";
  import { onMount, untrack, tick } from "svelte";
  import uPlot from "uplot";
  import "uplot/dist/uPlot.min.css";
  import Icon from "$lib/Icon.svelte";

  let {
    pid,
    label = "Metric",
    description = "",
    unit = "%",
    icon = "gear",
    color = "#ff6b6b",
    update_interval_ms = 500,
    min = 0,
    max = 100,
    ...rest
  } = $props();

  // --- REACTIVE DATA ---
  const pidData = $derived(canStore.pids.get(pid));
  const currentValue = $derived(pidData?.value ?? 0);
  const supported = $derived(pidData?.supported ?? false);
  const displayValue = $derived(supported ? currentValue.toFixed(1) : "···");

  // --- STATE ---
  let chartRef = $state();
  let wrapperWidth = $state(300);
  let wrapperHeight = $state(120);
  let chartInstance = null;

  $effect(() => {
    // We track width and height
    const w = wrapperWidth;
    const h = wrapperHeight;

    untrack(() => {
      if (chartInstance && w > 0 && h > 0) {
        // uPlot's built-in resize method is extremely fast
        chartInstance.setSize({
          width: Math.floor(w),
          height: Math.floor(h),
        });
      }
    });
  });

  // Use plain arrays (not $state) to hold data.
  // This prevents Svelte from looping when we call .push()
  let timeData = [];
  let valueData = [];
  const MAX_HISTORY_SEC = 60;

  function hexToRgba(hex, alpha) {
    hex = hex.replace("#", "");
    const r = parseInt(hex.substring(0, 2), 16);
    const g = parseInt(hex.substring(2, 4), 16);
    const b = parseInt(hex.substring(4, 6), 16);
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  onMount(() => {
    let unsubscribe = () => {};

    const setup = async () => {
      // 1. HYDRATE (Load the background history)
      const data = canStore.pids.get(pid);
      if (data && data.history.length > 0) {
        // Map history to uPlot arrays
        timeData = data.history.map((h) => h.timestamp);
        valueData = data.history.map((h) => h.value);
      }

      await tick();

      const opts = {
        width: wrapperWidth,
        height: wrapperHeight,
        cursor: { show: false },
        legend: { show: false },
        padding: [10, 0, 10, 0],
        scales: {
          x: { time: true },
          y: { range: () => [Number(min), Number(max)], clean: true },
        },
        axes: [{ show: false }, { show: false }],
        series: [
          {},
          {
            stroke: color,
            width: 3,
            paths: uPlot.paths.spline(),
            points: { show: false },
            spanGaps: true,
            fill: (u) => {
              const ctx = u.ctx;
              const gradient = ctx.createLinearGradient(0, 0, 0, u.bbox.height);
              gradient.addColorStop(0, hexToRgba(color, 0.4));
              gradient.addColorStop(1, hexToRgba(color, 0.0));
              return gradient;
            },
          },
        ],
        hooks: {
          drawSeries: [
            (u) => {
              if (!supported || u.data[1].length === 0) return;
              const ctx = u.ctx;
              const lastIdx = u.data[0].length - 1;
              const cx = u.valToPos(u.data[0][lastIdx], "x", true);
              const cy = u.valToPos(u.data[1][lastIdx], "y", true);
              ctx.beginPath();
              ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
              ctx.fillStyle = color;
              ctx.fill();
            },
          ],
        },
      };

      chartInstance = new uPlot(opts, [timeData, valueData], chartRef);

      unsubscribe = canStore.subscribe((update) => {
        // Only care about updates for THIS specific chart's PID
        if (update.pid !== pid) return;

        // Push new data to the local arrays
        timeData.push(update.timestamp);
        valueData.push(update.value);

        // Slide the 60s window
        const cutoff = update.timestamp - 60000; // if timestamp is ms
        while (timeData.length > 0 && timeData[0] < cutoff) {
          timeData.shift();
          valueData.shift();
        }

        // Tell uPlot to redraw with the new points
        chartInstance.setData([timeData, valueData]);
      });
    };

    setup();

    return () => {
      unsubscribe(); // Stop listening when chart is destroyed
      if (chartInstance) chartInstance.destroy();
    };
  });
</script>

<article
  class="pid-card"
  class:disabled={!supported}
  style="background: color-mix(in srgb, {color} 5%, transparent);"
>
  <header class="card-header">
    <div
      class="icon"
      style="background: color-mix(in srgb, {color} 20%, transparent);"
    >
      <Icon name={icon} size={32} />
    </div>
    <div class="titles">
      <div class="label">{label}</div>
      <div
        class="subtitle"
        style="color: color-mix(in srgb, {color} 70%, transparent);"
      >
        {description}
      </div>
    </div>

    <div
      class="badge"
      style="background: color-mix(in srgb, {color} 10%, transparent); color: {color};"
    >
      {displayValue}<span class="unit">{unit}</span>
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
