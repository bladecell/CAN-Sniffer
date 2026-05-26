<script lang="ts">
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { PidGridItem } from "$lib/types"; // Import our contract type
  import Icon from "../Icon.svelte";

  interface Props {
    item: PidGridItem; // Expect our structural routing definition explicitly
    [key: string]: any;
  }

  let { item, ...rest }: Props = $props();

  // Everything is fetched and automatically stays reactive using getters
  const metric = usePidData(() => item.pid);

  // Layout Constants
  const BAR_H = 10;
  const BAR_R = 5;
  const VB_W = 200;
  const VB_H = 55;
  const PAD = 8;
  const BAR_W = VB_W - PAD * 2;
  const BAR_Y = 22;

  // Compute layout values derived directly from the helper state
  const fillWidth = $derived.by((): number => {
    const clamped = Math.max(
      metric.min,
      Math.min(metric.max, metric.currentValue),
    );
    return ((clamped - metric.min) / (metric.max - metric.min)) * BAR_W;
  });
</script>

<article
  class="dashboard-card pid-bar-card"
  class:disabled={!metric.supported}
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
  </header>

  <div class="dashboard-card-body">
    <svg
      class="bar-svg"
      viewBox="0 0 {VB_W} {VB_H}"
      preserveAspectRatio="xMidYMid meet"
    >
      <defs>
        <clipPath id="bar-clip-{item.pid}">
          <rect x={PAD} y={BAR_Y} width={BAR_W} height={BAR_H} rx={BAR_R} />
        </clipPath>
      </defs>

      <text x={PAD + BAR_W} y={BAR_Y - 4} text-anchor="end" class="value-label">
        {metric.displayValue}<tspan class="unit-tspan"> {metric.unit}</tspan>
      </text>

      <rect
        x={PAD}
        y={BAR_Y}
        width={BAR_W}
        height={BAR_H}
        rx={BAR_R}
        fill="var(--pico-muted-border-color)"
      />

      <rect
        x={PAD}
        y={BAR_Y}
        width={fillWidth}
        height={BAR_H}
        rx={0}
        fill={metric.color}
        class="fill-bar"
        clip-path="url(#bar-clip-{item.pid})"
      />

      <text
        x={PAD}
        y={BAR_Y + BAR_H + 10}
        text-anchor="start"
        class="range-label">{metric.min}</text
      >
      <text
        x={PAD + BAR_W}
        y={BAR_Y + BAR_H + 10}
        text-anchor="end"
        class="range-label">{metric.max}</text
      >
    </svg>
  </div>
</article>

<style>
  /* All visual layout configurations remain completely untouched */
  .pid-bar-card {
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .bar-svg {
    flex: 1 1 0;
    min-height: 0;
    width: 100%;
    display: block;
    overflow: visible;
  }

  .fill-bar {
    transition: width 0.4s cubic-bezier(0.2, 0.8, 0.2, 1);
  }

  .value-label {
    font-size: 11px;
    font-weight: 800;
    font-family: var(--pico-font-family-monospace);
    transition: x 0.4s cubic-bezier(0.2, 0.8, 0.2, 1);
  }

  .unit-tspan {
    font-size: 7px;
    font-weight: 400;
  }

  .range-label {
    font-size: 7px;
    fill: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }
</style>
