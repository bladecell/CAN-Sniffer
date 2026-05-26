<script lang="ts">
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { PidGridItem } from "$lib/types"; // Import our contract type
  import Icon from "../Icon.svelte";

  interface Props {
    item: PidGridItem; // Expect our structural routing definition explicitly
    [key: string]: any;
  }

  let { item, ...rest }: Props = $props();

  // Connect to our shared reactive library using the pid inside the layout item
  const metric = usePidData(() => item.pid);

  // ── Geometry (SVG user-unit space, original CX=50 CY=50) ──────────────────
  const R = 38;
  const STROKE = 8;
  const CX = 50;
  const CY = 50;

  const VB_X = 7.5;
  const VB_Y = 7.5;
  const VB_W = 85;
  const VB_H = 66;

  const toRad = (d: number) => (d * Math.PI) / 180;
  const sx = CX + R * Math.cos(toRad(210));
  const sy = CY - R * Math.sin(toRad(210));
  const ex = CX + R * Math.cos(toRad(-30));
  const ey = CY - R * Math.sin(toRad(-30));
  const trackD = `M ${sx.toFixed(3)} ${sy.toFixed(3)} A ${R} ${R} 0 1 1 ${ex.toFixed(3)} ${ey.toFixed(3)}`;
  const arcLen = 2 * Math.PI * R * (240 / 360);

  // Text anchor in viewBox space
  const TX = CX;
  const TY_val = CY - 4;
  const TY_unit = CY + 13;

  // Compute progress state using centralized helper properties
  const progress = $derived.by((): number => {
    const clamped = Math.max(
      metric.min,
      Math.min(metric.max, metric.currentValue),
    );
    return ((clamped - metric.min) / (metric.max - metric.min)) * arcLen;
  });
</script>

<article
  class="dashboard-card pid-gauge-card"
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
      class="gauge-svg"
      viewBox="{VB_X} {VB_Y} {VB_W} {VB_H}"
      preserveAspectRatio="xMidYMid meet"
    >
    <!-- Track -->
    <path
      d={trackD}
      fill="none"
      stroke="var(--pico-muted-border-color)"
      stroke-width={STROKE}
      stroke-linecap="round"
    />
    <!-- Progress -->
    <path
      d={trackD}
      fill="none"
      stroke={metric.color}
      stroke-width={STROKE}
      stroke-linecap="round"
      stroke-dasharray="{progress} {arcLen + 100}"
      class="progress-path"
    />
    <!-- Value -->
    <text
      x={TX}
      y={TY_val}
      text-anchor="middle"
      dominant-baseline="central"
      class="gauge-value"
    >
      {metric.displayValue}
    </text>
    <!-- Unit -->
    <text
      x={TX}
      y={TY_unit}
      text-anchor="middle"
      dominant-baseline="central"
      class="gauge-unit"
    >
      {metric.unit}
    </text>
  </svg>
  </div>
</article>

<style>
  .pid-gauge-card {
    transition: transform 0.2s ease, box-shadow 0.2s ease;
  }

  .gauge-svg {
    flex: 1 1 0;
    min-height: 0;
    width: 100%;
    display: block;
    overflow: visible;
  }

  .gauge-value {
    font-size: 14px;
    font-weight: 800;
    fill: var(--pico-contrast);
    font-family: var(--pico-font-family-monospace);
  }

  .gauge-unit {
    font-size: 7px;
    fill: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }

  .progress-path {
    transition: stroke-dasharray 0.4s cubic-bezier(0.2, 0.8, 0.2, 1);
  }
</style>
