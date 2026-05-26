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
  class="pid-card"
  class:disabled={!metric.supported}
  style="background: color-mix(in srgb, {metric.color} 5%, transparent); --module-accent: {metric.color};"
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
  </header>

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
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
    border-color: color-mix(
      in srgb,
      var(--module-accent) 40%,
      var(--pico-muted-border-color)
    ) !important;
  }

  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 8px;
    background: none;
    border: none;
    flex-shrink: 0;
  }

  .label {
    font-size: 0.75rem;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: var(--pico-muted-color);
    font-weight: 500;
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

  .subtitle {
    font-size: 0.65rem;
    font-family: var(--pico-font-family-monospace);
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

  .disabled {
    opacity: 0.4;
    filter: grayscale(100%);
  }
</style>
