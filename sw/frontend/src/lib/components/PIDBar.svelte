<script>
  import { canStore } from "$lib/canStore.svelte.js";
  import Icon from "../Icon.svelte";

  let {
    pid,
    label = "Metric",
    description = "",
    unit = "%",
    icon = "gear",
    color = "#10b981",
    min = 0,
    max = 100,
    ...rest
  } = $props();

  const BAR_H = 10;
  const BAR_R = 5;
  const VB_W = 200;
  const VB_H = 55;
  const PAD = 8;
  const BAR_W = VB_W - PAD * 2;
  const BAR_Y = 22;

  const pidData = $derived(canStore.pids.get(pid));
  const currentValue = $derived(pidData?.value ?? 0);
  const supported = $derived(pidData?.supported ?? false);
  const displayValue = $derived(supported ? currentValue.toFixed(1) : "···");

  const fillWidth = $derived.by(() => {
    const clamped = Math.max(min, Math.min(max, currentValue));
    return ((clamped - min) / (max - min)) * BAR_W;
  });

  // value label x position tracks the fill, clamped so it doesn't overflow
  const labelX = $derived(Math.min(PAD + fillWidth, PAD + BAR_W - 2));
</script>

<article
  class="pid-card"
  class:disabled={!supported}
  style="background: color-mix(in srgb, {color} 5%, transparent);"
  {...rest}
>
  <header class="card-header" data-swapy-handle>
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
  </header>

  <svg
    class="bar-svg"
    viewBox="0 0 {VB_W} {VB_H}"
    preserveAspectRatio="xMidYMid meet"
  >
    <defs>
      <clipPath id="bar-clip-{pid}">
        <rect x={PAD} y={BAR_Y} width={BAR_W} height={BAR_H} rx={BAR_R} />
      </clipPath>
    </defs>

    <!-- Value fixed at top right -->
    <text x={PAD + BAR_W} y={BAR_Y - 4} text-anchor="end" class="value-label"
      >{displayValue}<tspan class="unit-tspan"> {unit}</tspan></text
    >
    <!-- Track -->
    <rect
      x={PAD}
      y={BAR_Y}
      width={BAR_W}
      height={BAR_H}
      rx={BAR_R}
      fill="var(--pico-muted-border-color)"
    />
    <!-- Fill -->
    <rect
      x={PAD}
      y={BAR_Y}
      width={fillWidth}
      height={BAR_H}
      rx={0}
      fill={color}
      class="fill-bar"
      clip-path="url(#bar-clip-{pid})"
    />
    <!-- Min label -->
    <text x={PAD} y={BAR_Y + BAR_H + 10} text-anchor="start" class="range-label"
      >{min}</text
    >
    <!-- Max label -->
    <text
      x={PAD + BAR_W}
      y={BAR_Y + BAR_H + 10}
      text-anchor="end"
      class="range-label">{max}</text
    >
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
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
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

  .disabled {
    opacity: 0.4;
    filter: grayscale(100%);
  }
</style>
