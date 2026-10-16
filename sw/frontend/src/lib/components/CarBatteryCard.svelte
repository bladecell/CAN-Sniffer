<script lang="ts">
  import type { HTMLAttributes } from "svelte/elements";
  import Icon from "$lib/Icon.svelte";
  import GaugeDial from "./GaugeDial.svelte";
  import ValueBar from "./ValueBar.svelte";
  import CarBatteryChart from "./CarBatteryChart.svelte";
  import { canStore } from "$lib/canStore.svelte";
  import { carBatteryFeed } from "$lib/carBatteryFeed.svelte";
  import {
    batteryReading,
    BATTERY_COLOR,
    BATTERY_LABEL,
    BATTERY_SCALE_MAX_V,
    BATTERY_SCALE_MIN_V,
    clampFraction,
    type BatteryState,
  } from "$lib/carBattery";
  import type { BatteryGridItem } from "$lib/types";

  interface Props extends HTMLAttributes<HTMLElement> {
    item: BatteryGridItem;
  }

  let { item, ...rest }: Props = $props();

  // The frame carries no time, so staleness is judged on the local clock. The tick is 500 ms, so a stale
  // card turns to no data within about 3.5 s. `now` never lags the last frame, so a frame between ticks is not stale.
  let tick = $state(Date.now());
  const now = $derived(Math.max(tick, carBatteryFeed.frameAt));
  $effect(() => {
    const timer = setInterval(() => (tick = Date.now()), 500);
    return () => clearInterval(timer);
  });
  // Sampling runs while a card is mounted.
  $effect(() => carBatteryFeed.attach());

  const reading = $derived(
    batteryReading({
      volts: canStore.wsCanStatus.battery_voltage,
      frameAt: carBatteryFeed.frameAt,
      now,
      wsConnected: canStore.connected,
      busUp: canStore.wsCanStatus.state === "CAN Connected",
    }),
  );
  const color = $derived(BATTERY_COLOR[reading.state]);
  const ariaNow = $derived(
    reading.volts === null
      ? undefined
      : Math.min(BATTERY_SCALE_MAX_V, Math.max(BATTERY_SCALE_MIN_V, reading.volts)),
  );
  const valueText = $derived(reading.volts === null ? "--" : reading.volts.toFixed(1));
  const valueLabel = $derived(
    reading.volts === null
      ? "No valid reading"
      : `${valueText} volts, ${BATTERY_LABEL[reading.state]}`,
  );

  // One zone per band of batteryState(). The band the reading is in is drawn at full strength.
  const BANDS: { from: number; to: number; band: BatteryState }[] = [
    { from: BATTERY_SCALE_MIN_V, to: 10.5, band: "critical" },
    { from: 10.5, to: 11.8, band: "low" },
    { from: 11.8, to: 13.2, band: "ok" },
    { from: 13.2, to: 15, band: "charging" },
    { from: 15, to: BATTERY_SCALE_MAX_V, band: "high" },
  ];
  const batteryZones = $derived(
    BANDS.map((z) => ({
      from: clampFraction(z.from),
      to: clampFraction(z.to),
      color: BATTERY_COLOR[z.band],
      opacity: reading.state === z.band ? 1 : 0.7,
    })),
  );

  // Ticks at the lower bounds of the low, ok and charging bands.
  const TICKS = [10.5, 11.8, 13.2];
  const GLYPH_TOP = 26;
  const GLYPH_H = 46;
  const tickY = (v: number) =>
    GLYPH_TOP + GLYPH_H - ((v - BATTERY_SCALE_MIN_V) / (BATTERY_SCALE_MAX_V - BATTERY_SCALE_MIN_V)) * GLYPH_H;
</script>

<article
  class="dashboard-card battery-card"
  data-state={reading.state}
  style:--c={color}
  style:--module-accent={color}
  {...rest}
>
  <header class="dashboard-card-header">
    <div class="dashboard-card-icon">
      <Icon name="engine" size={32} />
    </div>
    <div class="dashboard-card-titles">
      <div class="dashboard-card-label">Car battery</div>
      <div class="dashboard-card-subtitle">OBD port voltage</div>
    </div>
    {#if item.displayMode === "chart"}
      <div class="chart-badge" aria-label={valueLabel}>
        {valueText}<span class="unit">V</span>
      </div>
    {/if}
  </header>

  <!-- Focusable on purpose: keyboard and screen-reader users need a stop to hear the reading. -->
  <!-- svelte-ignore a11y_no_noninteractive_tabindex -->
  <div
    class="battery-meter"
    role={reading.volts === null ? "img" : "meter"}
    tabindex="0"
    aria-label="Car battery voltage"
    aria-valuemin={reading.volts === null ? undefined : BATTERY_SCALE_MIN_V}
    aria-valuemax={reading.volts === null ? undefined : BATTERY_SCALE_MAX_V}
    aria-valuenow={ariaNow}
    aria-valuetext={valueLabel}
  >
    {#if item.displayMode === "bar"}
      <div class="mode-bar">
        <div class="value-row">
          <span class="value">{valueText}</span><span class="unit">V</span>
        </div>
        <ValueBar
          fraction={reading.fraction}
          color={color}
          barStyle={item.barStyle ?? "zones"}
          zones={batteryZones}
        />
      </div>
    {:else if item.displayMode === "gauge"}
      <GaugeDial
        value={reading.volts ?? BATTERY_SCALE_MIN_V}
        min={BATTERY_SCALE_MIN_V}
        max={BATTERY_SCALE_MAX_V}
        displayValue={valueText}
        unit="V"
        {color}
        gaugeStyle={item.gaugeStyle}
      />
    {:else if item.displayMode === "chart"}
      <CarBatteryChart {color} />
    {:else}
      <div class="mode-glyph">
        <svg
          class="glyph"
          viewBox="0 0 132 84"
          width="132"
          height="84"
          style:--p={reading.fraction}
          aria-hidden="true"
          focusable="false"
        >
          <rect class="post" x="16" y="0" width="12" height="7" rx="2" />
          <rect class="post" x="72" y="0" width="12" height="7" rx="2" />
          <rect class="case" x="4" y="7" width="96" height="71" rx="6" />
          <path class="case" d="M4 22h96" />
          <rect class="cap" x="20" y="12" width="10" height="5" rx="1" />
          <rect class="cap" x="44" y="12" width="10" height="5" rx="1" />
          <rect class="cap" x="68" y="12" width="10" height="5" rx="1" />
          <rect class="track" x="10" y={GLYPH_TOP} width="84" height={GLYPH_H} rx="3" />
          <rect class="fill" x="10" y={GLYPH_TOP} width="84" height={GLYPH_H} rx="3" />
          {#each TICKS as tick (tick)}
            <path class="tk" d="M10 {tickY(tick)}H94" />
            <text x="106" y={tickY(tick) + 4}>{tick.toFixed(1)}</text>
          {/each}
        </svg>
        <div class="glyph-side">
          <div class="value-row">
            <span class="value">{valueText}</span><span class="unit">V</span>
          </div>
        </div>
      </div>
    {/if}
  </div>
</article>

<style>
  .battery-card {
    --bc-text: #c2c7d0;
    --bc-strong: #dfe3eb;
    --bc-muted: #7b8495;
    --bc-soft: #8f9aab;
    --bc-track: #313033;
    background: color-mix(in srgb, var(--c) 5%, transparent);
    transition: box-shadow 0.2s ease;
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
    color: var(--c);
    background: color-mix(in srgb, var(--c) 10%, transparent);
  }

  .chart-badge .unit {
    font-size: 0.8rem;
    font-weight: 500;
    color: var(--c);
  }

  .battery-meter {
    /* The gauge's 140 px minimum would push the status footer out of the card. */
    --gauge-min-h: 0px;
    flex: 1 1 0;
    min-height: 0;
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 8px;
    border-radius: 6px;
  }

  .battery-meter:focus-visible {
    outline: 3px solid var(--bc-strong);
    outline-offset: 2px;
  }

  .value-row {
    display: flex;
    align-items: baseline;
    gap: 6px;
  }

  .value {
    font-family: var(--pico-font-family-monospace);
    font-size: 2rem;
    font-weight: 700;
    line-height: 1;
    color: var(--bc-text);
  }

  .unit {
    font-family: var(--pico-font-family-monospace);
    font-size: 1rem;
    color: var(--bc-muted);
  }

  .mode-glyph {
    flex: 1 1 auto;
    display: flex;
    align-items: center;
    gap: 16px;
    min-width: 0;
  }

  .glyph {
    flex: none;
    width: 132px;
    max-width: 60%;
    height: auto;
    overflow: visible;
  }

  .glyph .post {
    fill: var(--bc-soft);
  }

  .glyph .case {
    fill: none;
    stroke: var(--bc-soft);
    stroke-width: 2;
  }

  .glyph .cap {
    fill: var(--bc-muted);
  }

  .glyph .track {
    fill: var(--bc-track);
  }

  .glyph .fill {
    fill: var(--c);
    transform-box: fill-box;
    transform-origin: 50% 100%;
    transform: scaleY(var(--p));
  }

  .glyph .tk {
    stroke: var(--pico-card-background, #181c25);
    stroke-width: 2;
  }

  .glyph text {
    fill: var(--bc-soft);
    font: 11px var(--pico-font-family-monospace);
  }

  .glyph-side {
    display: flex;
    flex-direction: column;
    gap: 8px;
    min-width: 0;
    align-self: flex-end;
    margin-left: auto;
  }

  .mode-bar {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }

  .battery-card[data-state="nodata"] .glyph .case {
    stroke-dasharray: 4 3;
  }

  .battery-card[data-state="nodata"] .glyph .fill {
    display: none;
  }

  .battery-card[data-state="nodata"] .value {
    color: var(--bc-soft);
  }

  @media (prefers-reduced-motion: no-preference) {
    .glyph .fill {
      transition: transform 0.4s ease;
    }
  }
</style>
