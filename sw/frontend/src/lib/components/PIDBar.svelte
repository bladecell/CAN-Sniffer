<script lang="ts">
  import type { HTMLAttributes } from "svelte/elements";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { PidGridItem } from "$lib/types";
  import { barFraction, equalThirds } from "$lib/valueBar";
  import Icon from "../Icon.svelte";
  import ValueBar from "./ValueBar.svelte";

  interface Props extends HTMLAttributes<HTMLElement> {
    item: PidGridItem;
  }

  let { item, ...rest }: Props = $props();

  // Everything is fetched and automatically stays reactive using getters
  const metric = usePidData(() => item.pid);

  // No data when the PID is unsupported or the value is not a number.
  const fraction = $derived(
    metric.isSupported ? barFraction(metric.currentValue, metric.min, metric.max) : null,
  );
  const zones = equalThirds();
</script>

<article
  class="dashboard-card pid-bar-card"
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
  </header>

  <!-- Focusable on purpose: keyboard and screen-reader users need a stop to hear the reading. -->
  <!-- svelte-ignore a11y_no_noninteractive_tabindex -->
  <!-- Sizes come from --u, which follows the card's width and height as the old SVG viewBox did. -->
  <div
    class="dashboard-card-body bar-body"
    role="meter"
    tabindex="0"
    aria-label={metric.label}
    aria-valuemin={metric.min}
    aria-valuemax={metric.max}
    aria-valuenow={fraction === null ? undefined : Math.min(metric.max, Math.max(metric.min, metric.currentValue))}
    aria-valuetext={fraction === null ? "No reading" : `${metric.displayValue} ${metric.unit}`}
  >
    <div class="value-label">
      {metric.displayValue}<span class="unit-tspan"> {metric.unit}</span>
    </div>

    <div class="bar-track">
      <ValueBar
        {fraction}
        color={metric.color}
        barStyle={item.barStyle ?? "solid"}
        {zones}
      />
    </div>

    <div class="range-row">
      <span>{metric.min}</span>
      <span>{metric.max}</span>
    </div>
  </div>
</article>

<style>
  /* All visual layout configurations remain completely untouched */
  /* The card is the size container. --u is one unit of the old 200 x 55 viewBox, scaled like its "meet" rule. */
  .pid-bar-card {
    container-type: size;
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .bar-body {
    --u: min(calc(100cqw / 200), calc(100cqh / 55));
    flex-direction: column;
    justify-content: center;
    gap: calc(var(--u) * 4);
    padding-inline: calc(var(--u) * 8);
  }

  .bar-body:focus-visible {
    outline: 3px solid var(--pico-contrast);
    outline-offset: 2px;
  }

  .value-label {
    align-self: flex-end;
    font-size: calc(var(--u) * 11);
    font-weight: 800;
    font-family: var(--pico-font-family-monospace);
    line-height: 1.2;
  }

  .unit-tspan {
    font-size: calc(var(--u) * 7);
    font-weight: 400;
  }

  .bar-track {
    height: calc(var(--u) * 10);
    width: 100%;
  }

  .range-row {
    display: flex;
    justify-content: space-between;
    font-size: calc(var(--u) * 7);
    color: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }
</style>
