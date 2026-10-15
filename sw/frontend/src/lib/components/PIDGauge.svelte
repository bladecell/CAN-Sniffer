<script lang="ts">
  import type { HTMLAttributes } from "svelte/elements";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { PidGridItem } from "$lib/types";
  import Icon from "../Icon.svelte";
  import GaugeDial from "./GaugeDial.svelte";

  interface Props extends HTMLAttributes<HTMLElement> {
    item: PidGridItem;
  }

  let { item, ...rest }: Props = $props();

  const metric = usePidData(() => item.pid);
</script>

<article
  class="dashboard-card pid-gauge-card"
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

  <GaugeDial
    value={metric.currentValue}
    min={metric.min}
    max={metric.max}
    displayValue={metric.displayValue}
    unit={metric.unit}
    color={metric.color}
    gaugeStyle={item.gaugeStyle}
  />
</article>

<style>
  .pid-gauge-card {
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }
</style>
