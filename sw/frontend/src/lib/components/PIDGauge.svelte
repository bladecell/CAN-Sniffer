<script lang="ts">
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { PidGridItem } from "$lib/types";
  import Icon from "../Icon.svelte";
  import { Arc, Chart, Group, Layer } from "layerchart";

  interface Props {
    item: PidGridItem;
    [key: string]: any;
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

  <div class="dashboard-card-body">
    <Chart padding={{ top: 0, bottom: -20, left: 0, right: 0 }}>
      <Layer center>
        <Group y={12}>
          <Arc
            value={metric.currentValue}
            domain={[metric.min, metric.max]}
            range={[-120, 120]}
            innerRadius={-18}
            cornerRadius={5}
            motion="spring"
            fill={metric.color}
            track={{ fill: "var(--pico-muted-border-color)" }}
          >
            {#snippet children()}
              <g transform="translate(0, -6)">
                <text
                  text-anchor="middle"
                  dominant-baseline="central"
                  class="gauge-value"
                >
                  {metric.displayValue}
                </text>
                <text
                  y="32"
                  text-anchor="middle"
                  dominant-baseline="central"
                  class="gauge-unit"
                >
                  {metric.unit}
                </text>
              </g>
            {/snippet}
          </Arc>
        </Group>
      </Layer>
    </Chart>
  </div>
</article>

<style>
  .pid-gauge-card {
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .dashboard-card-body {
    flex: 1 1 0;
    min-height: 140px;
    width: 100%;
    display: flex;
    overflow: visible;
  }

  :global(.gauge-value) {
    font-size: 38px;
    font-weight: 800;
    fill: var(--pico-contrast);
    font-family: var(--pico-font-family-monospace);
  }

  :global(.gauge-unit) {
    font-size: 14px;
    fill: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }
</style>
