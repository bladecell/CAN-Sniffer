<script lang="ts">
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { PidGridItem } from "$lib/types";
  import Icon from "../Icon.svelte";
  import {
    Arc,
    Chart,
    Group,
    Layer,
    Line,
    Circle,
    ClipPath,
    LinearGradient,
  } from "layerchart";
  import { scaleLinear } from "d3-scale";

  interface Props {
    item: PidGridItem;
    [key: string]: any;
  }

  let { item, ...rest }: Props = $props();

  const metric = usePidData(() => item.pid);

  let cw = $state(200);
  let ch = $state(140);
  // Scale down slightly so it doesn't overflow, and push the center up a bit
  const R = $derived(Math.max(10, Math.min(cw / 2.2, ch / 1.75)));

  const styleConfig = $derived.by(() => {
    switch (item.gaugeStyle) {
      case "donut":
        return {
          range: [0, 360] as [number, number],
          groupY: 0,
          textTranslateY: 0,
          padding: { top: 0, bottom: 0, left: 0, right: 0 },
        };
      case "half":
        return {
          range: [-90, 90] as [number, number],
          groupY: R * 0.6,
          textTranslateY: -15,
          padding: { top: 10, bottom: -30, left: 0, right: 0 },
        };
      case "speedometer":
        return {
          range: [-120, 120] as [number, number],
          groupY: R * 0.15,
          textTranslateY: 35,
          padding: { top: 0, bottom: -20, left: 0, right: 0 },
        };
      case "gradient":
        return {
          range: [-120, 120] as [number, number],
          groupY: R * 0.2,
          textTranslateY: -6,
          padding: { top: 0, bottom: -20, left: 0, right: 0 },
        };
      case "arc":
      default:
        return {
          range: [-120, 120] as [number, number],
          groupY: R * 0.2,
          textTranslateY: -6,
          padding: { top: 0, bottom: -20, left: 0, right: 0 },
        };
    }
  });

  const isSpeedometer = $derived(item.gaugeStyle === "speedometer");
  const isGradient = $derived(item.gaugeStyle === "gradient");

  // Use scaleLinear to map value to angle, and to generate clean "rounded" ticks
  const angleScale = $derived(
    scaleLinear().domain([metric.min, metric.max]).range(styleConfig.range),
  );

  const majorTicks = $derived(angleScale.ticks(8));
  const minorTicksTemp = $derived(angleScale.ticks(40));
  const minorTicks = $derived(
    minorTicksTemp.filter((t) => !majorTicks.includes(t)),
  );

  // Used by speedometer style
  const needleAngleRad = $derived(
    (angleScale(
      Math.max(metric.min, Math.min(metric.max, metric.currentValue)),
    ) *
      Math.PI) /
      180,
  );

  // Used by gradient style
  const gradientTicks = $derived(angleScale.ticks(4));
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

  <div class="dashboard-card-body" bind:clientWidth={cw} bind:clientHeight={ch}>
    <Chart padding={styleConfig.padding}>
      <Layer center>
        <Group y={styleConfig.groupY}>
          {#if isSpeedometer}
            <Arc
              value={0}
              domain={[metric.min, metric.max]}
              range={styleConfig.range}
              outerRadius={R}
              innerRadius={R - 15}
              track={{ fill: metric.color, opacity: 0.25 }}
            />

            {#each majorTicks as tick (tick)}
              {@const angleRad = (angleScale(tick) * Math.PI) / 180}
              <Line
                x1={Math.sin(angleRad) * (R - 27)}
                y1={-Math.cos(angleRad) * (R - 27)}
                x2={Math.sin(angleRad) * (R - 17)}
                y2={-Math.cos(angleRad) * (R - 17)}
                class="stroke-surface-content"
                style="stroke: var(--pico-contrast)"
                strokeWidth={2}
              />
              <text
                x={Math.sin(angleRad) * (R - 42)}
                y={-Math.cos(angleRad) * (R - 42)}
                text-anchor="middle"
                dominant-baseline="central"
                class="gauge-tick-label"
              >
                {Math.round(tick)}
              </text>
            {/each}

            {#each minorTicks as tick (tick)}
              {@const angleRad = (angleScale(tick) * Math.PI) / 180}
              <Line
                x1={Math.sin(angleRad) * (R - 22)}
                y1={-Math.cos(angleRad) * (R - 22)}
                x2={Math.sin(angleRad) * (R - 17)}
                y2={-Math.cos(angleRad) * (R - 17)}
                class="stroke-surface-content/40"
                style="stroke: var(--pico-contrast); opacity: 0.4"
                strokeWidth={1}
              />
            {/each}

            <Line
              x1={Math.sin(needleAngleRad) * -10}
              y1={-Math.cos(needleAngleRad) * -10}
              x2={Math.sin(needleAngleRad) * (R - 17)}
              y2={-Math.cos(needleAngleRad) * (R - 17)}
              style="stroke: {metric.color}"
              strokeWidth={3}
            />
            <Circle r={5} style="fill: var(--pico-contrast)" />

            <g transform={`translate(0, ${styleConfig.textTranslateY})`}>
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
          {:else if isGradient}
            <LinearGradient stops={["#10b981", "#eab308", "#ef4444"]}>
              {#snippet children({ gradient })}
                <ClipPath>
                  {#snippet clip()}
                    <Arc
                      value={metric.currentValue}
                      domain={[metric.min, metric.max]}
                      range={styleConfig.range}
                      outerRadius={R}
                      innerRadius={R - 18}
                      cornerRadius={6}
                      motion="spring"
                    />
                  {/snippet}
                  <Arc
                    value={metric.max}
                    domain={[metric.min, metric.max]}
                    range={styleConfig.range}
                    outerRadius={R}
                    innerRadius={R - 18}
                    cornerRadius={6}
                    fill={gradient}
                  />
                </ClipPath>
              {/snippet}
            </LinearGradient>

            <!-- Track outline -->
            <Arc
              value={metric.max}
              domain={[metric.min, metric.max]}
              range={styleConfig.range}
              outerRadius={R}
              innerRadius={R - 18}
              cornerRadius={6}
              class="fill-none"
              track={{
                fill: "none",
                stroke: "var(--pico-muted-border-color)",
                strokeWidth: 1.5,
                opacity: 0.4,
              }}
            />

            <!-- Tick marks and labels -->
            {#each gradientTicks as tick (tick)}
              {@const angleRad = (angleScale(tick) * Math.PI) / 180}
              <Line
                x1={Math.sin(angleRad) * (R - 21)}
                y1={-Math.cos(angleRad) * (R - 21)}
                x2={Math.sin(angleRad) * (R - 28)}
                y2={-Math.cos(angleRad) * (R - 28)}
                style="stroke: var(--pico-contrast); opacity: 0.4"
                strokeWidth={1.5}
              />
              <text
                x={Math.sin(angleRad) * (R - 38)}
                y={-Math.cos(angleRad) * (R - 38)}
                text-anchor="middle"
                dominant-baseline="central"
                class="gauge-tick-label"
              >
                {Math.round(tick)}
              </text>
            {/each}

            <!-- Value display -->
            <g transform={`translate(0, ${styleConfig.textTranslateY})`}>
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
          {:else}
            <Arc
              value={metric.currentValue}
              domain={[metric.min, metric.max]}
              range={styleConfig.range}
              innerRadius={-18}
              cornerRadius={5}
              motion="spring"
              fill={metric.color}
              track={{ fill: "var(--pico-muted-border-color)" }}
            >
              {#snippet children()}
                <g transform={`translate(0, ${styleConfig.textTranslateY})`}>
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
          {/if}
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
    font-size: 32px;
    font-weight: 800;
    fill: var(--pico-contrast);
    font-family: var(--pico-font-family-monospace);
  }

  :global(.gauge-unit) {
    font-size: 14px;
    fill: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }

  :global(.gauge-tick-label) {
    font-size: 10px;
    fill: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }
</style>
