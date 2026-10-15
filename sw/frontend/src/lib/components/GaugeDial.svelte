<script lang="ts">
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
  import type { GaugeStyle } from "$lib/types";

  interface Props {
    value: number;
    min: number;
    max: number;
    displayValue: string;
    unit: string;
    color: string;
    gaugeStyle?: GaugeStyle;
  }

  let { value, min, max, displayValue, unit, color, gaugeStyle }: Props = $props();

  let cw = $state(200);
  let ch = $state(140);
  // Scale down slightly so it doesn't overflow, and push the center up a bit
  const R = $derived(Math.max(10, Math.min(cw / 2.2, ch / 1.75)));

  const styleConfig = $derived.by(() => {
    switch (gaugeStyle) {
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

  const isSpeedometer = $derived(gaugeStyle === "speedometer");
  const isGradient = $derived(gaugeStyle === "gradient");

  // Use scaleLinear to map value to angle, and to generate clean "rounded" ticks
  const angleScale = $derived(
    scaleLinear().domain([min, max]).range(styleConfig.range),
  );

  const majorTicks = $derived(angleScale.ticks(8));
  const minorTicksTemp = $derived(angleScale.ticks(40));
  const minorTicks = $derived(
    minorTicksTemp.filter((t: number) => !majorTicks.includes(t)),
  );

  // Used by speedometer style
  const needleAngleRad = $derived(
    (angleScale(
      Math.max(min, Math.min(max, value)),
    ) *
      Math.PI) /
      180,
  );

  // Used by gradient style
  const gradientTicks = $derived(angleScale.ticks(4));
</script>

<div class="dashboard-card-body" bind:clientWidth={cw} bind:clientHeight={ch}>
    <Chart padding={styleConfig.padding}>
      <Layer center>
        <Group y={styleConfig.groupY}>
          {#if isSpeedometer}
            <Arc
              value={0}
              domain={[min, max]}
              range={styleConfig.range}
              outerRadius={R}
              innerRadius={R - 15}
              track={{ fill: color, opacity: 0.25 }}
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
              style="stroke: {color}"
              strokeWidth={3}
            />
            <Circle r={5} style="fill: var(--pico-contrast)" />

            <g transform={`translate(0, ${styleConfig.textTranslateY})`}>
              <text
                text-anchor="middle"
                dominant-baseline="central"
                class="gauge-value"
              >
                {displayValue}
              </text>
              <text
                y="32"
                text-anchor="middle"
                dominant-baseline="central"
                class="gauge-unit"
              >
                {unit}
              </text>
            </g>
          {:else if isGradient}
            <LinearGradient stops={["#10b981", "#eab308", "#ef4444"]}>
              {#snippet children({ gradient })}
                <ClipPath>
                  {#snippet clip()}
                    <Arc
                      value={value}
                      domain={[min, max]}
                      range={styleConfig.range}
                      outerRadius={R}
                      innerRadius={R - 18}
                      cornerRadius={6}
                      motion="spring"
                    />
                  {/snippet}
                  <Arc
                    value={max}
                    domain={[min, max]}
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
              value={max}
              domain={[min, max]}
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
                {displayValue}
              </text>
              <text
                y="32"
                text-anchor="middle"
                dominant-baseline="central"
                class="gauge-unit"
              >
                {unit}
              </text>
            </g>
          {:else}
            <Arc
              value={value}
              domain={[min, max]}
              range={styleConfig.range}
              innerRadius={-18}
              cornerRadius={5}
              motion="spring"
              fill={color}
              track={{ fill: "var(--pico-muted-border-color)" }}
            >
              {#snippet children()}
                <g transform={`translate(0, ${styleConfig.textTranslateY})`}>
                  <text
                    text-anchor="middle"
                    dominant-baseline="central"
                    class="gauge-value"
                  >
                    {displayValue}
                  </text>
                  <text
                    y="32"
                    text-anchor="middle"
                    dominant-baseline="central"
                    class="gauge-unit"
                  >
                    {unit}
                  </text>
                </g>
              {/snippet}
            </Arc>
          {/if}
        </Group>
      </Layer>
    </Chart>
</div>

<style>
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
