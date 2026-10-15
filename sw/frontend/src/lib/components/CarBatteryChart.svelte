<script lang="ts">
  import { Chart, Svg, Spline, Area } from "layerchart";
  import { scaleTime } from "d3-scale";
  import { carBatteryFeed, BATTERY_HISTORY_MS } from "$lib/carBatteryFeed.svelte";
  import { BATTERY_SCALE_MAX_V, BATTERY_SCALE_MIN_V } from "$lib/carBattery";

  interface Props {
    color: string;
  }

  let { color }: Props = $props();
  const uid = $props.id();

  const chartData = $derived.by(() => {
    const { t, v } = carBatteryFeed.snapshot();
    return t.map((time, i) => ({ date: new Date(time), value: v[i] }));
  });

  const xDomain = $derived(
    chartData.length > 0
      ? [
          new Date(chartData[chartData.length - 1].date.getTime() - BATTERY_HISTORY_MS),
          chartData[chartData.length - 1].date,
        ]
      : undefined,
  );
</script>

<div class="chart-wrapper">
  {#if chartData.length > 0}
    <Chart
      data={chartData}
      x="date"
      xScale={scaleTime()}
      {xDomain}
      y="value"
      yDomain={[BATTERY_SCALE_MIN_V, BATTERY_SCALE_MAX_V]}
      padding={{ top: 10, bottom: 2, left: 0, right: 0 }}
    >
      <Svg>
        <defs>
          <linearGradient id="battery-area-{uid}" x1="0" x2="0" y1="0" y2="1">
            <stop offset="0%" stop-color={color} stop-opacity="0.4" />
            <stop offset="100%" stop-color={color} stop-opacity="0.0" />
          </linearGradient>
        </defs>
        <Area fill="url(#battery-area-{uid})" />
        <Spline stroke={color} strokeWidth={3} class="fill-none" />
      </Svg>
    </Chart>
  {/if}
</div>

<style>
  .chart-wrapper {
    flex-grow: 1;
    width: calc(100% + 32px);
    margin-left: -16px;
    margin-right: -16px;
    position: relative;
    min-height: 120px;
    min-width: 0;
    overflow: hidden;
  }
</style>
