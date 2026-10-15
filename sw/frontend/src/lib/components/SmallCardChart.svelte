<script lang="ts">
  import { Chart, Svg, Spline, Area } from "layerchart";
  import { scaleTime } from "d3-scale";

  interface ChartPoint {
    date: Date;
    value: number | null;
  }

  interface Props {
    data: ChartPoint[];
    xDomain: Date[] | undefined;
    yDomain: [number, number];
    color: string;
    gradientId: string;
    minHeight?: string;
  }

  let {
    data,
    xDomain,
    yDomain,
    color,
    gradientId,
    minHeight = "0",
  }: Props = $props();
</script>

<div class="chart-wrapper" style:--chart-min-height={minHeight}>
  {#if data.length > 0}
    <Chart
      {data}
      x="date"
      xScale={scaleTime()}
      {xDomain}
      y="value"
      {yDomain}
      padding={{ top: 10, bottom: 2, left: 0, right: 0 }}
    >
      <Svg>
        <defs>
          <linearGradient id={gradientId} x1="0" x2="0" y1="0" y2="1">
            <stop offset="0%" stop-color={color} stop-opacity="0.4" />
            <stop offset="100%" stop-color={color} stop-opacity="0.0" />
          </linearGradient>
        </defs>
        <Area fill="url(#{gradientId})" />
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
    min-height: var(--chart-min-height);
    min-width: 0;
    overflow: hidden;
  }
</style>
