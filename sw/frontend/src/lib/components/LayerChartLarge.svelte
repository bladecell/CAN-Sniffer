<script lang="ts">
  import { onMount } from "svelte";
  import {
    Chart,
    Svg,
    Spline,
    Axis,
    Area,
    Tooltip,
    Highlight,
  } from "layerchart";
  import { scaleTime, scaleLinear } from "d3-scale";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import { chartHistoryStore } from "$lib/chartHistoryStore";
  import { canStore } from "$lib/canStore.svelte.ts";

  interface Props {
    pid?: number;
  }
  let { pid }: Props = $props();
  const metric = usePidData(() => pid ?? -1);

  // Derive chart data explicitly when triggered
  let chartData = $state<{ date: Date; value: number }[]>([]);

  $effect(() => {
    // When pid changes, reset data
    if (pid !== undefined) {
      chartData = [];
    }
  });

  $effect(() => {
    if (pid === undefined) return;

    // Seed initial data from history store (max 60s)
    const unsubscribeHistory = chartHistoryStore.subscribe(pid, (tData, vData) => {
      chartData = tData.map((t, i) => ({ date: new Date(t), value: vData[i] }));
    });
    unsubscribeHistory(); // Unsubscribe immediately, we just wanted the initial state

    // Listen to new events and accumulate locally infinitely while open
    const unsubscribeCan = canStore.subscribe((update) => {
      if (update.pid === pid) {
        chartData.push({ date: new Date(update.timestamp), value: update.value });
        chartData = [...chartData]; // trigger Svelte reactivity
      }
    });

    return () => {
      unsubscribeCan();
    };
  });
</script>

{#if pid === undefined}
  <div
    style="display:flex; justify-content:center; align-items:center; height: 100%; min-height: 400px; color: var(--pico-muted-color);"
  >
    Select a PID to view chart
  </div>
{:else if chartData.length === 0}
  <div
    style="display:flex; justify-content:center; align-items:center; height: 100%; min-height: 400px; color: var(--pico-muted-color);"
  >
    Waiting for data...
  </div>
{:else}
  <div
    class="chart-wrapper"
    style="height: 400px; padding: 1rem; position: relative;"
  >
    <Chart
      data={chartData}
      x="date"
      xScale={scaleTime()}
      y="value"
      yScale={scaleLinear()}
      padding={{ left: 56, bottom: 24, top: 16, right: 16 }}
      tooltipContext={{ mode: "bisect-x" }}
    >
      {#snippet children({ context })}
        <Svg>
          <Axis
            placement="left"
            ticks={5}
            format={(d) => `${d}${metric.unit ? " " + metric.unit : ""}`}
            grid={{ stroke: "rgba(128, 128, 128, 0.05)", dashArray: "2" }}
            rule={false}
            tickLabelProps={{
              fill: "var(--pico-muted-color)",
              fontSize: 11,
              textAnchor: "end",
              dx: -12,
            }}
          />
          <Axis
            placement="bottom"
            ticks={5}
            format={(d) =>
              new Date(d).toLocaleTimeString([], {
                hour12: false,
                minute: "2-digit",
                second: "2-digit",
              })}
            rule={{ stroke: "rgba(128, 128, 128, 0.1)" }}
            tickLabelProps={{
              fill: "var(--pico-muted-color)",
              fontSize: 11,
              dy: 16,
            }}
          />

          <Spline
            stroke={metric.color ?? "#01AAFF"}
            strokeWidth={3}
            class="fill-none chart-spline"
          />

          <Highlight
            axis="both"
            points={{
              fill: metric.color ?? "#01AAFF",
              stroke: "#ffffff",
              strokeWidth: 4,
              r: 6,
            }}
            lines={{
              stroke: "rgba(128, 128, 128, 0.4)",
              strokeWidth: 1,
              dashArray: "4",
            }}
          />
        </Svg>

        <Tooltip.Root
          {context}
          x={70}
          y="data"
          anchor="right"
          contained={false}
          variant="none"
          class="chart-tooltip-crosshair"
        >
          {#snippet children({ data })}
            {Math.round(data.value * 100) / 100}{metric.unit
              ? " " + metric.unit
              : ""}
          {/snippet}
        </Tooltip.Root>

        <Tooltip.Root
          {context}
          x="data"
          y={376}
          anchor="top"
          contained={false}
          variant="none"
          class="chart-tooltip-crosshair"
        >
          {#snippet children({ data })}
            {new Date(data.date).toLocaleTimeString([], {
              hour12: false,
              minute: "2-digit",
              second: "2-digit",
            })}
          {/snippet}
        </Tooltip.Root>
      {/snippet}
    </Chart>
  </div>
{/if}

<style>
  :global(.lc-text-svg) {
    overflow: visible;
  }
  :global(.chart-tick-label),
  :global(text) {
    fill: var(--pico-muted-color) !important;
    font-size: 0.75rem;
    font-weight: 500;
  }

  :global(.chart-spline) {
    stroke-linecap: round;
    stroke-linejoin: round;
  }

  :global(.chart-tooltip-crosshair) {
    font-size: 14px;
    font-weight: 600;
    color: var(--pico-h1-color);
    background-color: var(--pico-card-background-color);
    margin-top: 2px;
    padding: 2px 8px;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 4px;
    white-space: nowrap;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
  }
  .chart-wrapper :global(svg) {
    overflow: visible !important;
  }
</style>
