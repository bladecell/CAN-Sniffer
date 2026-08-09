<script lang="ts">
  import { onMount, untrack } from "svelte";
  import {
    Chart,
    Svg,
    Spline,
    Axis,
    Tooltip,
    Highlight,
    ChartClipPath,
  } from "layerchart";
  import { scaleTime, scaleLinear } from "d3-scale";
  import { chartHistoryStore } from "$lib/chartHistoryStore";
  import { canStore } from "$lib/canStore.svelte.ts";
  import Icon from "$lib/Icon.svelte";

  interface Props {
    pids?: number[];
    isActive?: boolean;
  }
  let { pids = [], isActive = true }: Props = $props();

  // A map of PID -> array of data points
  let chartDataMap = $state<Record<number, { date: Date; value: number | null; pid: number }[]>>({});

  function getPidDef(pid: number) {
    return canStore.pidDefinitions?.find((c: any) => c.pid === pid);
  }

  function getPidColor(pid: number) {
    const def = getPidDef(pid);
    if (!def?.color) return "#10b981";
    if (typeof def.color === "number") {
      return `#${def.color.toString(16).padStart(6, "0")}`;
    }
    return def.color;
  }

  // Update chart data map when pids change
  $effect(() => {
    // Only react to changes in `pids`
    const currentPids = pids;
    
    untrack(() => {
      const newMap: Record<number, { date: Date; value: number | null; pid: number }[]> = {};
      for (const pid of currentPids) {
        if (chartDataMap[pid]) {
          newMap[pid] = chartDataMap[pid];
        } else {
          const unsubscribe = chartHistoryStore.subscribe(pid, (tData, vData) => {
            newMap[pid] = tData.map((t, i) => ({ date: new Date(t), value: vData[i], pid }));
          });
          unsubscribe(); // just want one-shot
        }
      }
      chartDataMap = newMap;
    });
  });

  // Listen to new events and push to the respective PID's array
  $effect(() => {
    if (pids.length === 0) return;

    const unsubscribeCan = canStore.subscribe((update) => {
      if (pids.includes(update.pid)) {
        if (!chartDataMap[update.pid]) chartDataMap[update.pid] = [];
        chartDataMap[update.pid].push({ date: new Date(update.timestamp), value: update.value, pid: update.pid });
      }
    });

    return () => unsubscribeCan();
  });

  let flatChartData = $derived(Object.values(chartDataMap).flat());

  let chartContext = $state<any>();

  let isZoomed = $derived(
    chartContext?.transformState &&
    (Math.abs(chartContext.transformState.scale - 1) > 0.001 ||
     Math.abs(chartContext.transformState.translate.x) > 0.001 ||
     Math.abs(chartContext.transformState.translate.y) > 0.001)
  );

  let autoXDomain = $derived.by(() => {
    let minT = Infinity;
    let maxT = -Infinity;
    for (const data of Object.values(chartDataMap)) {
      if (data.length > 0) {
        minT = Math.min(minT, data[0].date.getTime());
        maxT = Math.max(maxT, data[data.length - 1].date.getTime());
      }
    }
    return minT <= maxT && isFinite(minT) ? [new Date(minT), new Date(maxT)] : undefined;
  });

  let frozenXDomain = $state<[Date, Date] | undefined>();

  $effect(() => {
    if (isZoomed) {
      if (!frozenXDomain) frozenXDomain = autoXDomain as [Date, Date];
    } else {
      frozenXDomain = undefined;
    }
  });

  let activeXDomain = $derived(frozenXDomain ?? autoXDomain);

  let yDomain = $derived.by(() => {
    let minV = Infinity;
    let maxV = -Infinity;
    
    // Calculate based on actual data
    for (const data of flatChartData) {
      minV = Math.min(minV, data.value);
      maxV = Math.max(maxV, data.value);
    }
    
    if (minV === Infinity) minV = 0;
    if (maxV === -Infinity) maxV = 100;
    
    // Add 5% padding
    let padding = (maxV - minV) * 0.05;
    if (padding === 0) padding = 10;
    
    return [minV - padding, maxV + padding];
  });
</script>

{#if pids.length === 0}
  <div class="empty-state">
    Select one or more PIDs from the table to view on the chart
  </div>
{:else if flatChartData.length === 0}
  <div class="empty-state">
    Waiting for data...
  </div>
{:else}
  <div class="chart-wrapper">
    <div class="chart-inner-container">
      {#if isZoomed}
        <button
          class="secondary outline"
          style="position: absolute; top: 0.5rem; right: 1rem; z-index: 10; padding: 0.25rem 0.5rem; display: flex; align-items: center; gap: 0.25rem; font-size: 0.75rem;"
          onclick={() => chartContext?.transformState?.reset()}
          title="Reset Zoom"
        >
          <Icon name="folder-closed" size={14} /> <!-- simple fallback icon for reset -->
          Reset View
        </button>
      {/if}

      <Chart
        bind:context={chartContext}
        data={flatChartData}
        x="date"
        xScale={scaleTime()}
        xDomain={activeXDomain}
        y="value"
        yScale={scaleLinear()}
        yDomain={yDomain}
        padding={{ left: pids.length === 1 && getPidDef(pids[0])?.unit ? 80 : 56, bottom: 24, top: 16, right: 16 }}
        tooltipContext={{ mode: "bisect-x" }}
        transform={isActive ? { mode: "domain", axis: "x", scrollMode: "scale" } : undefined}
        brush={isActive ? { axis: "x", zoomOnBrush: true } : undefined}
      >
        {#snippet children({ context })}
          <Svg>
            <Axis
              placement="left"
              ticks={5}
              format={(d) => {
                const rounded = Math.round(d * 100) / 100;
                if (pids.length === 1) {
                  const unit = getPidDef(pids[0])?.unit;
                  return unit ? `${rounded} ${unit}` : `${rounded}`;
                }
                return `${rounded}`;
              }}
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

            <ChartClipPath>
              {#each pids as p}
                <Spline
                  data={chartDataMap[p] ?? []}
                  x="date"
                  y="value"
                  defined={(d) => d.value !== null}
                  stroke={getPidColor(p)}
                  strokeWidth={3}
                  class="fill-none chart-spline"
                />
              {/each}

              <Highlight
                axis="both"
                points={{
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
            </ChartClipPath>
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
              {#if data.value !== null}
                <div style="display:flex; align-items:center; gap: 6px;">
                  <div style="width:10px; height:10px; border-radius:50%; background-color:{getPidColor(data.pid)};"></div>
                  {Math.round(data.value * 100) / 100}{getPidDef(data.pid)?.unit ? " " + getPidDef(data.pid)?.unit : ""}
                </div>
              {/if}
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
    
    {#if pids.length > 0}
      <div class="custom-chart-legend">
        {#each pids as p}
          <div class="legend-item" style="border-color: {getPidColor(p)}20; background-color: {getPidColor(p)}15;">
            <div class="legend-color-dot" style="background-color: {getPidColor(p)};"></div>
            <span class="legend-label">{getPidDef(p)?.name || `0x${p.toString(16).toUpperCase()}`}</span>
          </div>
        {/each}
      </div>
    {/if}
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
    padding: 4px 10px;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 6px;
    white-space: nowrap;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
  }
  .chart-wrapper :global(svg) {
    overflow: visible !important;
  }

  .custom-chart-legend {
    display: flex;
    flex-wrap: wrap;
    justify-content: center;
    gap: 12px;
    margin-top: 12px;
    margin-bottom: 4px;
  }

  .legend-item {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 6px 12px;
    border-radius: 20px;
    border: 1px solid transparent;
    transition: background-color 0.2s, border-color 0.2s;
  }

  .legend-color-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    box-shadow: 0 0 0 1px rgba(255, 255, 255, 0.1);
  }

  .legend-label {
    font-size: 0.85rem;
    font-weight: 500;
    color: var(--pico-color);
  }
  .empty-state {
    display: flex;
    justify-content: center;
    align-items: center;
    height: 100%;
    min-height: 400px;
    color: var(--pico-muted-color);
  }

  .chart-wrapper {
    padding: 1rem;
    position: relative;
    display: flex;
    flex-direction: column;
  }

  .chart-inner-container {
    height: 400px;
    position: relative;
    width: 100%;
    flex-shrink: 0;
  }

  @media (max-width: 768px) {
    .empty-state {
      min-height: 250px;
    }
    .chart-wrapper {
      padding: 0.5rem;
    }
    .chart-inner-container {
      height: 250px;
    }
  }
</style>
