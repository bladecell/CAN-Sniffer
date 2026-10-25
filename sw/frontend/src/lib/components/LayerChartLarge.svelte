<script lang="ts">
import { vehicleStore } from "$lib/vehicleStore.svelte";


  import { untrack } from "svelte";
  import { safeHexColor } from "$lib/color";
  import {
    Chart,
    Svg,
    Spline,
    Axis,
    Tooltip,
    Highlight,
    ChartClipPath,
    Circle,
  } from "layerchart";
  import { scaleTime, scaleLinear } from "d3-scale";
  import { chartHistoryStore } from "$lib/chartHistoryStore";
  import { chartSyncStore } from "$lib/chartSyncStore.svelte";
  import Icon from "$lib/Icon.svelte";

  interface Props {
    pids?: number[];
    isActive?: boolean;
    /** Out: true while the user has zoomed or brushed the x axis. The owning cell shows its reset tools from this. */
    zoomed?: boolean;
  }
  let { pids = [], isActive = true, zoomed = $bindable(false) }: Props = $props();

  type Point = { t: number; v: number | null; pid: number };

  // Each cell reads the shared, bounded history (chartBuffer.ts). It is copied at most 5 Hz.
  let chartDataMap = $state.raw<Record<number, Point[]>>({});

  $effect(() => {
    // Rebuilt from the current pids, so data of removed PIDs is released.
    untrack(() => {
      chartDataMap = {};
    });
    const unsubscribes = pids.map((pid) =>
      chartHistoryStore.subscribe(pid, (buffer) => {
        const { t, v } = buffer.snapshot();
        const points = t.map((time, i) => ({ t: time, v: v[i], pid }));
        untrack(() => {
          chartDataMap = { ...chartDataMap, [pid]: points };
        });
      }),
    );
    return () => unsubscribes.forEach((unsubscribe) => unsubscribe());
  });

  let defByPid = $derived(
    new Map(vehicleStore.pidDefinitions.map((d) => [d.pid, d])),
  );

  let flatChartData = $derived(pids.flatMap((pid) => chartDataMap[pid] ?? []));

  function getPidDef(pid: number | undefined) {
    return pid === undefined ? undefined : defByPid.get(pid);
  }

  function getPidColor(pid: number | undefined) {
    return safeHexColor(getPidDef(pid)?.color);
  }

  let chartContext = $state<any>();

  $effect(() => {
    zoomed = Boolean(isZoomed);
  });

  /** Called by the owning cell: back to the full x range and the auto y range. */
  export function reset() {
    chartContext?.transformState?.reset();
    yDomainOverride = null;
  }

  /** Called by the owning cell: fit the y axis to the visible x range. */
  export function fitY() {
    adjustYToZoom();
  }

  let isZoomed = $derived(
    chartContext?.transformState &&
      (Math.abs(chartContext.transformState.scale - 1) > 0.001 ||
        Math.abs(chartContext.transformState.translate.x) > 0.001 ||
        Math.abs(chartContext.transformState.translate.y) > 0.001),
  );

  let autoXDomain = $derived.by(() => {
    let minT = Infinity;
    let maxT = -Infinity;
    for (const pid of pids) {
      const data = chartDataMap[pid];
      if (data && data.length > 0) {
        minT = Math.min(minT, data[0].t);
        maxT = Math.max(maxT, data[data.length - 1].t);
      }
    }
    return minT <= maxT && isFinite(minT)
      ? [new Date(minT), new Date(maxT)]
      : undefined;
  });

  let frozenXDomain = $state<[Date, Date] | undefined>();

  $effect(() => {
    if (isZoomed) {
      if (!frozenXDomain) frozenXDomain = autoXDomain as [Date, Date];
    } else {
      frozenXDomain = undefined;
    }
  });

  $effect(() => {
    if (!chartSyncStore.syncEnabled || !isActive) return;
    const d = chartContext?.xDomain;
    chartSyncStore.domain = isZoomed && d ? [Number(d[0]), Number(d[1])] : null;
  });

  let syncedDomain = $derived(
    !isActive && chartSyncStore.syncEnabled && chartSyncStore.domain
      ? [new Date(chartSyncStore.domain[0]), new Date(chartSyncStore.domain[1])]
      : undefined,
  );

  let activeXDomain = $derived(
    isActive ? (frozenXDomain ?? autoXDomain) : (syncedDomain ?? autoXDomain),
  );

  let yDomainOverride = $state<[number, number] | null>(null);

  function adjustYToZoom() {
    const xd = chartContext?.xDomain as [Date, Date] | undefined;
    if (!xd) return;
    const xMin = Number(xd[0]);
    const xMax = Number(xd[1]);

    let minV = Infinity;
    let maxV = -Infinity;
    for (const data of flatChartData) {
      if (data.v === null) continue;
      if (data.t < xMin || data.t > xMax) continue;
      minV = Math.min(minV, data.v);
      maxV = Math.max(maxV, data.v);
    }
    if (!isFinite(minV) || !isFinite(maxV)) return;

    let padding = (maxV - minV) * 0.05;
    if (padding === 0) padding = 10;
    yDomainOverride = [minV - padding, maxV + padding];
  }

  let yDomain = $derived.by(() => {
    if (yDomainOverride) return yDomainOverride;

    let minV = Infinity;
    let maxV = -Infinity;

    // Calculate based on actual data
    for (const data of flatChartData) {
      if (data.v === null) continue;
      minV = Math.min(minV, data.v);
      maxV = Math.max(maxV, data.v);
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
    Select one or more PIDs from the list to view on the chart
  </div>
{:else if flatChartData.length === 0}
  <div class="empty-state">Waiting for data...</div>
{:else}
  <div class="chart-wrapper">
    <div class="chart-inner-container">
      <Chart
        bind:context={chartContext}
        data={flatChartData}
        x="t"
        xScale={scaleTime()}
        xDomain={activeXDomain}
        y="v"
        yScale={scaleLinear()}
        {yDomain}
        padding={{
          left: pids.length === 1 && getPidDef(pids[0])?.unit ? 80 : 56,
          bottom: 24,
          top: 16,
          right: 16,
        }}
        tooltipContext={{ mode: "quadtree" }}
        transform={isActive
          ? { mode: "domain", axis: "x", scrollMode: "scale" }
          : undefined}
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
              grid={{ dashArray: "2" }}
              rule={false}
              tickLabelProps={{
                fill: "var(--muted)",
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
              rule={{ dashArray: "2" }}
              tickLabelProps={{
                fill: "var(--muted)",
                fontSize: 11,
                dy: 16,
              }}
            />

            <ChartClipPath>
              {#each pids as p (p)}
                <Spline
                  data={chartDataMap[p] ?? []}
                  x="t"
                  y="v"
                  defined={(d: { t: number; v: number | null; pid: number }) =>
                    d.v !== null}
                  stroke={getPidColor(p)}
                  strokeWidth={3}
                  class="fill-none chart-spline"
                />
              {/each}

              <Highlight
                axis="both"
                lines={{
                  stroke: "rgba(128, 128, 128, 0.4)",
                  strokeWidth: 1,
                  dashArray: "4",
                }}
              >
                {#snippet points({ points })}
                  {#each points as point}
                    <Circle
                      cx={point.x}
                      cy={point.y}
                      fill={getPidColor(
                        flatChartData.find(
                          (d) => d.t === point.data.x && d.v === point.data.y,
                        )?.pid,
                      )}
                      stroke="#ffffff"
                      strokeWidth={4}
                      r={6}
                    />
                  {/each}
                {/snippet}
              </Highlight>
            </ChartClipPath>
          </Svg>

          <Tooltip.Root
            {context}
            x={70}
            y="data"
            anchor="right"
            contained={false}
            variant="none"
            class="chart-tooltip-crosshair tel-float"
          >
            {#snippet children({ data })}
              {#if data.v !== null}
                {Math.round(data.v * 100) / 100}{getPidDef(data.pid)?.unit
                  ? " " + getPidDef(data.pid)?.unit
                  : ""}
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
            class="chart-tooltip-crosshair tel-float"
          >
            {#snippet children({ data })}
              {new Date(data.t).toLocaleTimeString([], {
                hour12: false,
                minute: "2-digit",
                second: "2-digit",
              })}
            {/snippet}
          </Tooltip.Root>
        {/snippet}
      </Chart>
    </div>

  </div>
{/if}

<style>
  :global(.lc-text-svg) {
    overflow: visible;
  }
  :global(.chart-tick-label),
  :global(text) {
    fill: var(--muted) !important;
    font-size: 0.75rem;
    font-weight: 500;
    font-variant-numeric: tabular-nums;
  }
  /* Grid, rules and crosshair lines use the theme grid token (presentation attributes cannot take var()). */
  .chart-inner-container :global(svg line) {
    stroke: var(--grid);
  }

  :global(.chart-spline) {
    stroke-linecap: round;
    stroke-linejoin: round;
  }

  :global(.chart-tooltip-crosshair) {
    font-size: 14px;
    font-weight: 600;
    color: var(--text);
    margin-top: 2px;
    padding: 4px 10px;
    border-radius: 10px;
    white-space: nowrap;
    border: 0 !important;
    font-variant-numeric: tabular-nums;
  }
  .chart-wrapper :global(svg) {
    overflow: visible !important;
  }

  .empty-state {
    display: flex;
    justify-content: center;
    align-items: center;
    height: 100%;
    min-height: 400px;
    padding: 16px;
    text-align: center;
    color: var(--muted);
  }

  .chart-wrapper {
    position: relative;
    display: flex;
    flex-direction: column;
    min-width: 0;
  }

  .chart-inner-container {
    height: var(--plot-h, 360px);
    position: relative;
    width: 100%;
    flex-shrink: 0;
  }

  @media (max-width: 768px) {
    .empty-state {
      min-height: 250px;
    }
    .chart-inner-container {
      height: 250px;
    }
  }
</style>
