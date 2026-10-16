<script lang="ts">
  import { Chart, Svg, Rect } from "layerchart";
  import { scaleLinear } from "d3-scale";
  import type { BarStyle } from "$lib/types";
  import {
    fillWidth,
    gradientStops,
    litSegments,
    markerOffset,
    segmentLayout,
    type BarZone,
  } from "$lib/valueBar";

  interface Props {
    /** Position on the bar, 0 to 1. Null is no data: no fill, no marker, no lit blocks. */
    fraction: number | null;
    color: string;
    barStyle?: BarStyle;
    /** Used by zones (bands) and gradient (colour stops). Fractions of the bar. */
    zones?: BarZone[];
  }

  let { fraction, color, barStyle = "solid", zones = [] }: Props = $props();

  const GAP_PX = 2;
  const MARKER_W = 3;
  const TRACK_LINE_PX = 3;
  const MUTED = "var(--pico-muted-border-color)";
  const uid = $props.id();
  const stops = $derived(gradientStops(color, zones));
</script>

<div class="value-bar">
  <Chart
    xScale={scaleLinear()}
    xDomain={[0, 1]}
    yDomain={[0, 1]}
    padding={{ top: 0, bottom: 0, left: 0, right: 0 }}
  >
    {#snippet children({ context })}
      <Svg>
        {#if barStyle === "zones"}
          {#each zones as zone, i (i)}
            <Rect
              x={context.xScale(zone.from) + GAP_PX / 2}
              y={0}
              width={Math.max(0, context.xScale(zone.to) - context.xScale(zone.from) - GAP_PX)}
              height={context.height}
              rx={2}
              fill={zone.color ?? color}
              opacity={fraction === null ? 0.35 : zone.opacity}
            />
          {/each}
          {#if fraction !== null}
            <Rect
              x={context.xScale(fraction) - MARKER_W / 2}
              y={0}
              width={MARKER_W}
              height={context.height}
              rx={2}
              fill="var(--pico-contrast)"
              class="marker"
            />
          {/if}
        {:else if barStyle === "segmented"}
          {@const seg = segmentLayout(context.width)}
          {@const lit = litSegments(fraction, seg.count)}
          {#each Array.from({ length: seg.count }, (_, i) => i) as i (i)}
            <Rect
              x={i * (seg.width + seg.gap)}
              y={0}
              width={seg.width}
              height={context.height}
              rx={1}
              fill={i < lit ? color : MUTED}
            />
          {/each}
        {:else if barStyle === "gradient"}
          <defs>
            <linearGradient
              id="value-bar-gradient-{uid}"
              gradientUnits="userSpaceOnUse"
              x1={0}
              x2={context.width}
              y1={0}
              y2={0}
            >
              {#each stops as stop, i (i)}
                <stop offset={stop.offset} stop-color={stop.color} stop-opacity={stop.opacity} />
              {/each}
            </linearGradient>
            <clipPath id="value-bar-track-{uid}">
              <rect x={0} y={0} width={context.width} height={context.height} rx={context.height / 2} />
            </clipPath>
          </defs>
          <Rect x={0} y={0} width={context.width} height={context.height} rx={context.height / 2} fill={MUTED} />
          <!-- Same clipped shape as solid: a low value is a sliver, not a dot. -->
          <g clip-path="url(#value-bar-track-{uid})">
            <Rect
              x={0}
              y={0}
              width={fillWidth(fraction, context.width)}
              height={context.height}
              fill="url(#value-bar-gradient-{uid})"
              class="fill"
            />
          </g>
        {:else if barStyle === "marker"}
          <Rect
            x={0}
            y={(context.height - TRACK_LINE_PX) / 2}
            width={context.width}
            height={TRACK_LINE_PX}
            rx={TRACK_LINE_PX / 2}
            fill={MUTED}
          />
          {#if fraction !== null}
            <Rect
              x={markerOffset(fraction, context.width, context.height) ?? 0}
              y={0}
              width={context.height}
              height={context.height}
              rx={context.height / 2}
              fill={color}
              class="marker"
            />
          {/if}
        {:else}
          <defs>
            <clipPath id="value-bar-track-{uid}">
              <rect x={0} y={0} width={context.width} height={context.height} rx={context.height / 2} />
            </clipPath>
          </defs>
          <Rect x={0} y={0} width={context.width} height={context.height} rx={context.height / 2} fill={MUTED} />
          <!-- Square fill clipped to the rounded track, as the old PIDBar did: a low value is a sliver, not a dot. -->
          <g clip-path="url(#value-bar-track-{uid})">
            <Rect
              x={0}
              y={0}
              width={fillWidth(fraction, context.width)}
              height={context.height}
              fill={color}
              class="fill"
            />
          </g>
        {/if}
      </Svg>
    {/snippet}
  </Chart>
</div>

<style>
  .value-bar {
    width: 100%;
    height: 100%;
    position: relative;
  }

  @media (prefers-reduced-motion: no-preference) {
    :global(.value-bar .fill),
    :global(.value-bar .marker) {
      transition:
        width 0.4s cubic-bezier(0.2, 0.8, 0.2, 1),
        x 0.4s cubic-bezier(0.2, 0.8, 0.2, 1);
    }
  }
</style>
