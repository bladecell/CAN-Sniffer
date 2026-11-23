<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { createSeriesFeed } from "$lib/telemetryChart/feed.svelte";
  import { createPresentClock } from "$lib/telemetryChart/window";
  import { holdEndsOf } from "$lib/telemetryChart/scroll";
  import { ChartScroll } from "$lib/telemetryChart/scroll.svelte";
  import { chartFrames } from "$lib/chartFrames.svelte";
  import { rangeOf } from "$lib/dashboardCardForm";
  import { chartWindowRead, formatReading, readState, traceShape, type ReadState } from "$lib/dashboardCardView";
  import { pidSources } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  const WINDOW_MS = 60_000;
  const W = 300;
  const H = 100;
  const LANE_H = 40;
  // Unique per card (and per lane below): a clip path id is referenced from this card only.
  const clipId = `chart-clip-${Math.random().toString(36).slice(2, 10)}`;

  const clock = createPresentClock();
  const feed = createSeriesFeed(() => card.pids, (t) => clock.observe(t, performance.now()));
  // Geometry is built for scroll.anchor and shifted per frame, as the telemetry plots are (telemetryChart/scroll.ts).
  const scroll = new ChartScroll(
    (time) => clock.now(time),
    () => holdEnds,
  );
  $effect(() => chartFrames.subscribe((time) => scroll.frame(time)));
  const polling = $derived(vehicleStore.obd2Status ? vehicleStore.obd2Status.polling : null);
  pidSources();

  interface Series {
    pid: number;
    name: string;
    unit: string;
    color: string;
    latest: number | null;
    line: string;
    area: string;
    end: { x: number; y: number } | null;
  }

  // Only the window's samples are read. Each line runs to the anchor and holds its last value while fresh, as the
  // telemetry plots do (series.ts); the layer is then shifted so the anchor lands on the present.
  const raw = $derived.by((): { pid: number; samples: { t: number[]; v: (number | null)[] } }[] => {
    void feed.version;
    return card.pids.map((pid) => ({ pid, samples: chartWindowRead(feed.read, pid, scroll.anchor, WINDOW_MS) }));
  });

  const holdOf = (pid: number) => Math.max(1500, 2 * (vehicleStore.pidDefinitions.find((d) => d.pid === pid)?.update_interval_ms || 500));
  const holdEnds = $derived<number[]>(holdEndsOf(raw.map((r) => ({ ...r.samples, holdMs: holdOf(r.pid) })), 0));

  const series = $derived<Series[]>(
    raw.map(({ pid, samples }) => {
      const def = vehicleStore.pidDefinitions.find((d) => d.pid === pid);
      const range = def ? rangeOf(def) : null;
      const hold = holdOf(pid);
      const to = scroll.anchor;
      const nums = samples.v.filter((v): v is number => v !== null);
      const lo = range ? range.min : nums.length ? Math.min(...nums) : 0;
      const hi = range ? range.max : nums.length ? Math.max(...nums) : 1;
      const box = card.style === "lanes" ? { w: W, h: LANE_H } : { w: W, h: H };
      const paths = traceShape(samples, to, WINDOW_MS, hold, { ...box, lo, hi });
      return {
        pid,
        name: def?.name ?? `PID ${pid}`,
        unit: def?.unit ?? "",
        color: safeHexColor(def?.color),
        latest: nums.length ? nums[nums.length - 1] : null,
        line: paths.line,
        area: paths.area,
        end: paths.end,
      };
    }),
  );

  /** Layer shift in viewBox units for this frame. */
  const offset = $derived(scroll.shift(W / WINDOW_MS));
  /** The end dot sits on the present: a held line's end is past it (clipped), a stale one moves with the layer. */
  const dotX = (x: number) => Math.min(x + offset, W);

  const hasSamples = $derived(series.some((s) => s.latest !== null));
  const groupState = $derived<ReadState>(
    hasSamples ? "value" : "nodata",
  );
  const titleName = $derived(series.map((s) => s.name).join(" / "));
  const showLegend = $derived(card.style !== "lanes" && (card.h >= 3 || card.w >= 4));
</script>

<div class="c" class:dim={groupState !== "value"} class:lanes={card.style === "lanes"}>
  <div class="hd">
    <div class="t1"><CardTitle name={titleName} state={groupState} iconOnly={card.w < 3} /></div>
    <span class="chip">1 min</span>
  </div>

  {#if card.style === "lanes"}
    <div class="lanes">
      {#each series as s (s.pid)}
        <div class="lane" style:--pc={s.color}>
          <div class="lh">
            <i class="sw" style:background={s.color}></i>
            <span class="ln ellipsis">{s.name}</span>
            <span class="lv">{s.latest === null ? "—" : formatReading(s.latest)}{s.unit && s.latest !== null ? ` ${s.unit}` : ""}</span>
          </div>
          <svg viewBox="-4 -3 {W + 8} {LANE_H + 6}" preserveAspectRatio="none" aria-hidden="true">
            <defs>
              <clipPath id="{clipId}-{s.pid}"><rect x="0" y="-4" width={W} height={LANE_H + 8} /></clipPath>
            </defs>
            <g clip-path="url(#{clipId}-{s.pid})">
              <path class="line" d={s.line} style:stroke={s.color} transform="translate({offset} 0)" />
            </g>
            {#if s.end}<path class="dot" d="M{dotX(s.end.x)},{s.end.y}l0.01,0" style:stroke={s.color} />{/if}
          </svg>
        </div>
      {/each}
    </div>
  {:else}
    {#if showLegend}
      <ul class="legend">
        {#each series as s (s.pid)}
          <li>{#if series.length > 1}<i class="sw" style:background={s.color}></i>{/if}<span class="ln ellipsis">{s.name}</span><span class="lv">{s.latest === null ? "—" : formatReading(s.latest)}{s.unit && s.latest !== null ? ` ${s.unit}` : ""}</span></li>
        {/each}
      </ul>
    {/if}
    <div class="plot">
      <svg viewBox="-4 -4 {W + 8} {H + 8}" preserveAspectRatio="none" aria-hidden="true">
        <defs>
          <clipPath id={clipId}><rect x="0" y="-4" width={W} height={H + 8} /></clipPath>
        </defs>
        <line class="mid" x1="-4" x2={W + 4} y1={H / 2} y2={H / 2} />
        <g clip-path="url(#{clipId})">
          <g transform="translate({offset} 0)">
            {#each series as s (s.pid)}
              {#if card.style === "area"}<path class="area" d={s.area} style:fill={s.color} />{/if}
              <path class="line" d={s.line} style:stroke={s.color} />
            {/each}
          </g>
        </g>
        {#each series as s (s.pid)}
          {#if s.end}<path class="dot" d="M{dotX(s.end.x)},{s.end.y}l0.01,0" style:stroke={s.color} />{/if}
        {/each}
      </svg>
      {#if !hasSamples}
        <span class="empty">{groupState === "off" ? "Polling off" : "No data"}</span>
      {/if}
    </div>
  {/if}
</div>

<style>
  .c {
    display: flex;
    flex-direction: column;
    gap: 4px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .hd {
    display: flex;
    align-items: flex-start;
    gap: 8px;
    min-width: 0;
  }
  .t1 {
    flex: 1;
    min-width: 0;
  }
  .chip {
    flex: none;
  }
  .mid {
    stroke: var(--hair);
    stroke-width: 1;
    vector-effect: non-scaling-stroke;
  }
  .plot {
    position: relative;
    flex: 1;
    min-height: 0;
  }
  .plot svg,
  .lane svg {
    display: block;
    width: 100%;
    height: 100%;
    overflow: visible;
  }
  .line {
    fill: none;
    stroke-width: 2;
    stroke-linejoin: round;
    stroke-linecap: round;
    vector-effect: non-scaling-stroke;
  }
  /* The newest sample: a zero-length stroke with round caps stays a circle under a stretched viewBox. */
  .dot {
    fill: none;
    stroke-width: 7;
    stroke-linecap: round;
    vector-effect: non-scaling-stroke;
  }
  .area {
    fill-opacity: 0.18;
    stroke: none;
  }
  .empty {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    color: var(--muted);
    font-size: 0.85rem;
    font-weight: 600;
  }
  .lanes {
    display: flex;
    flex: 1;
    flex-direction: column;
    gap: 6px;
    min-height: 0;
  }
  .lane {
    display: flex;
    flex: 1;
    flex-direction: column;
    gap: 2px;
    min-height: 0;
  }
  .lane svg {
    flex: 1;
    min-height: 0;
  }
  .lh,
  .legend li {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
    font-size: 0.8rem;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 2px 12px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .sw {
    flex: none;
    width: 8px;
    height: 8px;
    border-radius: 50%;
  }
  .ln {
    min-width: 0;
  }
  .legend .lv {
    margin-left: 0;
  }
  .lv {
    margin-left: auto;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .dim .line {
    opacity: 0.5;
  }
</style>
