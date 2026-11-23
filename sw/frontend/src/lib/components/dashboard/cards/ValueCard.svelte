<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { createSeriesFeed } from "$lib/telemetryChart/feed.svelte";
  import { createPresentClock } from "$lib/telemetryChart/window";
  import { holdEndsOf } from "$lib/telemetryChart/scroll";
  import { ChartScroll } from "$lib/telemetryChart/scroll.svelte";
  import { chartFrames } from "$lib/chartFrames.svelte";
  import { bridgeGaps, chartWindowRead, emptyNote, formatReading, levelOf, readState, sparkScale, traceShape, windowOf } from "$lib/dashboardCardView";
  import { pidSources } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  const WINDOW_MS = 60_000;
  const STATS_MS = 300_000;
  /** Samples closer than this are one trace: a slow PID is not a broken one. */
  const BRIDGE_MS = 10_000;
  const pid = $derived(card.pids[0]);
  const def = $derived(vehicleStore.pidDefinitions.find((d) => d.pid === pid));
  const polling = $derived(vehicleStore.obd2Status ? vehicleStore.obd2Status.polling : null);
  const entry = $derived(connectionStore.pids.get(pid));
  const state = $derived(readState(entry, polling));
  const value = $derived(state === "value" && entry ? entry.value : null);
  const level = $derived(levelOf(value, card.thresholds));
  const direction = $derived(card.thresholds && card.thresholds.warn > card.thresholds.critical ? "low" : "high");
  const color = $derived(safeHexColor(def?.color));
  const unit = $derived(def?.unit ?? "");
  const name = $derived(card.pids.length === 0 ? "Choose a PID" : (def?.name ?? `PID ${pid}`));

  pidSources();
  const clock = createPresentClock();
  const feed = createSeriesFeed(() => card.pids.slice(0, 1), (t) => clock.observe(t, performance.now()));
  // The sparkline is built for scroll.anchor and shifted per frame, as the Chart card is (telemetryChart/scroll.ts).
  const scroll = new ChartScroll(
    (time) => clock.now(time),
    () => holdEnds,
  );
  $effect(() => chartFrames.subscribe((time) => scroll.frame(time)));
  const clipId = `spark-clip-${Math.random().toString(36).slice(2, 10)}`;
  const hold = $derived(Math.max(1500, 2 * (def?.update_interval_ms || 500)));

  // The last minute of stored samples for the sparkline, the last five minutes for the min and max.
  const history = $derived.by(() => {
    void feed.version;
    return feed.read(pid, 0);
  });
  const recent = $derived(bridgeGaps(windowOf(history, WINDOW_MS), BRIDGE_MS));
  const stats = $derived.by(() => {
    const nums = (card.style === "minmax" ? windowOf(history, STATS_MS) : recent).v.filter((v): v is number => v !== null);
    if (nums.length === 0) return null;
    let lo = nums[0];
    let hi = nums[0];
    for (const n of nums) {
      if (n < lo) lo = n;
      if (n > hi) hi = n;
    }
    return { lo, hi };
  });
  // The strip's own scale: the data span, widened a little so a quiet signal stays flat.
  const scale = $derived.by(() => {
    const nums = recent.v.filter((v): v is number => v !== null);
    if (nums.length === 0) return null;
    return sparkScale(Math.min(...nums), Math.max(...nums), def && def.maxValue > def.minValue ? { min: def.minValue, max: def.maxValue } : null);
  });
  // The drawn samples: the window's own, bridged, read from the buffer (not the whole history).
  const drawn = $derived.by((): { t: number[]; v: (number | null)[] } => {
    void feed.version;
    return bridgeGaps(chartWindowRead(feed.read, pid, scroll.anchor, WINDOW_MS), BRIDGE_MS);
  });
  // A young trace enters from the right and slides left: the window ends at the anchor, the layer shifts to the present.
  const spark = $derived.by(() => {
    if (!scale || drawn.t.length < 2) return null;
    return traceShape(drawn, scroll.anchor, WINDOW_MS, hold, { w: 100, h: 40, lo: scale.lo, hi: scale.hi });
  });
  const offset = $derived(scroll.shift(100 / WINDOW_MS));
  const holdEnds = $derived<number[]>(holdEndsOf([{ ...drawn, holdMs: hold }], 0));

  const numPx = $derived(card.h >= 3 ? (card.w >= 6 ? 61 : 48) : 34);
  const oneRow = $derived(card.h === 1 && card.w >= 3);
  const showSpark = $derived(card.style === "spark" && card.h >= 2);
  // Mockup: a thin strip under the number; from 6 x 3 it grows, with the scale ends and the window caption.
  const tall = $derived(card.h >= 3 && card.w >= 6);
  const inlineSpark = $derived(card.style === "spark" && oneRow && card.w >= 4);
  const showMinMax = $derived(card.style === "minmax" && card.h >= 2);
  const showDesc = $derived(card.h >= 3 && card.w >= 6 && !!def?.description);
</script>

<div class="v" class:oneRow class:sparkOn={showSpark} class:dim={state !== "value"} data-level={level ?? "none"} style:--pc={color}>
  {#if oneRow}
    <div class="head">
      <div class="t1"><CardTitle {name} {state} {level} {direction} iconOnly={card.w < 3} inline /></div>
      {#if inlineSpark && spark}
        <svg class="spark inl" viewBox="-2 -3 104 46" preserveAspectRatio="none" aria-hidden="true">
          <defs><clipPath id="{clipId}-inl"><rect x="0" y="-3" width="100" height="40" /></clipPath></defs>
          <g clip-path="url(#{clipId}-inl)"><g transform="translate({offset} 0)">
            <path class="area" d={spark.area} />
            <path class="line" d={spark.line} />
          </g></g>
        </svg>
      {/if}
      <span class="num" class:warn={level === "warn"} class:crit={level === "crit"} style:font-size="min(20px, 24cqw)">{formatReading(value)}{#if unit && value !== null}<span class="u"> {unit}</span>{/if}</span>
    </div>
  {:else}
    <CardTitle {name} {state} {level} {direction} iconOnly={card.w < 3} />
    {#if showDesc}<span class="desc">{def?.description}</span>{/if}

    {#if state === "value"}
      <div class="row">
        <span class="num" class:warn={level === "warn"} class:crit={level === "crit"} style:font-size="min({numPx}px, 24cqw)">{formatReading(value)}</span>
        {#if unit && value !== null}<span class="u">{unit}</span>{/if}
      </div>
    {:else}
      <p class="db-note">{emptyNote(state)}</p>
    {/if}

    {#if showSpark}
      <div class="sp" class:tall>
        {#if tall && scale}
          <span class="ax" aria-hidden="true"><b>{formatReading(scale.hi)}</b><b>{formatReading(scale.lo)}</b></span>
        {/if}
        {#if spark}
          <svg class="spark" viewBox="-2 -3 104 46" preserveAspectRatio="none" aria-hidden="true">
            <defs><clipPath id="{clipId}-tall"><rect x="0" y="-3" width="100" height="40" /></clipPath></defs>
            <g clip-path="url(#{clipId}-tall)"><g transform="translate({offset} 0)">
              <path class="area" d={spark.area} />
              <path class="line" d={spark.line} />
            </g></g>
          </svg>
        {/if}
      </div>
      {#if tall}<span class="cap">Last 60 s</span>{/if}
    {/if}

    {#if showMinMax && stats}
      <div class="mm">
        <span>Min <b>{formatReading(stats.lo)}</b></span>
        <span>Max <b>{formatReading(stats.hi)}</b></span>
        {#if card.h >= 3}<span class="cap">Last 5 min</span>{/if}
      </div>
    {/if}
  {/if}
</div>

<style>
  .v {
    container-type: inline-size;
    display: flex;
    flex-direction: column;
    gap: 2px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .row {
    display: flex;
    align-items: flex-end;
    gap: 8px;
  }
  .row {
    margin-top: auto;
  }
  .head {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .t1 {
    flex: 1;
    min-width: 0;
  }
  .desc {
    overflow: hidden;
    color: var(--muted);
    font-size: 0.78rem;
    font-weight: 600;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .num {
    flex: none;
    font-weight: 800;
    line-height: 1;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .num.warn {
    color: var(--warn);
  }
  .num.crit {
    color: var(--crit);
  }
  .dim .num {
    color: var(--muted);
    font-weight: 600;
  }
  .u {
    margin-left: 4px;
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 700;
  }
  .oneRow .num {
    font-weight: 800;
  }
  /* Mockup "number and sparkline": the number, then a thin trace under it. */
  .sp {
    position: relative;
    display: flex;
    height: 18px;
    margin-top: 4px;
  }
  .sp.tall {
    height: 36px;
  }
  .ax {
    display: flex;
    flex: none;
    flex-direction: column;
    justify-content: space-between;
    margin-right: 6px;
    color: var(--muted);
    font-size: 0.75rem;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
  }
  .spark {
    overflow: visible;
    flex: 1;
    min-width: 0;
    height: 100%;
  }
  .spark.inl {
    flex: 0 1 30%;
    height: 22px;
  }
  .spark .line {
    fill: none;
    stroke: var(--pc, var(--accent));
    stroke-width: 1.75;
    stroke-linejoin: round;
    stroke-linecap: round;
    vector-effect: non-scaling-stroke;
  }
  .spark .area {
    fill: var(--pc, var(--accent));
    fill-opacity: 0.16;
  }
  .cap {
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
    white-space: nowrap;
  }
  .mm {
    display: flex;
    flex-wrap: wrap;
    gap: 2px 12px;
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 600;
  }
  .mm b {
    color: var(--text);
    font-variant-numeric: tabular-nums;
  }
</style>
