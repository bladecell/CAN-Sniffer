<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { createSeriesFeed } from "$lib/telemetryChart/feed.svelte";
  import { chartPaths, emptyNote, formatReading, levelOf, readState, windowOf } from "$lib/dashboardCardView";
  import { pidSources } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  const WINDOW_MS = 60_000;
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
  const feed = createSeriesFeed(() => card.pids.slice(0, 1), () => {});

  // The last minute of stored samples, for the sparkline and the min and max.
  const recent = $derived.by(() => {
    void feed.version;
    return windowOf(feed.read(pid, 0), WINDOW_MS);
  });
  const stats = $derived.by(() => {
    const nums = recent.v.filter((v): v is number => v !== null);
    if (nums.length === 0) return null;
    return { lo: Math.min(...nums), hi: Math.max(...nums) };
  });
  const spark = $derived.by(() => {
    if (!stats || recent.t.length < 2) return null;
    const to = recent.t[recent.t.length - 1];
    const lo = stats.lo;
    const hi = stats.hi;
    // A trace shorter than the window fills the width, so the first seconds after mounting still look like a trace.
    return chartPaths(recent, { w: 100, h: 40, from: Math.max(to - WINDOW_MS, recent.t[0]), to, lo, hi });
  });

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
      <div class="t1"><CardTitle {name} unit={unit} {state} {level} {direction} iconOnly={card.w < 3} inline /></div>
      {#if inlineSpark && spark}
        <svg class="spark inl" viewBox="0 0 100 40" preserveAspectRatio="none" aria-hidden="true">
          <path class="area" d={spark.area} />
          <path class="line" d={spark.line} />
        </svg>
      {/if}
      <span class="num" class:warn={level === "warn"} class:crit={level === "crit"} style:font-size="min(20px, 24cqw)">{formatReading(value)}{#if unit && value !== null}<span class="u"> {unit}</span>{/if}</span>
    </div>
  {:else}
    <CardTitle {name} unit={unit} {state} {level} {direction} iconOnly={card.w < 3} />
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
        {#if tall && stats}
          <span class="ax" aria-hidden="true"><b>{formatReading(stats.hi)}</b><b>{formatReading(stats.lo)}</b></span>
        {/if}
        {#if spark}
          <svg class="spark" viewBox="0 0 100 40" preserveAspectRatio="none" aria-hidden="true">
            <path class="area" d={spark.area} />
            <path class="line" d={spark.line} />
          </svg>
        {/if}
      </div>
      {#if tall}<span class="cap">Last 60 s</span>{/if}
    {/if}

    {#if showMinMax && stats}
      <div class="mm">
        <span>Min <b>{formatReading(stats.lo)}</b></span>
        <span>Max <b>{formatReading(stats.hi)}</b></span>
        {#if card.h >= 3}<span class="cap">Last 60 s</span>{/if}
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
    font-size: 0.68rem;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
  }
  .spark {
    flex: 1;
    min-width: 0;
    height: 100%;
    border-bottom: 1px solid var(--hair);
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
