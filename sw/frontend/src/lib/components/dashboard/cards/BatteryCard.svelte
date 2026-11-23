<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { BATTERY_HISTORY_MS, carBatteryFeed } from "$lib/carBatteryFeed.svelte";
  import { BATTERY_LABEL, BATTERY_SCALE_MAX_V, BATTERY_SCALE_MIN_V, batteryReading } from "$lib/carBattery";
  import { emptyNote, traceShape, type Level, type ReadState } from "$lib/dashboardCardView";
  import { holdEndsOf } from "$lib/telemetryChart/scroll";
  import { ChartScroll } from "$lib/telemetryChart/scroll.svelte";
  import { chartFrames } from "$lib/chartFrames.svelte";
  import { batteryBands } from "$lib/dashboardCardLayout";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  /** A battery sample holds to the present this long: the feed's gap break (carBatteryFeed.svelte.ts). */
  const BATTERY_HOLD_MS = 2500;

  // The no-data rule needs a clock: a frame older than 3 s is no data. One tick a second is enough.
  let now = $state(Date.now());
  $effect(() => {
    const release = carBatteryFeed.attach();
    const timer = setInterval(() => (now = Date.now()), 1000);
    return () => {
      clearInterval(timer);
      release();
    };
  });

  const reading = $derived(
    batteryReading({
      volts: connectionStore.wsCanStatus.battery_voltage,
      frameAt: carBatteryFeed.frameAt,
      now,
      wsConnected: connectionStore.connected,
      busUp: connectionStore.wsCanStatus.state === "CAN Connected",
    }),
  );

  const titleState = $derived<ReadState>(reading.state === "nodata" ? "nodata" : "value");
  const level = $derived<Level | null>(
    reading.state === "critical" ? "crit" : reading.state === "low" || reading.state === "high" ? "warn" : null,
  );
  const okWord = $derived(level === null ? BATTERY_LABEL[reading.state] : undefined);
  const direction = $derived(reading.state === "high" ? "high" : "low");
  const fraction = $derived(reading.fraction);
  const fillColor = $derived(
    level === "crit" ? "var(--crit)" : level === "warn" ? "var(--warn)" : reading.state === "nodata" ? "var(--muted)" : "var(--ok)",
  );
  const valueColor = $derived(
    level === "crit" ? "var(--crit)" : level === "warn" ? "var(--warn)" : reading.state === "nodata" ? "var(--muted)" : "var(--text)",
  );
  const volts = $derived(reading.volts === null ? "—" : reading.volts.toFixed(1));
  const bands = batteryBands();
  const bandTone = { crit: "var(--crit)", warn: "var(--warn)", ok: "var(--ok)" } as const;
  // Mockup: from 4 columns and 3 rows the card adds a "Last 5 min" trace on the fixed battery scale.
  const big = $derived(card.h >= 3 && card.w >= 4);
  // Battery samples are stamped with performance.now(), the frame clock, so the present is the frame time itself.
  // The trace is built for scroll.anchor and shifted per frame (telemetryChart/scroll.ts). Samples come at 1 Hz; the
  // last one holds to the present for the same gap break as the feed.
  const trace = $derived(carBatteryFeed.snapshot());
  const holdEnds = $derived<number[]>(holdEndsOf([{ ...trace, holdMs: BATTERY_HOLD_MS }], 0));
  const scroll = new ChartScroll(
    (time) => time,
    () => holdEnds,
  );
  $effect(() => chartFrames.subscribe((time) => scroll.frame(time)));
  const clipId = `batt-clip-${Math.random().toString(36).slice(2, 10)}`;
  const offset = $derived(scroll.shift(100 / BATTERY_HISTORY_MS));
  const spark = $derived.by(() => {
    if (trace.t.length < 2) return null;
    return traceShape(trace, scroll.anchor, BATTERY_HISTORY_MS, BATTERY_HOLD_MS, { w: 100, h: 40, lo: BATTERY_SCALE_MIN_V, hi: BATTERY_SCALE_MAX_V });
  });
</script>

{#snippet history()}
  <div class="hist">
    <span class="cap">Last 5 min</span>
    {#if spark}
      <svg class="spark" style:color={fillColor} viewBox="-2 -3 104 46" preserveAspectRatio="none" aria-hidden="true">
        <defs><clipPath id={clipId}><rect x="0" y="-3" width="100" height="40" /></clipPath></defs>
        <g clip-path="url(#{clipId})"><g transform="translate({offset} 0)">
          <path class="area" d={spark.area} />
          <path class="line" d={spark.line} />
        </g></g>
      </svg>
    {/if}
  </div>
{/snippet}

<div class="bat" class:one={card.h === 1} class:glyph={card.style === "glyph"} class:dim={titleState !== "value"} data-level={level ?? "none"}>
  {#if titleState !== "value"}
    <CardTitle name="Battery" unit="V" state={titleState} iconOnly={card.w < 3} inline={card.h === 1} />
    <p class="db-note">{emptyNote(titleState)}</p>
  {:else if card.h === 1}
    <div class="head">
      <div class="t1"><CardTitle name="Battery" unit="V" state={titleState} {level} {direction} {okWord} iconOnly={card.w < 4} inline /></div>
      <span class="v" style:color={valueColor}>{volts}<span class="u"> V</span></span>
    </div>
  {:else}
    <CardTitle name="Battery" unit="V" state={titleState} {level} {direction} {okWord} iconOnly={card.w < 3} />
    {#if card.style === "glyph"}
      <div class="body">
        <svg class="gl" viewBox="0 0 64 30" aria-hidden="true">
          <rect class="shell" x="2" y="4" width="54" height="22" rx="5" />
          <rect class="nub" x="58" y="11" width="4" height="8" rx="1" />
          {#if fraction !== null}
            <rect class="charge" x="6" y="8" width={46 * fraction} height="14" rx="2" style:fill={fillColor} />
          {/if}
        </svg>
        <span class="v" style:color={valueColor}>{volts}<span class="u"> V</span></span>
        {#if big}{@render history()}{/if}
      </div>
    {:else}
      {#if big}
        <div class="row">
          <span class="v big" style:color={valueColor}>{volts}<span class="u"> V</span></span>
          {@render history()}
        </div>
      {:else}
        <span class="v big" style:color={valueColor}>{volts}<span class="u"> V</span></span>
      {/if}
      <div class="strip" aria-hidden="true">
        {#each bands as b, i (i)}
          <i style:left="{b.from * 100}%" style:width="{(b.to - b.from) * 100}%" style:background={bandTone[b.tone]}></i>
        {/each}
        {#if fraction !== null}<b class="needle" style:left="{fraction * 100}%" style:background={fillColor}></b>{/if}
      </div>
    {/if}
  {/if}
</div>

<style>
  .bat {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 5px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .bat.one {
    gap: 4px;
    padding: 6px 12px;
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
  .body {
    display: flex;
    align-items: center;
    gap: 10px;
    min-width: 0;
  }
  .row {
    display: flex;
    align-items: center;
    gap: 14px;
    min-width: 0;
  }
  .hist {
    display: flex;
    flex: 1;
    flex-direction: column;
    gap: 2px;
    align-self: stretch;
    min-width: 0;
    min-height: 0;
  }
  .cap {
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
    white-space: nowrap;
  }
  .spark {
    flex: 1;
    width: 100%;
    min-height: 24px;
    overflow: visible;
  }
  .spark .line {
    fill: none;
    stroke: currentColor;
    stroke-width: 1.75;
    stroke-linejoin: round;
    stroke-linecap: round;
    vector-effect: non-scaling-stroke;
  }
  .spark .area {
    fill: currentColor;
    fill-opacity: 0.16;
  }
  .gl {
    flex: none;
    width: 58px;
    height: 30px;
  }
  .shell {
    fill: none;
    stroke: var(--muted);
    stroke-width: 2;
  }
  .nub {
    fill: var(--muted);
  }
  .v {
    flex: none;
    font-size: 1.4rem;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    line-height: 1;
  }
  .one .v {
    font-size: 0.95rem;
  }
  .u {
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 700;
  }
  .one .u {
    font-size: 0.72rem;
  }
  /* Mockup bar: five voltage bands with a needle at the reading. */
  .strip {
    position: relative;
    height: 12px;
    margin-top: 2px;
    border-radius: 99px;
    overflow: visible;
  }
  .strip i {
    position: absolute;
    top: 0;
    bottom: 0;
    opacity: 0.4;
  }
  .strip i:first-child {
    border-radius: 99px 0 0 99px;
  }
  .strip i:last-child {
    border-radius: 0 99px 99px 0;
  }
  .needle {
    position: absolute;
    top: -4px;
    bottom: -4px;
    width: 3px;
    margin-left: -1px;
    border-radius: 2px;
    background: var(--text);
  }
  .dim .v {
    color: var(--muted);
    font-weight: 600;
  }
</style>
