<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { barFraction } from "$lib/valueBar";
  import { rangeOf } from "$lib/dashboardCardForm";
  import { emptyNote, formatReading, levelOf, readState, zoneStops } from "$lib/dashboardCardView";
  import { pidSources } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

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
  const range = $derived(def ? rangeOf(def) : null);
  const fraction = $derived(value !== null && range ? barFraction(value, range.min, range.max) : null);
  const zones = $derived(range && card.thresholds ? zoneStops(range, card.thresholds) : null);
  const ticks = [0, 25, 50, 75, 100];
  // Whole numbers print without decimals on the scale (0 25 50 75 100), other values keep one.
  const tickLabel = (v: number) => formatReading(Number.isInteger(v) ? v : Math.round(v * 10) / 10).replace(/\.0$/, "");
  const oneRow = $derived(card.h === 1);

  pidSources();

  const fillColor = $derived(level === "crit" ? "var(--crit)" : level === "warn" ? "var(--warn)" : color);
  // Zone segments from the left edge: [start, width, tone] as percentages.
  const zoneSegs = $derived.by(() => {
    if (!zones) return [];
    return zones.lowBad
      ? [{ at: 0, w: zones.critical, tone: "crit" }, { at: zones.critical, w: zones.warn - zones.critical, tone: "warn" }]
      : [{ at: zones.warn, w: zones.critical - zones.warn, tone: "warn" }, { at: zones.critical, w: 1 - zones.critical, tone: "crit" }];
  });
</script>

<div class="b" class:one={oneRow} class:dim={state !== "value"} data-level={level ?? "none"} style:--pc={color}>
  {#if oneRow}
    <div class="head">
      <div class="t1"><CardTitle {name} {state} {level} {direction} iconOnly={card.w < 4} inline /></div>
      <span class="num" class:warn={level === "warn"} class:crit={level === "crit"}>{formatReading(value)}{#if value !== null && unit}<span class="u"> {unit}</span>{/if}</span>
    </div>
  {:else}
    <CardTitle {name} {state} {level} {direction} iconOnly={card.w < 4} />
    {#if state === "value"}
      <div class="vals">
        <span class="num" class:warn={level === "warn"} class:crit={level === "crit"}>{formatReading(value)}</span>
        {#if value !== null && unit}<span class="u">{unit}</span>{/if}
      </div>
    {:else}
      <p class="db-note">{emptyNote(state)}</p>
    {/if}
  {/if}

  {#if range && state === "value"}
    <div class="track" class:thick={!oneRow}>
      {#if fraction !== null}
        <i class="fill" style:width="{fraction * 100}%" style:background={fillColor}></i>
      {/if}
    </div>
    {#if card.style === "ticks"}
      <div class="ticks" aria-hidden="true">
        {#each ticks as q (q)}<i style:left="{q}%"></i>{/each}
      </div>
      {#if !oneRow}
        <div class="tl">
          {#each ticks as q (q)}<span>{tickLabel(range.min + ((range.max - range.min) * q) / 100)}</span>{/each}
        </div>
      {/if}
    {/if}
    {#if card.style === "zones" && zones}
      <div class="zstrip" class:thin={oneRow} aria-hidden="true">
        {#each zoneSegs as s (s.at)}<i class={s.tone} style:left="{s.at * 100}%" style:width="{s.w * 100}%"></i>{/each}
      </div>
      {#if !oneRow && card.thresholds && card.w >= 4}
        <div class="legend">
          {#if zones.lowBad}
            <span><i class="k crit"></i>Critical at or below {formatReading(card.thresholds.critical)}{unit ? ` ${unit}` : ""}</span>
            <span><i class="k warn"></i>Low at or below {formatReading(card.thresholds.warn)}{unit ? ` ${unit}` : ""}</span>
          {:else}
            <span><i class="k warn"></i>High from {formatReading(card.thresholds.warn)}{unit ? ` ${unit}` : ""}</span>
            <span><i class="k crit"></i>Critical from {formatReading(card.thresholds.critical)}{unit ? ` ${unit}` : ""}</span>
          {/if}
        </div>
      {/if}
    {/if}
  {/if}
</div>

<style>
  .b {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 5px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .b.one {
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
  .vals {
    display: flex;
    align-items: baseline;
    gap: 4px;
  }
  .num {
    flex: none;
    font-size: 1.5rem;
    font-weight: 800;
    line-height: 1;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .one .num {
    font-size: 0.95rem;
  }
  .num.warn {
    color: var(--warn);
  }
  .num.crit {
    color: var(--crit);
  }
  .u {
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 700;
  }
  .one .u {
    font-size: 0.72rem;
  }
  .dim .num {
    color: var(--muted);
    font-weight: 600;
  }
  .track {
    position: relative;
    height: 10px;
    border-radius: 99px;
    background: var(--ctl);
    overflow: hidden;
  }
  .track.thick {
    height: 14px;
  }
  .fill {
    position: absolute;
    left: 0;
    top: 0;
    bottom: 0;
    border-radius: 99px;
    background: var(--pc, var(--accent));
  }
  /* Mockup zones: a thin strip under the bar, critical solid and warning hatched. */
  .zstrip {
    position: relative;
    height: 4px;
    border-radius: 99px;
    overflow: hidden;
  }
  .zstrip i {
    position: absolute;
    top: 0;
    bottom: 0;
  }
  .zstrip i.crit {
    background: var(--crit);
  }
  .zstrip i.warn {
    background: repeating-linear-gradient(-45deg, var(--warn) 0 2px, transparent 2px 4px);
  }
  .ticks {
    position: relative;
    height: 5px;
  }
  .ticks i {
    position: absolute;
    top: 0;
    width: 1px;
    height: 5px;
    background: var(--muted);
  }
  .tl {
    position: relative;
    display: flex;
    justify-content: space-between;
    color: var(--muted);
    font-size: 0.7rem;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 2px 12px;
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
  }
  .k {
    display: inline-block;
    width: 8px;
    height: 8px;
    margin-right: 4px;
    border-radius: 2px;
    background: var(--ctl-text);
  }
  .k.warn {
    background: var(--warn);
  }
  .k.crit {
    background: var(--crit);
  }
</style>
