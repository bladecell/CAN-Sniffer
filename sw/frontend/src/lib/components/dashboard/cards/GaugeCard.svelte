<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { barFraction } from "$lib/valueBar";
  import { rangeOf } from "$lib/dashboardCardForm";
  import {
    GAUGE_SWEEP,
    fitFont,
    formatReading,
    levelOf,
    readState,
    sweepPath,
    sweepPoint,
    sweepSpan,
    type Level,
    type ReadState,
    type Sweep,
  } from "$lib/dashboardCardView";
  import { pidSources } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  const polling = $derived(vehicleStore.obd2Status ? vehicleStore.obd2Status.polling : null);
  const single = $derived(card.pids.length === 1);
  // No PID yet: the dial is drawn empty under "Choose a PID", with the single-PID geometry.
  const empty = $derived(card.pids.length === 0);
  const one = $derived(single || empty);

  interface Item {
    pid: number;
    name: string;
    unit: string;
    color: string;
    state: ReadState;
    value: number | null;
    fraction: number | null;
    level: Level | null;
  }

  // A single PID can have thresholds; several PIDs share the dial and show their values in a list.
  const items = $derived<Item[]>(
    card.pids.map((pid) => {
      const def = vehicleStore.pidDefinitions.find((d) => d.pid === pid);
      const entry = connectionStore.pids.get(pid);
      const state = readState(entry, polling);
      const value = state === "value" && entry ? entry.value : null;
      const range = single && card.range ? card.range : def ? rangeOf(def) : null;
      return {
        pid,
        name: def?.name ?? `PID ${pid}`,
        unit: def?.unit ?? "",
        color: safeHexColor(def?.color),
        state,
        value,
        fraction: value !== null && range ? barFraction(value, range.min, range.max) : null,
        level: single ? levelOf(value, card.thresholds) : null,
      };
    }),
  );
  const first = $derived(items[0]);
  const firstDef = $derived(vehicleStore.pidDefinitions.find((d) => d.pid === card.pids[0]));
  // A card can carry its own scale (the card sheet); otherwise the PID range is used.
  const range = $derived(card.range ?? (firstDef ? rangeOf(firstDef) : null));
  const direction = $derived(card.thresholds && card.thresholds.warn > card.thresholds.critical ? "low" : "high");

  // Geometry per style, in the mockup's viewBox units (single PID: mockup GEO; several PIDs: MG).
  interface Geo {
    vb: string;
    cx: number;
    cy: number;
    sweep: Sweep;
    radii: number[];
    stroke: number;
    /** Outer radius of the threshold zone ring. */
    zone: number;
  }
  const geo = $derived<Geo>(
    card.style === "arc"
      ? one
        ? { vb: "8 4 114 68", cx: 65, cy: 62, sweep: GAUGE_SWEEP.arc, radii: [46], stroke: 10, zone: 56 }
        : { vb: "6 6 118 70", cx: 65, cy: 68, sweep: GAUGE_SWEEP.arc, radii: items.length === 2 ? [50, 36] : [52, 41, 30], stroke: items.length === 2 ? 11 : 9, zone: 0 }
      : card.style === "ring"
        ? one
          ? { vb: "3 3 94 94", cx: 50, cy: 50, sweep: GAUGE_SWEEP.ring, radii: [40], stroke: 10, zone: 47.5 }
          : { vb: "0 0 100 100", cx: 50, cy: 50, sweep: GAUGE_SWEEP.ring, radii: items.length === 2 ? [41, 28] : [42, 31, 20], stroke: items.length === 2 ? 10 : 8, zone: 0 }
        : { vb: "8 2 104 106", cx: 60, cy: 54, sweep: GAUGE_SWEEP.dial, radii: [42], stroke: 8, zone: 52 },
  );
  const dialTicks = $derived(
    card.style === "dial" && one
      ? Array.from({ length: 11 }, (_, i) => {
          const f = i / 10;
          const major = i % 5 === 0;
          const outer = sweepPoint(geo.cx, geo.cy, geo.radii[0] - (major ? 11 : 15), GAUGE_SWEEP.dial, f);
          const inner = sweepPoint(geo.cx, geo.cy, geo.radii[0] - 7, GAUGE_SWEEP.dial, f);
          return { x1: outer.x, y1: outer.y, x2: inner.x, y2: inner.y };
        })
      : [],
  );

  // Threshold zones: the fractions of warning and critical on the range, drawn on the outer ring.
  const zoneOf = (x: number | undefined) => (range && x !== undefined ? barFraction(x, range.min, range.max) : null);
  const zones = $derived.by(() => {
    const wf = zoneOf(card.thresholds?.warn);
    const cf = zoneOf(card.thresholds?.critical);
    if (!single || wf === null || cf === null || card.style === "bar" || geo.zone === 0) return null;
    return direction === "low"
      ? { crit: [0, cf] as const, warn: [cf, wf] as const }
      : { warn: [wf, cf] as const, crit: [cf, 1] as const };
  });

  pidSources();

  const ringColor = (it: Item) => (it.level === "crit" ? "var(--crit)" : it.level === "warn" ? "var(--warn)" : it.color);
  const combined = $derived(items.map((it) => it.name).join(" / "));
  // Several PIDs share one state word: value while any has a reading, else off or no data.
  const groupState = $derived<ReadState>(
    items.length === 0 || items.some((it) => it.state === "value") ? "value" : items.every((it) => it.state === "off") ? "off" : "nodata",
  );
  const ring = (r: number) => 2 * Math.PI * r;

  // The centre number shrinks until it fits inside the arc, the ring or the dial face (viewBox units).
  const shownNumber = $derived(first?.state === "value" ? formatReading(first.value) : "—");
  const numSize = $derived(card.style === "arc" ? 23 : card.style === "dial" ? 16 : 25);
  const numWidth = $derived(card.style === "dial" ? 60 : 2 * (geo.radii[0] - geo.stroke / 2) - 12);
</script>

<div class="g" class:dim={groupState !== "value"} class:multi={!one} class:side={!one && card.w >= 4} data-level={single ? (first.level ?? "none") : "none"}>
  <CardTitle
    name={empty ? "Choose a PID" : single ? first.name : combined}
    unit={single ? first.unit : undefined}
    state={single ? first.state : groupState}
    level={single ? first.level : null}
    {direction}
    iconOnly={card.w < 3}
  />

  <div class="gb">
  <div class="dial">
    <svg viewBox={geo.vb} aria-hidden="true">
      {#if empty}
        <path class="track" d={sweepPath(geo.cx, geo.cy, geo.radii[0], geo.sweep, 1)} stroke-width={geo.stroke} />
      {/if}
      {#each items as it, i (it.pid)}
        {@const r = geo.radii[i]}
        {#if card.style === "dial"}
          {#if i === 0}<path class="track" d={sweepPath(geo.cx, geo.cy, geo.radii[0], geo.sweep, 1)} stroke-width={geo.stroke} />{/if}
        {:else if card.style === "ring"}
          <circle class="track" cx={geo.cx} cy={geo.cy} r={r} stroke-width={geo.stroke} />
          {#if it.fraction !== null && it.fraction > 0}
            <circle
              class="prog"
              cx={geo.cx}
              cy={geo.cy}
              r={r}
              stroke-width={geo.stroke}
              stroke={ringColor(it)}
              stroke-dasharray="{it.fraction * ring(r)} {ring(r)}"
              transform="rotate(-90 {geo.cx} {geo.cy})"
            />
          {/if}
        {:else}
          <path class="track" d={sweepPath(geo.cx, geo.cy, r, geo.sweep, 1)} stroke-width={geo.stroke} />
          {#if it.fraction !== null && it.fraction > 0}
            <path class="prog" d={sweepPath(geo.cx, geo.cy, r, geo.sweep, it.fraction)} stroke-width={geo.stroke} stroke={ringColor(it)} />
          {/if}
        {/if}
      {/each}

      {#if zones}
        {#if zones.warn}
          <path class="zone warn" d={sweepSpan(geo.cx, geo.cy, geo.zone, geo.sweep, zones.warn[0], zones.warn[1])} stroke-width="3" />
        {/if}
        {#if zones.crit}
          <path class="zone crit" d={sweepSpan(geo.cx, geo.cy, geo.zone, geo.sweep, zones.crit[0], zones.crit[1])} stroke-width="3" />
        {/if}
      {/if}

      {#each dialTicks as t, i (i)}
        <line class="tick" x1={t.x1} y1={t.y1} x2={t.x2} y2={t.y2} />
      {/each}

      {#if card.style === "dial"}
        {#each items as it, i (it.pid)}
          {#if it.fraction !== null}
            {@const tip = sweepPoint(geo.cx, geo.cy, geo.radii[0] - 8, GAUGE_SWEEP.dial, it.fraction)}
            <line class="needle" x1={geo.cx} y1={geo.cy} x2={tip.x} y2={tip.y} stroke={i === 0 ? "var(--text)" : ringColor(it)} />
          {/if}
        {/each}
        <circle class="hub" cx={geo.cx} cy={geo.cy} r="4.5" />
      {/if}

      {#if one}
        <text
          class="num"
          class:warn={first?.level === "warn"}
          class:crit={first?.level === "crit"}
          class:dimmed={first?.state !== "value"}
          x={geo.cx}
          y={card.style === "arc" ? 60 : card.style === "dial" ? 102 : 58}
          font-size={fitFont(numSize, shownNumber.length, numWidth)}>{shownNumber}</text
        >
      {/if}
    </svg>
  </div>

  {#if !one}
    <ul class="list">
      {#each items as it (it.pid)}
        <li>
          <i class="sw" style:background={it.color}></i>
          <span class="ln">{it.name}</span>
          <span class="lv">{formatReading(it.value)}{it.unit && it.value !== null ? ` ${it.unit}` : ""}</span>
        </li>
      {/each}
    </ul>
  {/if}
  </div>
</div>

<style>
  .g {
    display: flex;
    flex-direction: column;
    gap: 2px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .dial {
    position: relative;
    flex: 1;
    min-height: 0;
    display: grid;
    place-items: center;
  }
  .gb {
    display: flex;
    flex: 1;
    flex-direction: column;
    min-height: 0;
    min-width: 0;
  }
  /* Several PIDs: the legend sits beside the dial from 4 columns, below it when narrower. */
  .g.side .gb {
    flex-direction: row;
    align-items: center;
    gap: 8px;
  }
  .g.side .dial {
    flex: 0 0 42%;
    height: 100%;
  }
  .g.side .list {
    flex: 1;
    min-width: 0;
  }
  /* Mockup legend: name on the first line, value and unit under it. */
  .g.side .list li {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr);
    column-gap: 6px;
  }
  .g.side .sw {
    grid-row: span 2;
    align-self: center;
  }
  .g.side .lv {
    grid-column: 2;
    margin-left: 0;
  }
  .g.side .ln {
    grid-column: 2;
  }
  .g.multi:not(.side) .dial {
    flex: 0 1 auto;
    width: 100%;
    max-height: 60%;
  }
  .dial svg {
    width: 100%;
    height: 100%;
    max-height: 100%;
    overflow: visible;
  }
  .track {
    fill: none;
    stroke: var(--ctl);
    stroke-linecap: round;
  }
  .prog {
    fill: none;
    stroke: var(--pc, var(--accent));
    stroke-linecap: round;
  }
  .zone {
    fill: none;
  }
  .zone.warn {
    stroke: var(--warn);
    stroke-dasharray: 2 2.5;
  }
  .zone.crit {
    stroke: var(--crit);
  }
  .tick {
    stroke: var(--track);
    stroke-width: 1.5;
  }
  .needle {
    stroke-width: 3;
    stroke-linecap: round;
  }
  .hub {
    fill: var(--text);
  }
  circle.track,
  circle.prog {
    stroke-linecap: butt;
  }
  .num {
    fill: var(--text);
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    text-anchor: middle;
  }
  .num.warn {
    fill: var(--warn);
  }
  .num.crit {
    fill: var(--crit);
  }
  .num.dimmed,
  .dim .num {
    fill: var(--muted);
  }
  .list {
    display: grid;
    gap: 2px;
    margin: 0;
    padding: 0;
    list-style: none;
    font-size: 0.8rem;
  }
  .list li {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
  }
  .sw {
    flex: none;
    width: 8px;
    height: 8px;
    border-radius: 50%;
  }
  .ln {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .lv {
    margin-left: auto;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
</style>
