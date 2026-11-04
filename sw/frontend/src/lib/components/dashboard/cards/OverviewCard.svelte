<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { formatReading, readState, type ReadState } from "$lib/dashboardCardView";
  import { faultChip } from "$lib/dashboardCardLayout";
  import { loadWhileMounted, pidSources } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();
  const items = $derived(card.items ?? []);
  const hasVin = $derived(items.some((i) => i.t === "vin"));

  // The DTC count is "No data" until the first read has finished. The store alone cannot tell zero from unread.
  let dtcRead = $state(false);
  pidSources();
  // The header chip always shows the fault count, so the DTCs are read whether or not the card lists them.
  loadWhileMounted(async () => {
    if (hasVin) void vehicleStore.getVin();
    await vehicleStore.getDTC();
    dtcRead = true;
  });
  const chip = $derived(faultChip(dtcRead ? vehicleStore.totalDTCs : null));

  const polling = $derived(vehicleStore.obd2Status ? vehicleStore.obd2Status.polling : null);

  interface Tile {
    key: string;
    label: string;
    value: string;
    unit: string;
    state: ReadState;
    /** VIN tiles always take the full width so the whole number shows. */
    wide: boolean;
  }

  // Tiles in the order the items were added. A label set on the item replaces the PID name.
  const tiles = $derived.by<Tile[]>(() =>
    items.map((item, i): Tile => {
      if (item.t === "vin") {
        const vin = vehicleStore.vin;
        return { key: `vin-${i}`, label: item.label || "VIN", value: vin || "—", unit: "", state: vin ? "value" : "nodata", wide: true };
      }
      if (item.t === "dtc") {
        return {
          key: `dtc-${i}`,
          label: item.label || "DTCs",
          value: dtcRead ? String(vehicleStore.totalDTCs) : "—",
          unit: "",
          state: dtcRead ? "value" : "nodata",
          wide: false,
        };
      }
      const def = vehicleStore.pidDefinitions.find((d) => d.pid === item.pid);
      const entry = connectionStore.pids.get(item.pid);
      const state = readState(entry, polling);
      const value = state === "value" && entry ? entry.value : null;
      return {
        key: `pid-${item.pid}-${i}`,
        label: item.label || def?.name || `PID ${item.pid}`,
        value: formatReading(value),
        unit: value !== null ? (def?.unit ?? "") : "",
        state,
        wide: false,
      };
    }),
  );

  const hero = $derived(card.style === "hero" ? tiles[0] : undefined);
  const rest = $derived(card.style === "hero" ? tiles.slice(1) : tiles);
</script>

<div class="o" class:list={card.style === "list"} class:hero={card.style === "hero"} class:wide={card.style === "hero" && card.h <= 2 && card.w >= 5}>
  <div class="hd">
    <div class="t1"><CardTitle name="Overview" iconOnly={card.w < 3} /></div>
    <span class="chip {chip.tone}">{chip.text}</span>
  </div>

  {#if hero}
    <div class="big" class:dim={hero.state !== "value"}>
      <span class="hl">{hero.label}</span>
      <span class="hv">{hero.value}{#if hero.unit}<span class="u"> {hero.unit}</span>{/if}</span>
      {#if hero.state !== "value"}<span class="hs">{hero.state === "off" ? "Polling off" : "No data"}</span>{/if}
    </div>
  {/if}

  <div class="items">
    {#each rest as t (t.key)}
      <div class="tile" class:wide={t.wide || card.style === "list"} class:dim={t.state !== "value"}>
        <span class="tl">{t.label}</span>
        <span class="tv" class:vin={t.key.startsWith("vin-")}>{t.value}{#if t.unit}<span class="u"> {t.unit}</span>{/if}</span>
        {#if t.state === "off"}<span class="ts">Polling off</span>{/if}
      </div>
    {/each}
  </div>
</div>

<style>
  .hd {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .t1 {
    flex: 1;
    min-width: 0;
  }
  .chip {
    flex: none;
    padding: 2px 8px;
    border-radius: 99px;
    background: var(--ctl);
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 800;
    white-space: nowrap;
  }
  .chip.ok {
    color: var(--ok);
  }
  .chip.warn {
    color: var(--warn);
  }
  .o {
    display: flex;
    flex-direction: column;
    gap: 6px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
    container-type: size;
  }
  .items {
    display: grid;
    flex: 1;
    grid-template-columns: repeat(auto-fit, minmax(92px, 1fr));
    grid-auto-rows: min-content;
    align-content: start;
    gap: 4px 10px;
    min-height: 0;
    overflow: auto;
  }
  .tile {
    display: flex;
    flex-direction: column;
    min-width: 0;
  }
  .tile.wide {
    grid-column: 1 / -1;
  }
  .tl {
    overflow: hidden;
    color: var(--muted);
    font-size: 0.75rem;
    font-weight: 700;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .tv {
    font-size: 1.1rem;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
  }
  .tv.vin {
    font-family: var(--mono, ui-monospace, monospace);
    font-size: 0.8rem;
    font-weight: 700;
    letter-spacing: 0.02em;
  }
  .u {
    color: var(--muted);
    font-size: 0.75rem;
    font-weight: 700;
  }
  .ts {
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
  }
  .dim .tv,
  .tile.dim .tv {
    color: var(--muted);
    font-weight: 600;
  }
  .list .items {
    grid-template-columns: 1fr;
    gap: 2px;
  }
  .list .tile {
    flex-direction: row;
    align-items: baseline;
    justify-content: space-between;
    gap: 8px;
    min-height: 32px;
  }
  /* Mockup hero 6 x 2: the headline on the left, the sub-values on the right. */
  .o.wide {
    display: grid;
    grid-template-columns: minmax(0, 2fr) minmax(0, 3fr);
    grid-template-rows: auto minmax(0, 1fr);
    align-content: start;
    column-gap: 12px;
  }
  .wide .hd {
    grid-column: 1 / -1;
  }
  .wide .big {
    grid-column: 1;
    grid-row: 2;
    justify-content: center;
  }
  .wide .items {
    grid-column: 2;
    grid-row: 2;
  }
  .hero .big {
    display: flex;
    flex-direction: column;
    gap: 2px;
  }
  .hl {
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 700;
  }
  .hv {
    font-size: 2rem;
    font-weight: 800;
    line-height: 1;
    font-variant-numeric: tabular-nums;
  }
  .hs {
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 600;
  }
  .big.dim .hv {
    color: var(--muted);
  }
  /* Short cards (2 rows): shrink type and put every tile's label and value on one line so all items still show. */
  @container (max-height: 140px) {
    .o:not(.wide) {
      gap: 2px;
    }
    .items {
      gap: 2px 10px;
    }
    .tl {
      font-size: 0.68rem;
    }
    .tv {
      font-size: 0.9rem;
      line-height: 1.15;
    }
    .tv.vin {
      font-size: 0.78rem;
    }
    .u {
      font-size: 0.68rem;
    }
    .tile {
      flex-direction: row;
      align-items: baseline;
      justify-content: space-between;
      gap: 8px;
    }
    .items {
      grid-template-columns: repeat(auto-fit, minmax(128px, 1fr));
    }
    .hl {
      font-size: 0.68rem;
    }
    .hv {
      font-size: 1.3rem;
    }
    .hero .big {
      gap: 0;
    }
  }
</style>
