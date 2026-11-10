<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { formatReading, readState, type ReadState } from "$lib/dashboardCardView";
  import { faultChip, overviewPlan, type OverviewKind } from "$lib/dashboardCardLayout";
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
    kind: OverviewKind;
    label: string;
    value: string;
    unit: string;
    state: ReadState;
  }

  // Tiles in the order the items were added. A label set on the item replaces the PID name.
  const tiles = $derived.by<Tile[]>(() =>
    items.map((item, i): Tile => {
      if (item.t === "vin") {
        const vin = vehicleStore.vin;
        return { key: `vin-${i}`, kind: "vin", label: item.label || "VIN", value: vin || "—", unit: "", state: vin ? "value" : "nodata" };
      }
      if (item.t === "dtc") {
        return {
          key: `dtc-${i}`,
          kind: "dtc",
          label: item.label || "DTCs",
          value: dtcRead ? String(vehicleStore.totalDTCs) : "—",
          unit: "",
          state: dtcRead ? "value" : "nodata",
        };
      }
      const def = vehicleStore.pidDefinitions.find((d) => d.pid === item.pid);
      const entry = connectionStore.pids.get(item.pid);
      const state = readState(entry, polling);
      const value = state === "value" && entry ? entry.value : null;
      return {
        key: `pid-${item.pid}-${i}`,
        kind: "pid",
        label: item.label || def?.name || `PID ${item.pid}`,
        value: formatReading(value),
        unit: value !== null ? (def?.unit ?? "") : "",
        state,
      };
    }),
  );

  const plan = $derived(overviewPlan(card.style, card.w, card.h, tiles.map((t) => t.kind)));
  // Rows the grid really uses: the rows stretch to fill the card instead of leaving the bottom empty.
  const gridRows = $derived.by(() => {
    let used = 0;
    for (const i of plan.shown) used += tiles[i].kind === "vin" ? Math.min(2, plan.cols) : 1;
    return Math.max(1, Math.ceil(used / plan.cols));
  });
  const shownValue = (t: Tile) => (t.state === "off" ? "Polling off" : t.value);
  // Mockup: the headline is 1.25rem on a narrow card or beside a VIN, 2rem when roomy, 1.6rem otherwise.
  const headSize = $derived(card.w < 4 || plan.hero?.vin != null ? "m" : card.h >= 3 || plan.hero?.wide ? "xl" : "md");
</script>

<div class="o">
  <div class="hd">
    <div class="t1"><CardTitle name="Overview" iconOnly={card.w < 3} inline /></div>
    <span class="chip {chip.tone}">{chip.text}</span>
  </div>

  {#if tiles.length === 0}
    <p class="db-note">No items yet</p>
  {:else if plan.hero}
    {@const hv = tiles[plan.hero.head]}
    <div class="hero" class:wide={plan.hero.wide}>
      <div class="hv" class:dim={hv.state !== "value"}>
        <small>{hv.label}</small>
        <span class="dv {headSize}">{shownValue(hv)}{#if hv.unit}<span class="u"> {hv.unit}</span>{/if}</span>
      </div>
      {#if plan.hero.subs.length}
        <div class="hs" style:grid-template-columns="repeat({plan.cols}, minmax(0, 1fr))">
          {#each plan.hero.subs as i (tiles[i].key)}
            {@const t = tiles[i]}
            <div class:dim={t.state !== "value"}>
              <small>{t.label}</small>
              <b>{shownValue(t)}{#if t.unit}<span class="u"> {t.unit}</span>{/if}</b>
            </div>
          {/each}
        </div>
      {/if}
    </div>
    {#if plan.hero.vin !== null}
      {@const v = tiles[plan.hero.vin]}
      <div class="hvin" class:dim={v.state !== "value"}><small>{v.label}</small><b class="vin">{v.value}</b></div>
    {/if}
  {:else if plan.style === "list"}
    <ul class="ovl" style:grid-template-columns="repeat({plan.cols}, minmax(0, 1fr))">
      {#each plan.shown as i (tiles[i].key)}
        {@const t = tiles[i]}
        <li class:full={t.kind === "vin"} class:dim={t.state !== "value"}>
          <small>{t.label}</small>
          <b class:vin={t.kind === "vin"}>{shownValue(t)}{#if t.unit}<span class="u"> {t.unit}</span>{/if}</b>
        </li>
      {/each}
    </ul>
  {:else}
    <div
      class="ovg"
      style:grid-template-columns="repeat({plan.cols}, minmax(0, 1fr))"
      style:grid-template-rows="repeat({gridRows}, minmax(34px, 1fr))"
      style:--vf={Math.min(2, plan.cols) / plan.cols}
    >
      {#each plan.shown as i (tiles[i].key)}
        {@const t = tiles[i]}
        <div class:dim={t.state !== "value"} style:grid-column={t.kind === "vin" ? `span ${Math.min(2, plan.cols)}` : undefined}>
          <small>{t.label}</small>
          <b class:vin={t.kind === "vin"} class:dtc={t.kind === "dtc"}>{shownValue(t)}{#if t.unit}<span class="u"> {t.unit}</span>{/if}</b>
        </div>
      {/each}
    </div>
  {/if}
</div>

<style>
  .o {
    display: flex;
    flex-direction: column;
    gap: 6px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
    container-type: inline-size;
  }
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
    padding: 2px 11px;
    border-radius: 99px;
    background: var(--ctl);
    color: var(--ctl-sub);
    font-size: 0.8rem;
    font-weight: 800;
    line-height: 1.5;
    white-space: nowrap;
  }
  .chip.ok {
    background: var(--ok-f);
    color: var(--ok);
  }
  .chip.warn {
    background: var(--warn-f);
    color: var(--warn);
  }
  small {
    overflow: hidden;
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 700;
    line-height: 1.25;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  b {
    min-width: 0;
    font-size: 0.9rem;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    line-height: 1.2;
    white-space: nowrap;
  }
  b.vin {
    font-family: var(--mono);
    font-size: 0.74rem;
    font-weight: 700;
  }
  /* The VIN is 17 monospace characters: size it to fill the width its tile spans. */
  .ovg b.vin,
  .hvin b.vin {
    font-size: clamp(0.74rem, calc((100cqw - 24px) * var(--vf, 1) / 10.8), 1.25rem);
  }
  .ovg b {
    font-size: clamp(1rem, 5cqw, 1.35rem);
  }
  .ovg b.dtc {
    font-size: clamp(1.4rem, 8cqw, 2rem);
  }
  .u {
    color: var(--muted);
    font-size: 0.75rem;
    font-weight: 700;
  }
  .dim b,
  .dim .dv {
    color: var(--muted);
    font-weight: 600;
  }
  /* Grid of values: small label over a bold value. */
  .ovg {
    display: grid;
    flex: 1;
    align-content: stretch;
    gap: 4px 10px;
    min-height: 0;
  }
  .ovg > div {
    display: flex;
    flex-direction: column;
    justify-content: center;
    min-width: 0;
    padding: 2px 0;
  }
  /* List: label left, value right, one row per item. */
  .ovl {
    display: grid;
    flex: 1;
    grid-auto-rows: 26px;
    align-content: start;
    gap: 2px 14px;
    min-height: 0;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .ovl li {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    min-width: 0;
    padding: 0 8px;
    border-radius: 9px;
    background: var(--row);
  }
  .ovl li.full {
    grid-column: 1 / -1;
  }
  .ovl small {
    font-size: 0.76rem;
  }
  .ovl b {
    font-size: 0.92rem;
  }
  /* Hero: one large headline value, the rest as small tiles, the VIN as a full-width row. */
  .hero {
    display: grid;
    flex: 1;
    grid-template-rows: auto minmax(0, 1fr);
    gap: 4px 12px;
    align-items: stretch;
    min-height: 0;
    overflow: hidden;
  }
  .hero.wide {
    grid-template-columns: minmax(0, 0.8fr) minmax(0, 2.2fr);
    grid-template-rows: none;
  }
  .hv {
    display: flex;
    flex-direction: column;
    justify-content: center;
    min-width: 0;
  }
  .hv small {
    font-size: 0.76rem;
  }
  .dv {
    font-size: 1.6rem;
    font-weight: 800;
    line-height: 1.05;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .dv.m {
    font-size: 1.25rem;
  }
  .dv.xl {
    font-size: 2rem;
  }
  .hs {
    display: grid;
    gap: 4px 10px;
    align-content: stretch;
    min-width: 0;
  }
  .hs > div {
    display: flex;
    flex-direction: column;
    justify-content: center;
    min-width: 0;
    padding: 2px 5px;
    border-radius: 9px;
    background: var(--row);
  }
  .hs small {
    font-size: 0.7rem;
  }
  .hs b {
    font-size: 0.8rem;
  }
  .hvin {
    display: flex;
    flex: none;
    flex-direction: column;
    margin-top: 2px;
    padding: 2px 6px;
    border-radius: 9px;
    background: var(--row);
  }
  .hvin small {
    font-size: 0.7rem;
    line-height: 1.2;
  }
</style>
