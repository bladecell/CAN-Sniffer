<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { dtcRows, dtcSummary } from "$lib/diagnosticsView";
  import { DTC_GAP_PX, DTC_MORE_PX, DTC_ROW_PX, dtcVisible } from "$lib/dashboardCardLayout";
  import type { Level, ReadState } from "$lib/dashboardCardView";
  import { loadWhileMounted } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  // The store cannot tell zero codes from a read that has not happened yet, so the card tracks its own first read.
  let read = $state(false);
  loadWhileMounted(async () => {
    await vehicleStore.getDTC();
    read = true;
  });

  const cardState = $derived<ReadState>(read ? "value" : "nodata");
  const summary = $derived(read ? dtcSummary(vehicleStore.dtc) : null);
  const level = $derived<Level | null>(summary === null || summary.tone === "ok" ? null : summary.tone);
  const count = $derived(read ? vehicleStore.totalDTCs : null);
  let listH = $state(0);
  let list = $state<HTMLElement>();
  // Row pitch (height plus gap) read from the first rendered row, so a font or padding change cannot drift from a constant.
  let rowPx = $state(DTC_ROW_PX);
  let rowGap = $state(DTC_GAP_PX);
  let moreH = $state(DTC_MORE_PX);
  const rows = $derived(read ? dtcRows(vehicleStore.dtc) : []);
  const visible = $derived(dtcVisible(rows.length, listH, rowPx, moreH, rowGap));
  const shown = $derived(rows.slice(0, visible.shown));
  // Row heights change when the descriptions arrive, so every change of the rows re-measures. The more line is only in the
  // DOM when some codes are hidden, so its height is kept from the last time it was measured.
  $effect(() => {
    void shown.length;
    void visible.more;
    void rows;
    if (!list) return;
    const [r0, r1] = list.querySelectorAll<HTMLElement>("li:not(.more)");
    if (r0) {
      const box = r0.getBoundingClientRect();
      const cs = getComputedStyle(r0);
      // Space after a row: the list gap plus the item's own margin (the base stylesheet gives list items one).
      const after = r1 ? r1.getBoundingClientRect().top - box.bottom : (parseFloat(getComputedStyle(list).rowGap) || 0) + (parseFloat(cs.marginBottom) || 0);
      rowGap = after;
      rowPx = box.height + after;
    }
    const more = list.querySelector<HTMLElement>("li.more")?.getBoundingClientRect().height;
    if (more) moreH = more;
  });
  const breakdown = $derived([
    { key: "confirmed", label: "Confirmed", n: vehicleStore.dtc.confirmed.dtc_count },
    { key: "pending", label: "Pending", n: vehicleStore.dtc.pending.dtc_count },
    { key: "permanent", label: "Permanent", n: vehicleStore.dtc.permanent.dtc_count },
  ]);
  const warnWord = "Warning";
</script>

<div class="d" class:count={card.style === "count"} class:dim={cardState !== "value"}>
  <CardTitle name="DTCs" state={cardState} {level} direction="high" {warnWord} iconOnly={card.w < 3} />

  {#if card.style === "count"}
    {#if !read}
      <span class="none">No data</span>
    {:else}
      <div class="cnt">
        <div class="big">
          <span class="n" class:warn={level === "warn"} class:crit={level === "crit"}>{count}</span>
          <span class="cap">{count === 1 ? "fault code" : "fault codes"}</span>
        </div>
        <ul class="brk">
          {#each breakdown as b (b.key)}
            <li class={b.key}><span class="bn">{b.n}</span><span class="bl">{b.label}</span></li>
          {/each}
        </ul>
      </div>
    {/if}
  {:else if !read}
    <span class="none">No data</span>
  {:else if rows.length === 0}
    <span class="none ok">No fault codes stored</span>
  {:else}
    <ul class="rows" bind:clientHeight={listH} bind:this={list}>
      {#each shown as r (r.key)}
        <li class={r.group}>
          <span class="st">{r.groupLabel}</span>
          <span class="code">{r.code}</span>
          <span class="desc">{r.description}</span>
        </li>
      {/each}
      {#if visible.more > 0}<li class="more">+{visible.more} more</li>{/if}
    </ul>
  {/if}
</div>

<style>
  .d {
    display: flex;
    flex-direction: column;
    gap: 4px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .cnt {
    display: flex;
    align-items: center;
    justify-content: space-between;
    flex-wrap: wrap;
    gap: 0 12px;
    flex: 1;
    min-height: 0;
  }
  .big {
    display: flex;
    align-items: baseline;
    gap: 6px;
    min-width: 0;
  }
  .n {
    font-size: 1.8rem;
    font-weight: 800;
    line-height: 1;
    font-variant-numeric: tabular-nums;
  }
  .n.warn {
    color: var(--warn);
  }
  .n.crit {
    color: var(--crit);
  }
  .cap,
  .none,
  .bl {
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 600;
  }
  .none.ok {
    color: var(--ok);
    font-weight: 700;
  }
  .brk {
    display: flex;
    flex-wrap: wrap;
    gap: 2px 12px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .brk li {
    display: flex;
    align-items: baseline;
    gap: 6px;
  }
  .cap {
    white-space: nowrap;
  }
  .brk .bl {
    white-space: nowrap;
  }
  .bn {
    font-size: 0.95rem;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
  }
  .brk .confirmed .bn {
    color: var(--crit);
  }
  .brk .pending .bn {
    color: var(--warn);
  }
  .rows {
    display: grid;
    align-content: start;
    gap: 3px;
    flex: 1;
    min-height: 0;
    margin: 0;
    padding: 0;
    overflow: hidden;
    list-style: none;
  }
  .rows li {
    display: grid;
    grid-template-columns: auto auto 1fr;
    align-items: baseline;
    gap: 2px 8px;
    min-width: 0;
    padding-left: 8px;
    border-left: 3px solid var(--muted);
    font-size: 0.82rem;
  }
  .rows li.confirmed {
    border-left-color: var(--crit);
  }
  .rows li.confirmed .st {
    color: var(--crit);
  }
  .rows li.pending {
    border-left-color: var(--warn);
  }
  .rows li.pending .st {
    color: var(--warn);
  }
  .st {
    font-size: 0.72rem;
    font-weight: 800;
    white-space: nowrap;
  }
  .code {
    font-weight: 800;
    font-family: var(--mono, ui-monospace, monospace);
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .desc {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .more {
    display: block !important;
    color: var(--muted);
    font-weight: 700;
    border-left: 0 !important;
    padding-left: 0 !important;
  }
</style>
