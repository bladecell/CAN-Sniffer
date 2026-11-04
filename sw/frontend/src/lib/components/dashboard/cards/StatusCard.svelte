<script lang="ts">
  import { untrack } from "svelte";
  import type { Card } from "$lib/dashboardModel";
  import { auth } from "$lib/auth.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { uptimeLabel, wifiModeLabel, worstLevel, type Level } from "$lib/dashboardCardView";
  import { statusVisible } from "$lib/dashboardCardLayout";
  import { loadWhileMounted } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  // Each read is tracked here, so "No data" means this card has not finished one.
  let systemRead = $state(false);
  loadWhileMounted(async () => {
    await vehicleStore.requestSystem();
    systemRead = true;
  });

  // The Wi-Fi settings are gated: read them once the session is open, never before.
  $effect(() => {
    if (auth.view.loggedIn) untrack(() => void settingsStore.load());
  });

  const LOW_HEAP_BYTES = 32 * 1024;

  interface Item {
    key: string;
    label: string;
    value: string;
    level: Level | null;
    /** Shown muted: no reading yet, or the value needs a login. */
    muted: boolean;
  }

  const canState = $derived(connectionStore.connected ? connectionStore.wsCanStatus.state : null);
  const sys = $derived(vehicleStore.system);
  const wifi = $derived(settingsStore.doc?.wifi ?? null);

  const items = $derived.by<Item[]>(() => {
    const list: Item[] = [];
    list.push({
      key: "can",
      label: "CAN link",
      value: canState === null ? "No data" : canState.replace(/^CAN /, ""),
      level: canState === null ? null : canState === "CAN Connected" ? null : "crit",
      muted: canState === null,
    });
    list.push({
      key: "sd",
      label: "SD card",
      value: !systemRead ? "No data" : sys.sd_card_detected ? "Ready" : "No card",
      level: !systemRead ? null : sys.sd_card_detected ? null : "crit",
      muted: !systemRead,
    });
    list.push({
      key: "heap",
      label: "Free heap",
      value: !systemRead ? "No data" : `${Math.round(sys.heap_free / 1024)} KB`,
      level: !systemRead ? null : sys.heap_free < LOW_HEAP_BYTES ? "warn" : null,
      muted: !systemRead,
    });
    list.push({
      key: "uptime",
      label: "Uptime",
      value: !systemRead ? "No data" : uptimeLabel(sys.uptime_s),
      level: null,
      muted: !systemRead,
    });
    if (card.style === "list" || card.h >= 3) {
      const locked = !auth.view.loggedIn;
      list.push({ key: "mode", label: "Wi-Fi mode", value: locked ? "Log in to view" : wifi ? wifiModeLabel(wifi.mode) : "No data", level: null, muted: locked || !wifi });
      list.push({ key: "ap", label: "AP name", value: locked ? "Log in to view" : wifi ? wifi.ssid || "—" : "No data", level: null, muted: locked || !wifi });
      list.push({ key: "sta", label: "Network (configured)", value: locked ? "Log in to view" : wifi ? wifi.sta_ssid || "—" : "No data", level: null, muted: locked || !wifi });
    }
    return list;
  });

  const worst = $derived(worstLevel(items.map((i) => i.level)));
  const visible = $derived(statusVisible(items.length, card.w, card.h, card.style));
  const shown = $derived(items.slice(0, visible.shown));
</script>

<div class="s" class:list={card.style === "list"} style:--cols={card.w >= 6 ? 3 : 2}>
  <CardTitle name="Status" level={worst} warnWord="Warning" iconOnly={card.w < 4} />

  <ul class="items">
    {#each shown as it (it.key)}
      <li class:muted={it.muted} class:warn={it.level === "warn"} class:crit={it.level === "crit"}>
        <span class="lb">{it.label}</span>
        <span class="vl">{it.value}</span>
      </li>
    {/each}
    {#if visible.more > 0}<li class="more">+{visible.more} more</li>{/if}
  </ul>
</div>

<style>
  .s {
    display: flex;
    flex-direction: column;
    gap: 6px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .items {
    display: grid;
    grid-template-columns: repeat(var(--cols, 2), minmax(0, 1fr));
    flex: 1;
    grid-auto-rows: min-content;
    align-content: start;
    gap: 4px 6px;
    min-height: 0;
    margin: 0;
    padding: 0;
    overflow: hidden;
    list-style: none;
  }
  .items li.more {
    display: block;
    align-self: center;
    padding: 0 4px;
    background: none;
    color: var(--muted);
    font-size: 0.78rem;
    font-weight: 700;
  }
  .items li {
    display: flex;
    flex-direction: column;
    min-width: 0;
    padding: 3px 8px;
    border-radius: 12px;
    background: var(--ctl);
  }
  .list .items {
    grid-template-columns: 1fr;
    gap: 0;
  }
  .list .items li {
    flex-direction: row;
    align-items: baseline;
    justify-content: space-between;
    gap: 8px;
    min-height: 32px;
    background: none;
    border-bottom: 1px solid var(--hair);
    border-radius: 0;
  }
  .lb {
    overflow: hidden;
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 700;
    line-height: 1.2;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .vl {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-size: 0.85rem;
    line-height: 1.2;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
  }
  .items li.muted .vl {
    color: var(--muted);
    font-weight: 600;
  }
  .items li.warn .vl {
    color: var(--warn);
  }
  .items li.crit .vl {
    color: var(--crit);
  }
</style>
