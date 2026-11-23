<script lang="ts">
  import { untrack } from "svelte";
  import type { Card, StatusItemKey } from "$lib/dashboardModel";
  import { auth } from "$lib/auth.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import { sdCardStore } from "$lib/sdCardStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { uptimeLabel, wifiModeLabel, worstLevel, type Level } from "$lib/dashboardCardView";
  import { statusVisible } from "$lib/dashboardCardLayout";
  import { sdView } from "$lib/settingsModel";
  import { loadWhileMounted } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";

  let { card }: { card: Card } = $props();

  // Each read is tracked here, so "No data" means this card has not finished one.
  let systemRead = $state(false);
  loadWhileMounted(async () => {
    await vehicleStore.requestSystem();
    systemRead = true;
  });
  // The bus bitrate is the one detail that comes from GET /can_bus, not from the WebSocket frame.
  loadWhileMounted(() => vehicleStore.getCanStatus());

  // Card space comes from GET /sd_card/info. A busy card (503) or a failed read leaves the detail out.
  loadWhileMounted(() => vehicleStore.requestSDInfo());
  const sdDetail = $derived.by(() => {
    const v = vehicleStore.sdInfo ? sdView(vehicleStore.sdInfo) : null;
    return v?.kind === "ok" ? `${v.usedGb} / ${v.totalGb} GB` : undefined;
  });

  // The Wi-Fi settings are gated: read them once the session is open, never before.
  $effect(() => {
    if (auth.canUseGated) untrack(() => void settingsStore.load());
  });

  const LOW_HEAP_BYTES = 32 * 1024;

  interface Item {
    key: string;
    label: string;
    value: string;
    level: Level | null;
    /** Shown muted: no reading yet, or the value needs a login. */
    muted: boolean;
    /** The state of a checked item: healthy (check), warning or critical. Items that only inform have none. */
    tone?: "ok" | Level;
    /** Mockup detail text, shown at 5 columns (list) or 4 rows (chips). */
    detail?: string;
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
      tone: canState === null ? undefined : canState === "CAN Connected" ? "ok" : "crit",
      detail: vehicleStore.canStatus && vehicleStore.canStatus.bitrate > 0 ? `${Math.round(vehicleStore.canStatus.bitrate / 1000)} kbit/s` : undefined,
    });
    list.push({
      key: "sd",
      label: "SD card",
      value: !systemRead ? "No data" : sdCardStore.unavailable ? "Unavailable" : sys.sd_card_detected ? "Ready" : "No card",
      level: !systemRead ? null : sdCardStore.unavailable || !sys.sd_card_detected ? "crit" : null,
      muted: !systemRead,
      tone: !systemRead ? undefined : sdCardStore.unavailable || !sys.sd_card_detected ? "crit" : "ok",
      detail: sys.sd_card_detected && !sdCardStore.unavailable ? sdDetail : undefined,
    });
    list.push({
      key: "heap",
      label: "Free heap",
      value: !systemRead ? "No data" : `${Math.round(sys.heap_free / 1024)} KB`,
      level: !systemRead ? null : sys.heap_free < LOW_HEAP_BYTES ? "warn" : null,
      muted: !systemRead,
      tone: !systemRead ? undefined : sys.heap_free < LOW_HEAP_BYTES ? "warn" : undefined,
      detail: systemRead ? `lowest ${Math.round(sys.heap_min_free / 1024)} KB` : undefined,
    });
    list.push({
      key: "uptime",
      label: "Uptime",
      value: !systemRead ? "No data" : uptimeLabel(sys.uptime_s),
      level: null,
      muted: !systemRead,
      detail: "since boot",
    });
    if (card.style === "list" || card.h >= 3) {
      const locked = !auth.canUseGated;
      list.push({ key: "mode", label: "Wi-Fi mode", value: locked ? "Log in to view" : wifi ? wifiModeLabel(wifi.mode) : "No data", level: null, muted: locked || !wifi, detail: "from Settings" });
      list.push({ key: "ap", label: "AP name", value: locked ? "Log in to view" : wifi ? wifi.ssid || "—" : "No data", level: null, muted: locked || !wifi, detail: "access point" });
      list.push({ key: "sta", label: "Network", value: locked ? "Log in to view" : wifi ? wifi.sta_ssid || "—" : "No data", level: null, muted: locked || !wifi, detail: "not live" });
    }
    return list;
  });

  // Check, warning triangle and critical circle, so the tone never rests on colour alone.
  const TONE_PATH = {
    ok: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zm-1.2 14.4-4-4 1.4-1.4 2.6 2.6 5.6-5.6 1.4 1.4z",
    warn: "M12 3 22 20H2zm-1 6v5h2V9zm0 7v2h2v-2z",
    crit: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zm-1 5h2v6h-2zm0 8h2v2h-2z",
  } as const;
  // Rows the card's settings switched off are left out before the fit, so the rest fill the space.
  const offered = $derived(items.filter((i) => !card.hidden?.includes(i.key as StatusItemKey)));
  const worst = $derived(worstLevel(offered.map((i) => i.level)));
  const visible = $derived(statusVisible(offered.length, card.w, card.h, card.style));
  const shown = $derived(offered.slice(0, visible.shown));
</script>

<div class="s">
  <CardTitle name="Status" level={worst} warnWord="Warning" iconOnly={card.w < 4} />

  {#if card.style === "list"}
    <ul class="rows">
      {#each shown as it (it.key)}
        <li class:muted={it.muted}>
          <span class="ico">
            {#if it.tone}<svg class="ti {it.tone}" viewBox="0 0 24 24" aria-hidden="true"><path d={TONE_PATH[it.tone]} /></svg>{/if}
          </span>
          <span class="lb">{it.label}</span>
          <b class="vl {it.tone ?? ''}">{it.value}</b>
          {#if card.w >= 5 && it.detail}<span class="sd">{it.detail}</span>{/if}
        </li>
      {/each}
      {#if visible.more > 0}<li class="more">+{visible.more} more</li>{/if}
    </ul>
  {:else}
    <ul class="chips" style:--cols={card.w >= 6 ? 3 : 2}>
      {#each shown as it (it.key)}
        <li class:muted={it.muted}>
          <span class="lb">{it.label}</span>
          <span class="chip c-{it.tone ?? 'plain'}">
            {#if it.tone}<svg class="ti" viewBox="0 0 24 24" aria-hidden="true"><path d={TONE_PATH[it.tone]} /></svg>{/if}
            <span class="ct">{it.value}</span>
          </span>
          {#if card.h >= 4 && it.detail}<em>{it.detail}</em>{/if}
        </li>
      {/each}
      {#if visible.more > 0}<li class="more">+{visible.more} more</li>{/if}
    </ul>
  {/if}
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
  .more {
    align-self: center;
    color: var(--muted);
    font-size: 0.78rem;
    font-weight: 700;
  }
  .ti {
    display: inline-block;
    flex: none;
    width: 1em;
    height: 1em;
    fill: currentColor;
  }
  .ti.ok,
  .vl.ok {
    color: var(--ok);
  }
  .ti.warn,
  .vl.warn {
    color: var(--warn);
  }
  .ti.crit,
  .vl.crit {
    color: var(--crit);
  }
  .lb {
    overflow: hidden;
    color: var(--muted);
    font-size: 0.74rem;
    font-weight: 700;
    line-height: 1.2;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  /* Chip grid: a label, then a pill tinted by the level (the mockup chip, not the whole cell). */
  .chips {
    display: grid;
    grid-template-columns: repeat(var(--cols, 2), minmax(0, 1fr));
    flex: 1;
    grid-auto-rows: min-content;
    align-content: start;
    gap: 6px 10px;
    min-height: 0;
    margin: 0;
    padding: 0;
    overflow: hidden;
    list-style: none;
  }
  .chips li {
    display: flex;
    flex-direction: column;
    gap: 2px;
    min-width: 0;
  }
  .chips li.more {
    align-self: center;
    grid-column: 1 / -1;
  }
  .chip {
    align-self: flex-start;
    max-width: 100%;
  }
  .chip .ct {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .chips em {
    overflow: hidden;
    color: var(--muted);
    font-size: 0.74rem;
    font-style: normal;
    font-weight: 600;
    line-height: 1.2;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .chips li.muted .chip {
    color: var(--muted);
    font-weight: 600;
  }

  /* List: icon, label, value and, from 5 columns, the detail. Dividers between rows, as the mockup. */
  .rows {
    display: grid;
    flex: 1;
    align-content: start;
    min-height: 0;
    margin: 0;
    padding: 0;
    overflow: hidden;
    list-style: none;
  }
  .rows li {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
    min-height: 32px;
    font-size: 0.88rem;
    border-top: 1px solid var(--hair);
  }
  .rows li:first-child {
    border-top: 0;
  }
  .rows li.more {
    display: block;
    min-height: 0;
    padding-left: 2px;
    border-top: 0;
  }
  .rows .ico {
    display: grid;
    flex: none;
    place-items: center;
    width: 14px;
    height: 14px;
  }
  .rows .ti {
    width: 1.15em;
    height: 1.15em;
  }
  .rows .lb {
    flex: 1;
    font-size: 0.88rem;
  }
  .rows .vl {
    min-width: 0;
    max-width: 46%;
    overflow: hidden;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .rows li.muted .vl {
    color: var(--muted);
    font-weight: 600;
  }
  .rows .sd {
    flex: none;
    width: 104px;
    overflow: hidden;
    color: var(--muted);
    font-size: 0.78rem;
    font-weight: 600;
    text-align: right;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
</style>
