<script lang="ts">
  import { untrack } from "svelte";
  import type { Card } from "$lib/dashboardModel";
  import { auth } from "$lib/auth.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { uptimeLabel, wifiModeLabel, worstLevel, type Level } from "$lib/dashboardCardView";
  import { statusVisible } from "$lib/dashboardCardLayout";
  import { sdView } from "$lib/settingsModel";
  import { loadWhileMounted } from "./cardSources.svelte";
  import CardTitle from "./CardTitle.svelte";
  import { pollingControl, recorderControl, type Control } from "./controls.svelte";

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

  // The same switches as the Polling and Recorder cards.
  const polling = pollingControl();
  const recorder = recorderControl();

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
    /** A switch in place of the value: the item is a control, not a reading. */
    control?: Control;
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
      value: !systemRead ? "No data" : sys.sd_card_detected ? "Ready" : "No card",
      level: !systemRead ? null : sys.sd_card_detected ? null : "crit",
      muted: !systemRead,
      tone: !systemRead ? undefined : sys.sd_card_detected ? "ok" : "crit",
      detail: sys.sd_card_detected ? sdDetail : undefined,
    });
    list.push({ key: "polling", label: "Polling", value: polling.word, level: null, muted: polling.reason !== null, control: polling, detail: polling.reason ?? "OBD2 requests" });
    list.push({ key: "recorder", label: "Recorder", value: recorder.word, level: null, muted: recorder.reason !== null, control: recorder, detail: recorder.reason ?? "trip to SD card" });
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
      list.push({ key: "sta", label: "Network (configured)", value: locked ? "Log in to view" : wifi ? wifi.sta_ssid || "—" : "No data", level: null, muted: locked || !wifi, detail: "not live" });
    }
    return list;
  });

  // Check, warning triangle and critical circle, so the tone never rests on colour alone.
  const TONE_PATH = {
    ok: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zm-1.2 14.4-4-4 1.4-1.4 2.6 2.6 5.6-5.6 1.4 1.4z",
    warn: "M12 3 22 20H2zm-1 6v5h2V9zm0 7v2h2v-2z",
    crit: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zm-1 5h2v6h-2zm0 8h2v2h-2z",
  } as const;
  const worst = $derived(worstLevel(items.map((i) => i.level)));
  const visible = $derived(statusVisible(items.length, card.w, card.h, card.style));
  const shown = $derived(items.slice(0, visible.shown));
</script>

{#snippet toggle(c: Control, label: string)}
  <button type="button" role="switch" class="sw" aria-checked={c.on} aria-label={label} title={c.reason ?? undefined} disabled={c.reason !== null} onclick={() => void c.toggle()}>
    <span class="trk"><span class="knob"></span></span>
  </button>
{/snippet}

<div class="s">
  <CardTitle name="Status" level={worst} warnWord="Warning" iconOnly={card.w < 4} />

  {#if card.style === "list"}
    <ul class="rows">
      {#each shown as it (it.key)}
        <li class:muted={it.muted} class:ctl={it.control}>
          <span class="ico">
            {#if it.tone}<svg class="ti {it.tone}" viewBox="0 0 24 24" aria-hidden="true"><path d={TONE_PATH[it.tone]} /></svg>{/if}
          </span>
          <span class="lb">{it.label}</span>
          <b class="vl {it.tone ?? ''}">{it.value}</b>
          {#if it.control}{@render toggle(it.control, it.label)}{/if}
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
          {#if it.control}
            <span class="cw">{@render toggle(it.control, it.label)}<span class="cv">{it.value}</span></span>
          {:else}
            <span class="chip {it.tone ?? 'plain'}">
              {#if it.tone}<svg class="ti" viewBox="0 0 24 24" aria-hidden="true"><path d={TONE_PATH[it.tone]} /></svg>{/if}
              <span class="ct">{it.value}</span>
            </span>
          {/if}
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
  .chip.ok,
  .vl.ok {
    color: var(--ok);
  }
  .ti.warn,
  .chip.warn,
  .vl.warn {
    color: var(--warn);
  }
  .ti.crit,
  .chip.crit,
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

  /* Switch: a 48 x 44 hit area around a 40 x 24 track, as the 44 px target rule asks. */
  .sw {
    display: grid;
    flex: none;
    place-items: center;
    width: 48px;
    height: 44px;
    margin: 0;
    padding: 0;
    border: 0;
    background: none;
    cursor: pointer;
  }
  .trk {
    position: relative;
    width: 40px;
    height: 24px;
    border-radius: 99px;
    background: var(--ctl);
  }
  .sw[aria-checked="true"] .trk {
    background: var(--accent);
  }
  .knob {
    position: absolute;
    top: 3px;
    left: 3px;
    width: 18px;
    height: 18px;
    border-radius: 50%;
    background: var(--float-solid);
    transition: transform 0.2s;
  }
  .sw[aria-checked="true"] .knob {
    transform: translateX(16px);
    background: var(--on-accent);
  }
  .sw:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }
  .sw:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
    border-radius: 12px;
  }
  .cw {
    display: flex;
    align-items: center;
    gap: 2px;
    min-width: 0;
    margin-left: -6px;
  }
  .cv {
    font-size: 0.76rem;
    font-weight: 800;
  }
  .chips li.muted .cv {
    color: var(--muted);
    font-weight: 600;
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
    display: inline-flex;
    align-items: center;
    gap: 4px;
    align-self: flex-start;
    max-width: 100%;
    padding: 1px 8px 1px 6px;
    border-radius: 99px;
    background: var(--ctl);
    color: var(--text);
    font-size: 0.76rem;
    font-weight: 800;
    line-height: 1.5;
    white-space: nowrap;
  }
  .chip.ok {
    background: var(--ok-f);
  }
  .chip.warn {
    background: var(--warn-f);
  }
  .chip.crit {
    background: var(--crit-f);
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
  .rows li.ctl {
    min-height: 44px;
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
