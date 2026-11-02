<script lang="ts">
  import { onMount } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { auth } from "$lib/auth.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { ACCENTS } from "$lib/appearance";
  import { appearance } from "$lib/appearance.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { GROUP, GROUP_IDS, RAIL, bitrateLabel, groupShown, type GroupId } from "$lib/settingsModel";
  import AppearanceGroup from "$lib/components/settings/AppearanceGroup.svelte";
  import WifiGroup from "$lib/components/settings/WifiGroup.svelte";
  import ModemGroup from "$lib/components/settings/ModemGroup.svelte";
  import VehicleGroup from "$lib/components/settings/VehicleGroup.svelte";
  import RecorderGroup from "$lib/components/settings/RecorderGroup.svelte";
  import StorageGroup from "$lib/components/settings/StorageGroup.svelte";
  import AccessGroup from "$lib/components/settings/AccessGroup.svelte";
  import DeviceGroup from "$lib/components/settings/DeviceGroup.svelte";
  import "$lib/components/settings/settings.css";

  const THEME_LABEL = { system: "System", light: "Light", dark: "Dark" } as const;
  const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

  let query = $state("");
  const q = $derived(query.trim());
  let wide = $state(true);
  let picked = $state<GroupId>("look");
  /** Phone only: the group page that is open, or null for the grouped list. */
  let openId = $state<GroupId | null>(null);

  const canUse = $derived(auth.view.loggedIn || auth.view.devBypass);
  const doc = $derived(settingsStore.doc);
  const visible = $derived(GROUP_IDS.filter((id) => groupShown(q, GROUP[id])));
  /** Wide: the picked group, or the first match when the picked one is filtered out. */
  const activeId = $derived<GroupId | undefined>(visible.includes(picked) ? picked : visible[0]);

  onMount(() => void auth.refresh());

  $effect(() => {
    const mq = matchMedia("(min-width: 860px)");
    const update = () => (wide = mq.matches);
    update();
    mq.addEventListener("change", update);
    return () => mq.removeEventListener("change", update);
  });

  // Settings are gated: read them once a session exists. A failed read waits for "Try again", not a loop.
  $effect(() => {
    if (canUse && doc === null && !settingsStore.loading && settingsStore.loadError === null) void settingsStore.load();
  });

  const accentLabel = $derived(ACCENTS.find((a) => a.id === appearance.accent)?.label ?? "Orange");

  function reading(): string {
    return canUse ? "Reading settings" : "Log in to change";
  }

  function summary(id: GroupId): string {
    switch (id) {
      case "look":
        return `${THEME_LABEL[appearance.theme]}, ${accentLabel}`;
      case "wifi":
        return doc ? `Hotspot ${doc.wifi.ssid}` : reading();
      case "modem":
        return "Needs the HW v2 board";
      case "vehicle":
        return doc ? `${bitrateLabel(doc.can.bitrate)} bus` : reading();
      case "recorder":
        return vehicleStore.obd2Status?.auto_poll ? "Auto poll on" : "Auto poll off";
      case "storage":
        return "Card and PID folder";
      case "access":
        return auth.view.loggedIn ? "Logged in" : "Log in to change";
      case "device":
        return vehicleStore.system.app_version ? `Firmware ${vehicleStore.system.app_version}` : "Firmware";
    }
  }

  /** Phone: the Cellular & GNSS row is disabled (no modem API yet), so it does not open. */
  function openOnPhone(id: GroupId) {
    if (id !== "modem") openId = id;
  }
</script>

{#snippet search()}
  <div class="st-search">
    <Icon name="magnifying-glass" size={18} />
    <input
      class="st-in"
      type="search"
      placeholder="Search settings"
      aria-label="Search settings"
      bind:value={query}
      onkeydown={(e) => {
        if (e.key === "Escape") query = "";
      }}
    />
  </div>
{/snippet}

{#snippet chips()}
  <div class="st-chips">
    {#if connectionStore.connected}
      <span class="st-chip c-ok"><Icon name="circle-check" size={14} />Connected</span>
    {:else}
      <span class="st-chip"><Icon name="circle" size={14} />Not connected</span>
    {/if}
    {#if vehicleStore.system.app_version}
      <span class="st-chip">Firmware {vehicleStore.system.app_version}</span>
    {/if}
    {#if settingsStore.restartRequired}
      <span class="st-chip c-warn"><Icon name="triangle-exclamation" size={14} />Restart required</span>
    {/if}
  </div>
{/snippet}

{#snippet body(id: GroupId)}
  {#if id === "look"}
    <AppearanceGroup q={q} />
  {:else if id === "modem"}
    <ModemGroup q={q} />
  {:else if id === "recorder"}
    <RecorderGroup q={q} />
  {:else if id === "access"}
    <AccessGroup q={q} />
  {:else if id === "device"}
    <DeviceGroup q={q} />
  {:else if id === "storage"}
    <StorageGroup q={q} doc={doc} />
  {:else if !canUse}
    <div class="st-none">
      <p>Log in to read and change these settings.</p>
      <button type="button" class="st-btn pri" onclick={() => auth.ensureLogin()}>Log in</button>
    </div>
  {:else if doc === null}
    {#if settingsStore.loadError}
      <div class="st-none">
        <p role="alert">Settings could not be read: {settingsStore.loadError}</p>
        <button type="button" class="st-btn" onclick={() => settingsStore.load()}>Try again</button>
      </div>
    {:else}
      <p class="st-none">{reading()}</p>
    {/if}
  {:else if id === "wifi"}
    <WifiGroup q={q} doc={doc} />
  {:else}
    <VehicleGroup q={q} doc={doc} />
  {/if}
{/snippet}

<div class="st-root">
  {#if wide}
    <section class="st-card st-head">
      <div>
        <h1>Settings</h1>
        {@render chips()}
      </div>
      {@render search()}
    </section>

    {#if visible.length === 0}
      <section class="st-card">
        <p class="st-none" role="status">No setting matches that search.</p>
      </section>
    {:else}
    <div class="st-split">
      <div class="st-rail" role="tablist" aria-orientation="vertical" aria-label="Settings groups">
        {#each RAIL as section (section.label)}
          {@const ids = section.ids.filter((id) => visible.includes(id))}
          {#if ids.length > 0}
            <div class="st-rail-label">{section.label}</div>
            {#each ids as id (id)}
              <button type="button" class="st-tab" role="tab" aria-selected={id === activeId} onclick={() => (picked = id)}>
                <Icon name={GROUP[id].icon} size={24} />{GROUP[id].title}
              </button>
            {/each}
          {/if}
        {/each}
      </div>

      <section class="st-card st-pane">
        {#if activeId}
          <div class="st-pane-head">
            <span class="st-pane-ico"><Icon name={GROUP[activeId].icon} size={26} /></span>
            <div>
              <h2>{GROUP[activeId].title}</h2>
              <p>{summary(activeId)}</p>
            </div>
          </div>
          {@render body(activeId)}
        {/if}
      </section>
    </div>
    {/if}
  {:else if openId === null}
    <div class="st-phone">
      <h1>Settings</h1>
      {@render search()}
      <div class="st-phone-list">
        {#each RAIL as section (section.label)}
          {@const ids = section.ids.filter((id) => visible.includes(id))}
          {#if ids.length > 0}
            <div class="st-sec">
              <h2>{section.label}</h2>
              <div class="st-items">
                {#each ids as id (id)}
                  <button
                    type="button"
                    class="st-item"
                    aria-disabled={id === "modem" ? "true" : undefined}
                    onclick={() => openOnPhone(id)}
                  >
                    <span class="st-item-ico"><Icon name={GROUP[id].icon} size={24} /></span>
                    <span class="st-item-t">{GROUP[id].title}</span>
                    <Icon name="chevron-down" size={18} />
                    <span class="st-item-s">{summary(id)}</span>
                  </button>
                {/each}
              </div>
            </div>
          {/if}
        {/each}
      </div>
      {#if visible.length === 0}
        <p class="st-none" role="status">No setting matches that search.</p>
      {/if}
    </div>
  {:else}
    <div class="st-phone st-slide">
      <div class="st-drill-head st-float">
        <button type="button" class="st-btn icon" aria-label="Back to Settings" onclick={() => (openId = null)}>
          <Icon name="chevron-down" size={22} />
        </button>
        <h2>{GROUP[openId].title}</h2>
      </div>
      <section class="st-card st-phone-card">
        {@render body(openId)}
      </section>
    </div>
  {/if}
</div>

<style>
  .st-search {
    position: relative;
    min-width: 0;
    align-self: center;
  }
  .st-search :global(svg) {
    display: block;
    position: absolute;
    left: 14px;
    top: 50%;
    transform: translateY(-50%);
    color: var(--ctl-sub);
    pointer-events: none;
  }
  .st-search .st-in {
    width: 100%;
    padding-left: 44px;
  }
  .st-phone-card {
    display: grid;
    gap: 22px;
    align-content: start;
  }
  @media (max-width: 859px) {
    .st-chips {
      display: none;
    }
    .st-phone-card {
      border-radius: 24px;
      padding: 20px;
    }
    .st-search :global(svg) {
      left: 12px;
    }
  }
</style>
