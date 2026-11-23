<script lang="ts">
  import { untrack } from "svelte";
  import { MediaQuery } from "svelte/reactivity";
  import Icon from "$lib/Icon.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { canBus, type IdDetail } from "$lib/canBus/canBusStore.svelte";
  import { findRuleForId } from "$lib/canBus/canRuleForm";
  import { rulesStore } from "$lib/canBus/canRulesStore.svelte";
  import CanTiles from "$lib/components/canBus/CanTiles.svelte";
  import CanIdList from "$lib/components/canBus/CanIdList.svelte";
  import CanFrameTable from "$lib/components/canBus/CanFrameTable.svelte";
  import CanIdDetails from "$lib/components/canBus/CanIdDetails.svelte";
  import CanEcuIds from "$lib/components/canBus/CanEcuIds.svelte";
  import CanRules from "$lib/components/canBus/CanRules.svelte";
  import CanSend from "$lib/components/canBus/CanSend.svelte";
  import CanCaptures from "$lib/components/canBus/CanCaptures.svelte";
  import { captures } from "$lib/canBus/captureStore.svelte";

  type Tab = "ov" | "ids" | "ecu" | "cap" | "info" | "send" | "rules";
  const phone = new MediaQuery("max-width: 859px");
  // Phone only: "ov" is the Overview tab (header + status tiles). Desktop always shows those beside the panel.
  let tab = $state<Tab>(phone.current ? "ov" : "ids");
  let sideOpen = $state(false);
  const sideShown = $derived(phone.current || sideOpen);
  const showOverview = $derived(!phone.current || tab === "ov");

  const RAIL: { id: Tab; label: string; icon: string }[] = $derived([
    ...(phone.current ? [{ id: "ov" as const, label: "Overview", icon: "grid" }] : []),
    { id: "ids", label: "IDs", icon: "hashtag" },
    { id: "ecu", label: "ECU IDs", icon: "microchip" },
    { id: "rules", label: "Rules", icon: "filter" },
    { id: "info", label: "ID details", icon: "circle-info" },
    { id: "send", label: "Send", icon: "paper-plane" },
    { id: "cap", label: "Captures", icon: "sd-card" },
  ]);

  // Desktop: a second click on the selected tab hides the panel; the rail stays.
  function pick(id: Tab) {
    if (!phone.current && tab === id) sideOpen = !sideOpen;
    else {
      tab = id;
      sideOpen = true;
    }
  }

  function openDetails(key: number) {
    canBus.select(key);
    tab = "info";
    sideOpen = true;
  }

  function openRuleFor(detail: IdDetail) {
    canBus.requestRule(detail);
    tab = "rules";
    sideOpen = true;
  }

  const selectedRuleExists = $derived(canBus.snapshot.selected ? findRuleForId(rulesStore.list, canBus.snapshot.selected.id, canBus.snapshot.selected.ext) !== undefined : false);

  // A click on empty space (not a row, the side panel or a control) clears the selection.
  function clearSelection(e: MouseEvent) {
    if (canBus.selectedKey !== null && !(e.target as Element).closest("tr, .cl-row, .cb-side, button, input, select, a, label")) canBus.select(null);
  }

  // The page attaches on mount. The stream stops on leave (0xB1) unless "Log in background" is on.
  $effect(() => untrack(() => canBus.attach()));

  // Bus status is read on every open and whenever the CAN state changes (GET /can_bus, GET /can/mode).
  // wsCanStatus is replaced on every 1 Hz status frame, so the $derived keeps the effect from re-running unless the state itself changes.
  const busTrigger = $derived(connectionStore.connected && connectionStore.wsCanStatus.state);
  $effect(() => {
    if (busTrigger) untrack(() => void canBus.refreshBus());
  });

  const live = $derived(connectionStore.connected && canBus.counters.streaming);
  const statusText = $derived(
    !connectionStore.connected
      ? connectionStore.sleeping
        ? "Sleeping"
        : "Not connected"
      : !canBus.logging
        ? "Not logging"
        : !canBus.counters.streaming
          ? "Waiting for frames"
          : "Live",
  );
  const bitrateText = $derived(canBus.bus ? `${canBus.bus.bitrate / 1000} kbit/s` : "Bitrate unknown");
  const modeText = $derived(canBus.mode ? (canBus.mode.mode === "raw" ? "Raw" : "Monitor") : "Mode unknown");

  // The capture status is an open read; watched while the page is open.
  $effect(() => untrack(() => captures.watch()));

  async function toggleSave() {
    const wasCapturing = captures.capturing;
    const err = await captures.toggle();
    if (err) alertStore.add(err, "error");
    else if (!wasCapturing) alertStore.add("Saving the raw stream to the SD card", "success");
  }
</script>

<!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_static_element_interactions -->
<section class="cb" onclick={clearSelection}>
  <div class="cb-ws" class:collapsed={!sideShown}>
    <div class="cb-left">
    <nav class="card cb-rail icon-rail" aria-label="CAN views">
      {#each RAIL as r (r.id)}
        <button aria-current={tab === r.id && sideShown ? "page" : undefined} aria-label={r.label} title={r.label} onclick={() => pick(r.id)}>
          <Icon name={r.icon} size={20} />
        </button>
      {/each}
    </nav>

    {#if tab !== "ov" || !phone.current}
    <aside class="card cb-side" class:closed={!sideShown} inert={!sideShown} aria-label="CAN panel">
      {#if tab === "ids" || tab === "ov"}
        <CanIdList
          ids={canBus.snapshot.ids}
          now={canBus.counters.now}
          selected={canBus.selectedKey}
          onselect={(key) => canBus.select(key)}
          ondetails={openDetails}
        />
      {:else if tab === "ecu"}
        <CanEcuIds ecus={canBus.snapshot.ecus} now={canBus.counters.now} />
      {:else if tab === "cap"}
        <CanCaptures />
      {:else if tab === "send"}
        <CanSend />
      {:else if tab === "rules"}
        <CanRules />
      {:else if tab === "info"}
        <CanIdDetails detail={canBus.snapshot.selected} startUs={canBus.snapshot.startUs} ruleExists={selectedRuleExists} onrule={openRuleFor} />
      {/if}
    </aside>
    {/if}
    </div>

    <div class="cb-main">
      <header class="card cb-head">
        <div class="cb-title">
          <h1>CAN Bus</h1>
          <p class="cb-sub">
            <span class="cb-dot" class:on={live} aria-hidden="true"></span>
            <span>{statusText}</span>
            <a href="#settings">{bitrateText}</a>
            <a href="#settings">{modeText}</a>
          </p>
        </div>
        <div class="cb-actions">
          <button class="btn" aria-pressed={canBus.logging} onclick={() => canBus.setLogging(!canBus.logging)}>
            <Icon name={canBus.logging ? "pause" : "play"} size={16} />
            <span>{canBus.logging ? "Pause" : "Start"}</span>
          </button>
          <button class="btn pri" aria-pressed={captures.capturing} disabled={captures.busy} onclick={toggleSave}>
            <Icon name={captures.capturing ? "square" : "floppy-disks"} size={16} />
            <span>{captures.capturing ? "Stop saving" : "Save to SD"}</span>
          </button>
        </div>
      </header>

      {#if showOverview}<CanTiles />{/if}

      <CanFrameTable
        mode={canBus.tableMode}
        rows={canBus.snapshot.rows}
        byId={canBus.snapshot.byId}
        startUs={canBus.snapshot.startUs}
        selected={canBus.selectedKey}
        onselect={(key) => canBus.select(key)}
        ondetails={openDetails}
        onmode={(mode) => canBus.setTableMode(mode)}
        onclear={() => canBus.clearTable()}
        filter={canBus.filter}
        rules={canBus.rules}
        onfilter={(f) => canBus.setFilter(f)}
        onrules={(r) => canBus.setFilterRules(r)}
      />
    </div>
  </div>
</section>

<style>
  /* Same container tokens as Telemetry (.tel): gap, card padding, sticky top and the 860 px split. */
  .cb {
    --gap: 20px;
    --stick: 104px;
    display: grid;
    gap: var(--gap);
    min-width: 0;
  }
  .cb-ws {
    display: grid;
    gap: var(--gap, 20px);
    grid-template-columns: clamp(420px, 31vw, 470px) minmax(0, 1fr);
    align-items: start;
  }
  /* Closed: the side column is the 56 px rail alone. The track and the panel animate like the plot tools tray. */
  .cb-ws {
    transition: grid-template-columns 0.28s var(--ease-out);
  }
  .cb-ws.collapsed {
    grid-template-columns: 56px minmax(0, 1fr);
  }
  .cb-left {
    transition: gap 0.28s var(--ease-out);
  }
  .cb-ws.collapsed .cb-left {
    gap: 0;
  }
  .cb-side {
    transition:
      opacity 0.28s var(--ease-out),
      padding 0.28s var(--ease-out),
      visibility 0s;
  }
  .cb-side.closed {
    padding-inline: 0;
    opacity: 0;
    visibility: hidden;
    overflow: hidden;
    transition:
      opacity 0.28s var(--ease-out),
      padding 0.28s var(--ease-out),
      visibility 0s 0.28s;
  }
  @media (prefers-reduced-motion: reduce) {
    .cb-ws,
    .cb-left,
    .cb-side,
    .cb-side.closed {
      transition: none;
    }
  }
  /* Rail and panel share Telemetry's side column: same width, 10 px between them, sticky under the 104 px header. */
  .cb-left {
    display: flex;
    gap: 10px;
    align-items: stretch;
    min-width: 0;
    min-height: 460px;
  }
  @media (min-height: 760px) {
    .cb-left {
      position: sticky;
      top: var(--stick);
      max-height: calc(100dvh - 200px);
    }
  }
  /* 44x48 buttons, selected = accent icon + bar. The rail box is app.css .icon-rail. */
  .cb-rail button {
    position: relative;
    display: grid;
    place-items: center;
    width: 44px;
    height: 48px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: calc(var(--r-ctl) - 2px);
    background: none;
    color: var(--ctl-sub);
    cursor: pointer;
  }
  .cb-rail button[aria-current] {
    color: var(--accent-text);
  }
  .cb-rail button[aria-current]::before {
    content: '';
    position: absolute;
    left: -6px;
    top: 22%;
    bottom: 22%;
    width: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  @media (hover: hover) {
    .cb-rail button:hover {
      background: var(--hov);
    }
  }
  .cb-side {
    padding: 14px;
    flex: 1;
    overflow: auto;
    min-width: 0;
  }
  .cb-main {
    display: grid;
    gap: var(--gap, 20px);
    min-width: 0;
  }
  .cb-head {
    padding: var(--pad-card);
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 14px;
  }
  .cb-title h1 {
    margin: 0;
    font-size: 2.1rem;
    font-weight: 800;
    line-height: 1.15;
  }
  .cb-sub {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 6px 14px;
    margin: 4px 0 0;
    color: var(--muted);
    font-weight: 600;
  }
  .cb-sub a {
    color: var(--accent-text);
  }
  .cb-dot {
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: var(--muted);
  }
  .cb-dot.on {
    background: var(--ok);
  }
  .cb-actions {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 10px;
  }
  .cb-actions .btn {
    display: inline-flex;
    align-items: center;
    gap: 8px;
  }

  @media (max-width: 859px) {
    .cb {
      --gap: 14px;
    }
    .cb-ws {
      grid-template-columns: minmax(0, 1fr);
    }
    .cb-left {
      display: contents;
    }
    .cb-rail {
      align-self: auto;
      flex-direction: row;
      justify-content: start;
      position: static;
      width: 100%;
    }
    .cb-rail button {
      flex: 1 1 0;
      width: auto;
    }
    /* Phone: the rail is a horizontal bar, so the marker sits under the selected button. */
    .cb-rail button[aria-current]::before {
      left: 22%;
      right: 22%;
      top: auto;
      bottom: -6px;
      width: auto;
      height: 3px;
    }
    /* Phone order: views, then the header with its actions, the status tiles, the view panel, the frame table.
       cb-main drops its box so its children join the page grid, like Telemetry's header comes first. */
    .cb-main {
      display: contents;
    }
    .cb-head {
      order: 1;
    }
    .cb-rail {
      order: 2;
    }
    .cb-main > :global(.ct) {
      order: 3;
    }
    .cb-side {
      order: 4;
      grid-column: auto;
      position: static;
    }
    .cb-main > :global(.ft) {
      order: 5;
    }
    .cb-title h1 {
      font-size: 1.7rem;
    }
    /* Same phone header as Telemetry: the two actions share a full-width row. */
    .cb-actions {
      width: 100%;
    }
    .cb-actions .btn {
      flex: 1 1 0;
      justify-content: center;
    }
  }
</style>
