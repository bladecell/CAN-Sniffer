<script lang="ts">
  import { untrack } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { canBus } from "$lib/canBus/canBusStore.svelte";
  import CanTiles from "$lib/components/canBus/CanTiles.svelte";
  import CanIdList from "$lib/components/canBus/CanIdList.svelte";
  import CanFrameTable from "$lib/components/canBus/CanFrameTable.svelte";
  import CanIdDetails from "$lib/components/canBus/CanIdDetails.svelte";
  import CanEcuIds from "$lib/components/canBus/CanEcuIds.svelte";
  import CanRules from "$lib/components/canBus/CanRules.svelte";
  import CanSend from "$lib/components/canBus/CanSend.svelte";
  import CanCaptures from "$lib/components/canBus/CanCaptures.svelte";
  import { captures } from "$lib/canBus/captureStore.svelte";

  type Tab = "ids" | "ecu" | "cap" | "info" | "send" | "rules";
  let tab = $state<Tab>("ids");

  const RAIL: { id: Tab; label: string; icon: string }[] = [
    { id: "ids", label: "IDs", icon: "waves-sine" },
    { id: "ecu", label: "ECU IDs", icon: "engine" },
    { id: "cap", label: "Captures", icon: "sd-card" },
    { id: "info", label: "ID details", icon: "circle-info" },
    { id: "send", label: "Send", icon: "upload" },
    { id: "rules", label: "Rules", icon: "filter" },
  ];

  function openDetails(key: number) {
    canBus.select(key);
    tab = "info";
  }

  // A click on empty space (not a row, the side panel or a control) clears the selection.
  function clearSelection(e: MouseEvent) {
    if (canBus.selectedKey !== null && !(e.target as Element).closest("tr, .cl-row, .cb-side, button, input, select, a, label")) canBus.select(null);
  }

  // The page attaches on mount. The stream stops on leave (0xB1) unless "Log in background" is on.
  $effect(() => untrack(() => canBus.attach()));

  // Bus status is read on every open and whenever the CAN state changes (GET /can_bus, GET /can/mode).
  $effect(() => {
    const trigger = connectionStore.connected && connectionStore.wsCanStatus.state;
    if (trigger) untrack(() => void canBus.refreshBus());
  });

  const live = $derived(connectionStore.connected && canBus.counters.streaming);
  const statusText = $derived(
    !connectionStore.connected
      ? "Not connected"
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
  <div class="cb-ws">
    <nav class="card cb-rail" aria-label="CAN views">
      {#each RAIL as r (r.id)}
        <button aria-current={tab === r.id ? "page" : undefined} aria-label={r.label} title={r.label} onclick={() => (tab = r.id)}>
          <Icon name={r.icon} size={20} />
        </button>
      {/each}
    </nav>

    <aside class="card cb-side" aria-label="CAN panel">
      {#if tab === "ids"}
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
        <CanIdDetails detail={canBus.snapshot.selected} startUs={canBus.snapshot.startUs} />
      {/if}
    </aside>

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
            <Icon name={captures.capturing ? "square" : "save"} size={16} />
            <span>{captures.capturing ? "Stop saving" : "Save to SD"}</span>
          </button>
        </div>
      </header>

      <CanTiles />

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
  .cb {
    display: grid;
    gap: var(--gap, 20px);
    min-width: 0;
  }
  .cb-ws {
    display: grid;
    gap: var(--gap, 20px);
    grid-template-columns: 56px 320px minmax(0, 1fr);
    align-items: start;
  }
  .cb-rail {
    display: grid;
    gap: 6px;
    align-content: start;
    padding: 8px;
    position: sticky;
    top: 120px;
  }
  .cb-rail button {
    position: relative;
    width: 40px;
    height: 40px;
    padding: 0;
    border: 0;
    border-radius: 14px;
    background: transparent;
    color: var(--muted);
    display: grid;
    place-items: center;
  }
  .cb-rail button[aria-current] {
    background: var(--ctl);
    color: var(--accent-text);
  }
  .cb-rail button[aria-current]::before {
    content: '';
    position: absolute;
    left: -8px;
    top: 22%;
    bottom: 22%;
    width: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .cb-side {
    padding: 16px;
    position: sticky;
    top: 120px;
    min-width: 0;
  }
  .cb-main {
    display: grid;
    gap: var(--gap, 20px);
    min-width: 0;
  }
  .cb-head {
    padding: 18px 22px;
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

  @media (max-width: 1100px) {
    .cb-ws {
      grid-template-columns: 56px minmax(0, 1fr);
    }
    .cb-side {
      grid-column: 2;
      position: static;
    }
    .cb-main {
      grid-column: 1 / -1;
    }
  }
  @media (max-width: 859px) {
    .cb-ws {
      grid-template-columns: minmax(0, 1fr);
    }
    .cb-rail {
      grid-auto-flow: column;
      justify-content: start;
      position: static;
      width: max-content;
      max-width: 100%;
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
    .cb-side {
      grid-column: auto;
      position: static;
    }
    .cb-head {
      padding: 14px 16px;
    }
    .cb-title h1 {
      font-size: 1.7rem;
    }
  }
</style>
