<script lang="ts">
  import { untrack } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { canBus } from "$lib/canBus/canBusStore.svelte";
  import type { TableFilter } from "$lib/canBus/canModel";
  import CanTiles from "$lib/components/canBus/CanTiles.svelte";
  import CanIdList from "$lib/components/canBus/CanIdList.svelte";
  import CanFrameTable from "$lib/components/canBus/CanFrameTable.svelte";

  type Tab = "ids";
  let tab = $state<Tab>("ids");

  const FILTERS: { id: TableFilter; label: string }[] = [
    { id: "all", label: "All" },
    { id: "changed", label: "Changed" },
    { id: "obd", label: "OBD" },
  ];

  const RAIL: { id: Tab; label: string; icon: string }[] = [{ id: "ids", label: "IDs", icon: "list" }];

  // The page owns the raw stream: it starts on mount and stops (0xB1) when the page goes away.
  $effect(() => canBus.start());

  // Bus status is read on every open and whenever the CAN state changes (GET /can_bus, GET /can/mode).
  $effect(() => {
    const trigger = connectionStore.connected && connectionStore.wsCanStatus.state;
    if (trigger) untrack(() => void canBus.refreshBus());
  });

  const live = $derived(connectionStore.connected && canBus.counters.streaming);
  const statusText = $derived(
    !connectionStore.connected
      ? "Not connected"
      : !canBus.counters.streaming
        ? "Waiting for frames"
        : canBus.paused
          ? "Paused"
          : "Live",
  );
  const bitrateText = $derived(canBus.bus ? `${canBus.bus.bitrate / 1000} kbit/s` : "Bitrate unknown");
  const modeText = $derived(canBus.mode ? (canBus.mode.mode === "raw" ? "Raw" : "Monitor") : "Mode unknown");

  function saveToSd() {
    // Captures need firmware support that is not built yet (UI3e-16).
    alertStore.add("Saving to the SD card is not available yet", "warning");
  }
</script>

<section class="cb">
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
        />
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
          <div class="seg" role="radiogroup" aria-label="Show">
            {#each FILTERS as f (f.id)}
              <button role="radio" aria-checked={canBus.filter === f.id} onclick={() => canBus.setFilter(f.id)}>{f.label}</button>
            {/each}
          </div>
          <button class="btn" aria-pressed={canBus.paused} onclick={() => canBus.togglePause()}>
            <Icon name={canBus.paused ? "play" : "pause"} size={16} />
            <span>{canBus.paused ? "Resume" : "Pause"}</span>
          </button>
          <button class="btn pri" aria-disabled="true" onclick={saveToSd}>
            <Icon name="save" size={16} />
            Save to SD
          </button>
        </div>
      </header>

      <CanTiles />

      <CanFrameTable
        rows={canBus.snapshot.rows}
        startUs={canBus.snapshot.startUs}
        selected={canBus.selectedKey}
        onselect={(key) => canBus.select(key)}
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
  .cb-actions .btn[aria-disabled='true'] {
    opacity: 0.55;
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
