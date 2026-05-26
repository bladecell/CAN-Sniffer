<script>
  import PIDCard from "$lib/components/PIDCard.svelte";
  import CANConnectionCard from "$lib/components/CANConnectionCard.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import { canStore } from "$lib/canStore.svelte";
  import { alertStore } from "./lib/alertStore.svelte";

  import { createSwapy } from "swapy";
  import { onDestroy, untrack } from "svelte";

  function createCardFromPID(pidData) {
    return {
      label: pidData.name,
      value: 0,
      unit: pidData.unit,
      icon: pidData.icon,
      color: `#${pidData.color.toString(16).padStart(6, "0")}`,
      min: pidData.minValue || 0,
      max: pidData.maxValue || 100,
      supported: pidData.supported ?? true,
      valid: true,
      visible: true,
      pid: pidData.pid,
      mode: pidData.mode,
    };
  }

  // Persistence Keys
  const ORDER_KEY = "can-sniffer-telemetry-order";
  const VISIBILITY_KEY = "can-sniffer-telemetry-visibility";

  let searchTerm = $state("");

  // 1. Core State
  let visibilityState = $state(
    JSON.parse(localStorage.getItem(VISIBILITY_KEY) || "{}"),
  );

  // CRITICAL: savedOrder is NO LONGER a $state. Svelte won't react to it mid-drag.
  let savedOrder = JSON.parse(localStorage.getItem(ORDER_KEY) || "[]");

  let renderCards = $state([]);
  let renderKey = $state(0); // Used to nuke and rebuild the grid cleanly

  // 2. Sync Function: Builds the view array, bumps renderKey only if layout changed
  function syncRenderCards() {
    const cardMap = new Map(
      canStore.pidDefinitions.map((c) => [
        c.pid.toString(),
        createCardFromPID(c),
      ]),
    );
    const result = [];

    for (const pid of savedOrder) {
      if (visibilityState[pid] ?? true) {
        const card = cardMap.get(pid);
        if (card) result.push({ ...card, visible: true });
      }
    }

    // Check if the physical lineup of cards changed
    const oldPids = renderCards.map((c) => c.pid.toString()).join(",");
    const newPids = result.map((c) => c.pid.toString()).join(",");

    renderCards = result;

    if (oldPids !== newPids) {
      // The cards shown changed (added/removed). Nuke the DOM for Swapy.
      renderKey++;
    }
  }

  // 3. Effect: Watch for data updates & new PIDs safely
  $effect(() => {
    // Read definitions so Svelte tracks it for live telemetry updates
    const defs = canStore.pidDefinitions;

    untrack(() => {
      let changed = false;
      for (const p of defs) {
        const pidStr = p.pid.toString();
        if (!savedOrder.includes(pidStr)) {
          savedOrder.push(pidStr);
          changed = true;
        }
      }

      if (changed) {
        localStorage.setItem(ORDER_KEY, JSON.stringify(savedOrder));
      }
      syncRenderCards();
    });
  });

  // 4. Svelte Action to manage Swapy lifecycle perfectly
  function swapyAction(node) {
    const swapy = createSwapy(node, { animation: "dynamic" });

    swapy.onSwap((event) => {
      if (event?.newSlotItemMap?.asArray) {
        const newVisibleItems = [...event.newSlotItemMap.asArray]
          .sort((a, b) => parseInt(a.slot) - parseInt(b.slot))
          .map((entry) => entry.item);

        // Update our background array and local storage quietly
        const newOrder = [...savedOrder];
        let visibleCounter = 0;

        for (let i = 0; i < newOrder.length; i++) {
          const pid = newOrder[i];
          if (visibilityState[pid] ?? true) {
            if (newVisibleItems[visibleCounter]) {
              newOrder[i] = newVisibleItems[visibleCounter];
            }
            visibleCounter++;
          }
        }

        savedOrder = newOrder;
        localStorage.setItem(ORDER_KEY, JSON.stringify(savedOrder));
        // Notice we DO NOT call syncRenderCards() here. Svelte sleeps while you drag.
      }
    });

    return {
      destroy() {
        swapy.destroy();
      },
    };
  }

  // 5. Derived state for the search dropdown
  let searchResults = $derived(
    canStore.pidDefinitions
      .map((p) => {
        const card = createCardFromPID(p);
        card.visible = visibilityState[p.pid.toString()] ?? true;
        return card;
      })
      .filter((c) => c.label.toLowerCase().includes(searchTerm.toLowerCase())),
  );

  function toggleCard(pid) {
    const pidStr = pid.toString();
    visibilityState[pidStr] = !(visibilityState[pidStr] ?? true);
    localStorage.setItem(VISIBILITY_KEY, JSON.stringify(visibilityState));
    syncRenderCards(); // This triggers the #key block to rebuild
  }

  $effect(() => {
    if (canStore.connected) {
      canStore.startLogging();
    }
  });

  onDestroy(() => {
    canStore.stopLogging();
  });
</script>

<div class="controls-container">
  <div class="status-group">
    <CANConnectionCard />
    <Switch
      label="Data Polling"
      checked={canStore.obd2Status?.continuous_running}
      statusText={canStore.obd2Status?.continuous_running
        ? "Running"
        : "Stopped"}
      onchange={(val) => canStore.setContinuousPolling(val)}
    />
  </div>

  <details class="dropdown">
    <summary> Parameters to show </summary>
    <ul>
      <li class="search-container">
        <input
          type="search"
          placeholder="Search..."
          bind:value={searchTerm}
          onclick={(e) => e.stopPropagation()}
        />
      </li>
      {#each searchResults as card (card.pid)}
        <li>
          <label>
            <input
              type="checkbox"
              checked={card.visible}
              onchange={() => toggleCard(card.pid)}
            />
            {card.label}
          </label>
        </li>
      {/each}
      {#if searchResults.length === 0}
        <li class="no-results">No matches found</li>
      {/if}
    </ul>
  </details>
</div>

<style>
  .status-group {
    display: flex;
    gap: 1rem;
    align-items: stretch;
  }

  details.dropdown > summary + ul li:first-of-type {
    margin-top: 0;
  }

  .controls-container {
    display: flex;
    justify-content: space-between;
    align-items: stretch;
    gap: 1rem;
    margin-bottom: 1rem;
    flex-wrap: wrap;
  }

  .dropdown {
    margin-bottom: 0;
    height: auto;
    min-width: 250px;
    position: relative;
  }

  .dropdown summary {
    height: 100%;
    display: flex;
    align-items: center;
    margin-bottom: 0;
    white-space: nowrap;
  }

  .dropdown ul {
    max-height: 500px;
    overflow-y: auto;
    scroll-behavior: smooth;
    backdrop-filter: var(--backdrop-filter);
    -webkit-backdrop-filter: var(--backdrop-filter);
    background-color: var(--backdrop-filter-background) !important;
    position: absolute;
  }

  .cards-grid {
    display: grid;
    grid-template-columns: repeat(
      auto-fill,
      minmax(min(100%, var(--card-pref-width)), 1fr)
    );
    gap: var(--card-gap);
  }

  .search-container {
    position: sticky;
    top: 0;
    padding: 0.5rem;
    z-index: 10;
    box-shadow: 0 4px 6px -6px rgba(0, 0, 0, 0.2);
    background-color: var(--pico-dropdown-background-color) !important;
  }

  .search-container input {
    margin: 0;
  }

  .no-results {
    text-align: center;
    color: var(--pico-muted-color);
    padding: 1rem;
  }

  @media (max-width: 768px) {
    .controls-container {
      flex-direction: column;
    }

    .status-group {
      width: 100%;
    }

    .status-group > :global(*) {
      flex: 1;
    }

    .dropdown {
      width: 100%;
    }
  }
</style>
