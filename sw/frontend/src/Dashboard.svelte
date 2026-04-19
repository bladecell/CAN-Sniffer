<script>
  import Grid, { GridItem } from "@appulsauce/svelte-grid";
  import PIDCardWrapper from "$lib/components/PIDCardWrapper.svelte";
  import { canStore } from "./lib/canStore.svelte.js";
  import { onMount, onDestroy, untrack } from "svelte";

  // 1. GRID CONFIGURATION & STATE

  const itemSize = { height: 10 };
  const MIN_SIZES = {
    card: { w: 12, h: 7 },
    chart: { w: 16, h: 12 },
    gauge: { w: 12, h: 12 },
  };
  const MAX_SIZES = {
    card: { w: MIN_SIZES.card.w * 1.5, h: MIN_SIZES.card.h * 1.5 },
    chart: { w: MIN_SIZES.chart.w * 1.5, h: MIN_SIZES.chart.h * 1.5 },
    gauge: { w: MIN_SIZES.gauge.w * 1.5, h: MIN_SIZES.gauge.h * 1.5 },
  };

  let containerWidth = $state(0);
  let gridController = $state();
  let items = $state([]);

  let dynamicCols = $derived.by(() => {
    if (containerWidth === 0) return 78; // Default
    if (containerWidth < 768) return 26; // Mobile: fits 2
    if (containerWidth < 1200) return 53; // Tablet: fits 4
    return 78; // Desktop: fits 6
  });

  // A helper to handle loading vs packing so our effect stays clean
  function applyOrRepackLayout(targetItems, cols) {
    const stored = localStorage.getItem(`dashboard-layout-${cols}`);
    if (stored) {
      try {
        const savedLayout = JSON.parse(stored);
        targetItems.forEach((item) => {
          const savedCard = savedLayout.find((loc) => loc.id === item.id);
          if (savedCard) {
            item.x = savedCard.x;
            item.y = savedCard.y;
            item.w = savedCard.w;
            item.h = savedCard.h;
            item.displayMode = savedCard.displayMode || "card";
          }
        });
        return; // Success! We applied the saved coordinates.
      } catch (e) {
        console.error("Layout corrupted, repacking...");
      }
    }

    // If we reach here, there was no saved layout (or it was corrupted)
    targetItems.sort((a, b) => a.y - b.y || a.x - b.x);
    repackGrid(targetItems, cols);
  }

  let isInitialized = $state(false);

  $effect(() => {
    // 1. Tell Svelte to watch BOTH the screen size AND the CAN data
    const currentCols = dynamicCols;
    const pids = canStore.pidDefinitions;

    if (pids?.length > 0) {
      untrack(() => {
        // Scenario A: VERY FIRST LOAD. Build the items and instantly apply the layout.
        if (items.length === 0) {
          let initialItems = pids.map((c) => ({
            id: c.pid.toString(),
            displayMode: "card",
            x: 0,
            y: 0, // These will be instantly overwritten!
            w: MIN_SIZES.card.w + 1,
            h: MIN_SIZES.card.h + 1,
            pid: c.pid,
            label: c.name,
            description: c.description,
            unit: c.unit,
            icon: c.icon,
            color: `#${c.color.toString(16).padStart(6, "0")}`,
            min: c.minValue || 0,
            max: c.maxValue || 100,
          }));

          applyOrRepackLayout(initialItems, currentCols);
          items = initialItems; // Assign them fully positioned!
          isInitialized = true;
        }

        // Scenario B: RESIZING. Items exist, just update their coordinates.
        else if (isInitialized) {
          applyOrRepackLayout(items, currentCols);
        }
      });
    }
  });

  function repackGrid(currentItems, totalCols) {
    const gridMap = [];

    // Condensed helper functions
    const isSpaceFree = (sX, sY, w, h) => {
      if (sX + w > totalCols) return false;
      for (let y = sY; y < sY + h; y++) {
        if (!gridMap[y]) gridMap[y] = [];
        for (let x = sX; x < sX + w; x++) if (gridMap[y][x]) return false;
      }
      return true;
    };

    const markSpace = (sX, sY, w, h) => {
      for (let y = sY; y < sY + h; y++) {
        if (!gridMap[y]) gridMap[y] = [];
        for (let x = sX; x < sX + w; x++) gridMap[y][x] = true;
      }
    };

    currentItems.forEach((item) => {
      const actualW = Math.min(item.w, totalCols);
      item.w = actualW;

      let placed = false;
      for (let y = 0; y < 9999 && !placed; y++) {
        for (let x = 0; x <= totalCols - actualW; x++) {
          if (isSpaceFree(x, y, actualW, item.h)) {
            item.x = x;
            item.y = y;
            markSpace(x, y, actualW, item.h);
            placed = true;
            break;
          }
        }
      }
    });
  }

  function saveLayout() {
    const cleanLayout = $state.snapshot(items).map((item) => ({
      id: item.id,
      x: item.x,
      y: item.y,
      w: item.w,
      h: item.h,
      displayMode: item.displayMode,
    }));
    localStorage.setItem(
      `dashboard-layout-${dynamicCols}`,
      JSON.stringify(cleanLayout),
    );
  }

  // 3. HARDWARE & LOGGING LOGIC

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });

  onDestroy(() => {
    canStore.stopLogging();
  });

  // 4. CONTEXT MENU LOGIC

  let contextMenu = $state({
    show: false,
    x: 0,
    y: 0,
    targetId: null,
  });

  function handleRightClick(event, itemId) {
    event.preventDefault(); // Prevents the default browser menu from opening!

    contextMenu.show = true;
    contextMenu.x = event.clientX;
    contextMenu.y = event.clientY;
    contextMenu.targetId = itemId;
  }

  function closeMenu() {
    contextMenu.show = false;
  }

  function deleteCard(id) {
    // Filter out the deleted card
    items = items.filter((item) => item.id !== id);
    closeMenu();

    untrack(() => {
      repackGrid(items, dynamicCols);
      saveLayout();
    });
  }

  function toggleMode(id) {
    const item = items.find((i) => i.id === id);
    if (item) {
      item.displayMode =
        item.displayMode === "card"
          ? "chart"
          : (item.displayMode =
              item.displayMode === "chart" ? "gauge" : "card");
      items = [...items]; // Trigger reactivity
    }
    closeMenu();
  }
</script>

<svelte:window onclick={closeMenu} onscroll={closeMenu} />

<div
  class="grid-container"
  bind:clientWidth={containerWidth}
  style="width: 100%;"
>
  {#if items.length === 0}
    <div aria-busy="true">Loading dashboard...</div>
  {:else}
    {#key dynamicCols}
      <Grid
        bind:controller={gridController}
        {itemSize}
        cols={dynamicCols}
        collision="compress"
        autoCompress={false}
        onchange={saveLayout}
      >
        {#each items as item (item.id)}
          <GridItem
            bind:x={item.x}
            bind:y={item.y}
            bind:w={item.w}
            bind:h={item.h}
            min={MIN_SIZES[item.displayMode]}
            max={{
              w: MIN_SIZES[item.displayMode].w * 1.5,
              h: MIN_SIZES[item.displayMode].h * 1.5,
            }}
            resizerClass="grid-item-resizer"
            previewClass="grid-item-preview"
          >
            {#snippet children()}
              <!-- svelte-ignore a11y_no_static_element_interactions -->
              <div
                style="width: 100%; height: 100%;"
                oncontextmenu={(e) => handleRightClick(e, item.id)}
              >
                <PIDCardWrapper {...item} />
              </div>
            {/snippet}
          </GridItem>
        {/each}
      </Grid>
    {/key}
  {/if}
</div>

{#if contextMenu.show}
  <!-- svelte-ignore a11y_click_events_have_key_events -->
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <div
    class="custom-context-menu"
    style="top: {contextMenu.y}px; left: {contextMenu.x}px;"
    onclick={(e) => e.stopPropagation()}
  >
    <button onclick={() => deleteCard(contextMenu.targetId)}>
      Delete Card
    </button>
    <button onclick={() => toggleMode(contextMenu.targetId)}>
      Edit Properties
    </button>
  </div>
{/if}

<style>
  .custom-context-menu {
    position: fixed; /* Fixed is crucial here! */
    z-index: 9999; /* Make sure it pops over the grid cards */
    background: var(--backdrop-filter-background);
    backdrop-filter: var(--backdrop-filter);
    -webkit-backdrop-filter: var(--backdrop-filter);
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 8px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
    padding: 8px;
    display: flex;
    flex-direction: column;
    gap: 4px;
    min-width: 160px;
  }

  .custom-context-menu button {
    background: transparent;
    color: var(--pico-secondary);
    border: none;
    text-align: left;
    padding: 8px 12px;
    border-radius: 4px;
    cursor: pointer;
    transition: background 0.2s;
  }

  .custom-context-menu button:hover {
    background: rgba(255, 255, 255, 0.05);
  }
</style>
