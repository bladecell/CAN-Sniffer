<script lang="ts">
  import { dndzone, SOURCES, TRIGGERS } from "svelte-dnd-action";
  import { flip } from "svelte/animate";
  import { onDestroy } from "svelte";
  import { fade } from "svelte/transition";

  import DashboardCardWrapper from "$lib/components/DashboardCardWrapper.svelte";
  import PIDSettingsModal from "$lib/components/PIDSettingsModal.svelte";

  import { dashboardStore } from "$lib/dashboardStore.svelte.ts";
  import { createResizeHandler } from "$lib/resizeLogic.svelte";
  import { canStore } from "$lib/canStore.svelte";
  import type { DashboardItem } from "$lib/types";

  const DESKTOP_COLS = 60;
  const MOBILE_COLS = 24;
  const ROW_HEIGHT_PX = 10;
  const DESKTOP_GAP = 16;
  const MOBILE_GAP = 12;
  const flipDurationMs = 200;

  let containerWidth = $state(0);
  let previewItem = $state<DashboardItem | null>(null);

  // OFFICIAL SVELTE-DND TRIGGER PATTERN
  let dragDisabled = $state(true);

  let isModalOpen = $state(false);
  let modalTargetItem = $state<DashboardItem | null>(null);
  let modalIsNewCard = $state(false);

  let bgMenu = $state({ x: 0, y: 0 });
  const isBgMenuOpen = $derived(dashboardStore.activeMenuId === "background");

  const resizeHandler = createResizeHandler(
    () => containerWidth,
    DESKTOP_COLS,
    MOBILE_COLS,
    ROW_HEIGHT_PX,
    DESKTOP_GAP,
    MOBILE_GAP,
  );

  const gridConstants = $derived(
    containerWidth > 768
      ? { gap: 16, padding: 12, cellW: 20, cellH: 10 }
      : { gap: 12, padding: 12, cellW: 20, cellH: 10 },
  );

  const dynamicCols = $derived(
    Math.max(
      1,
      Math.floor(
        (containerWidth - 2 * gridConstants.padding + gridConstants.gap) /
          (gridConstants.cellW + gridConstants.gap),
      ),
    ),
  );

  $effect(() => {
    const isResizing = !!resizeHandler.state.resizingItemId;
    if (isResizing) {
      document.body.classList.add("no-scroll");
    } else {
      document.body.classList.remove("no-scroll");
    }
  });

  function handleDesktopBgContext(e: MouseEvent) {
    if (dashboardStore.isEditMode) return;
    const target = e.target as HTMLElement;

    // Guarantee we clicked the grid floor, not an element inside a card
    if (
      target.classList.contains("unified-grid-zone") ||
      target.classList.contains("unified-flow-dashboard")
    ) {
      e.preventDefault();
      bgMenu = {
        x: Math.min(e.clientX, window.innerWidth - 210),
        y: Math.min(e.clientY, window.innerHeight - 160),
      };
      dashboardStore.activeMenuId = "background";
    }
  }

  function handleConsider(e: CustomEvent<any>) {
    if (resizeHandler.state.resizingItemId) return;
    dashboardStore.items = e.detail.items;
  }

  function handleFinalize(e: CustomEvent<any>) {
    if (resizeHandler.state.resizingItemId) return;
    dashboardStore.setItems(e.detail.items);
    dragDisabled = true; // ← already here, good
  }

  function handleRequestDrag() {
    dragDisabled = false;
    // Safety: if the user lifts without triggering DND finalize, re-lock
    window.addEventListener(
      "pointerup",
      () => {
        dragDisabled = true;
      },
      { once: true },
    );
  }

  function openEditSettings(item: DashboardItem) {
    modalTargetItem = item;
    modalIsNewCard = false;
    isModalOpen = true;
  }

  function openAddWizard() {
    modalTargetItem = dashboardStore.addItem("pid");
    modalIsNewCard = true;
    isModalOpen = true;
    dashboardStore.activeMenuId = null;
  }

  function handleModalSave(item: DashboardItem) {
    modalIsNewCard = false;
    dashboardStore.updateItem(item);
    isModalOpen = false;
    modalTargetItem = null;
    previewItem = null;
  }

  function handleModalClose() {
    if (modalIsNewCard && modalTargetItem)
      dashboardStore.deleteItem(modalTargetItem.id);
    isModalOpen = false;
    modalTargetItem = null;
    previewItem = null;
    modalIsNewCard = false;
  }

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });
  onDestroy(() => {
    canStore.stopLogging();
  });
</script>

<svelte:window
  onpointerdown={(e) => {
    if (
      dashboardStore.activeMenuId &&
      (e.button === 0 || e.pointerType === "touch")
    ) {
      const target = e.target as HTMLElement;
      if (!target.closest(".local-context-menu, .bg-context-menu")) {
        dashboardStore.activeMenuId = null;
      }
    }
  }}
/>

<div
  class="unified-flow-dashboard"
  bind:clientWidth={containerWidth}
  class:user-is-resizing={!!resizeHandler.state.resizingItemId}
  oncontextmenu={handleDesktopBgContext}
  style="--container-width: {containerWidth}px;"
>
  {#if !dashboardStore.isInitialized}
    <div aria-busy="true">Initialising dashboard telemetry channels...</div>
  {:else if dashboardStore.items.length === 0}
    <div class="empty-placeholder">
      <p>Dashboard canvas workspace is empty.</p>
      <button class="outline" onclick={openAddWizard}>Map First Module</button>
    </div>
  {:else}
    <div
      class="unified-grid-zone"
      style="
        --grid-cols: {dynamicCols}; --grid-gap: {gridConstants.gap}px;
        --grid-padding: {gridConstants.padding}px; --cell-w: {gridConstants.cellW}px; --cell-h: {gridConstants.cellH}px;
      "
      use:dndzone={{
        items: dashboardStore.items,
        flipDurationMs,
        // STRICT GATE: Slaved completely to the explicit drag handle callback
        dragDisabled:
          dragDisabled ||
          !dashboardStore.isEditMode ||
          !!resizeHandler.state.resizingItemId,
        dropTargetStyle: {},
        type: "dashboard",
        transformDraggedElement: (element, item) => {
          if (element) {
            element.style.setProperty("--card-w", (item.w || 10).toString());
            element.style.setProperty("--card-h", (item.h || 7).toString());
          }
        },
      }}
      onconsider={handleConsider}
      onfinalize={handleFinalize}
    >
      {#each dashboardStore.items as item (item.id)}
        {@const activeItem = previewItem?.id === item.id ? previewItem : item}
        {@const isResizing = resizeHandler.state.resizingItemId === item.id}
        <div
          class="unified-flow-card"
          class:is-resizing-target={isResizing}
          style="
            --card-w: {activeItem.w}; --card-h: {activeItem.h};
            --resize-dx: {isResizing ? resizeHandler.state.resizeDeltaX : 0}px;
            --resize-dy: {isResizing ? resizeHandler.state.resizeDeltaY : 0}px;
          "
          animate:flip={{ duration: flipDurationMs }}
        >
          <div class="card-inner-hull">
            <DashboardCardWrapper
              item={activeItem}
              onRequestDrag={handleRequestDrag}
              resizeStart={(e) => resizeHandler.start(e, item.id)}
              onOpenSettings={openEditSettings}
              onAddNew={openAddWizard}
              onDelete={(id) => dashboardStore.deleteItem(id)}
            />
          </div>
        </div>
      {/each}
    </div>
  {/if}
</div>

{#if isBgMenuOpen}
  <div class="bg-context-menu" style="top: {bgMenu.y}px; left: {bgMenu.x}px;">
    <button
      class={canStore.obd2Status?.continuous_running
        ? "stop-action"
        : "start-action"}
      onclick={() => {
        const state = canStore.obd2Status?.continuous_running ?? false;
        canStore.setContinuousPolling(!state);
        dashboardStore.activeMenuId = null;
      }}
    >
      {canStore.obd2Status?.continuous_running
        ? "Stop Logging"
        : "Start Logging"}
    </button>
    <button
      class="add-action"
      onclick={() => {
        openAddWizard();
        dashboardStore.activeMenuId = null;
      }}>Add New Module</button
    >
    <button
      class="layout-action"
      onclick={() => {
        dashboardStore.isEditMode = true;
        dashboardStore.activeMenuId = null;
      }}>Customize Canvas Layout</button
    >
  </div>
{/if}

<PIDSettingsModal
  isOpen={isModalOpen}
  item={modalTargetItem}
  isNewCard={modalIsNewCard}
  bind:previewItem
  onSave={handleModalSave}
  onClose={handleModalClose}
/>

<style>
  .unified-flow-dashboard {
    width: 100%;
    min-height: calc(100vh - 120px);
    overflow-x: hidden;
    -webkit-touch-callout: none;
    user-select: none;
    -webkit-user-select: none;
  }

  .user-is-resizing,
  .user-is-resizing * {
    cursor: se-resize !important;
    user-select: none !important;
    pointer-events: none !important;
  }
  .user-is-resizing .unified-grid-zone {
    pointer-events: auto !important;
  }

  .unified-grid-zone {
    display: grid !important;
    grid-template-columns: repeat(var(--grid-cols), var(--cell-w)) !important;
    grid-auto-rows: var(--cell-h) !important;
    grid-auto-flow: dense;
    gap: var(--grid-gap) !important;
    width: 100%;
    min-height: 500px;
    padding: var(--grid-padding);
    /* padding-bottom: 200px !important; */
    outline: none !important;
    justify-content: center;
  }

  .unified-flow-card {
    grid-column: span min(var(--grid-cols), var(--card-w, 10));
    grid-row: span var(--card-h, 7) !important;
    width: 100% !important;
    height: calc(
      (var(--card-h) * var(--cell-h)) + ((var(--card-h) - 1) * var(--grid-gap))
    ) !important;
    transition: transform 0.15s ease;
  }

  .is-resizing-target {
    z-index: 1000 !important;
    position: relative;
    transition: none !important;
    width: calc(100% + var(--resize-dx, 0px)) !important;
    height: calc(
      (
          (var(--card-h) * var(--cell-h)) +
            ((var(--card-h) - 1) * var(--grid-gap))
        ) + var(--resize-dy, 0px)
    ) !important;
  }

  .card-inner-hull {
    position: relative;
    width: 100%;
    height: 100% !important;
    border-radius: 8px;
    overflow: hidden;
    background: rgba(30, 30, 35, 0.6);
  }

  .empty-placeholder {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    padding: 6rem 2rem;
    border: 2px dashed rgba(255, 255, 255, 0.1);
    border-radius: 12px;
    color: #a0a0a5;
    gap: 1.5rem;
  }

  /* --- PICO BACKGROUND CONTEXT MENU --- */
  .bg-context-menu {
    position: fixed;
    z-index: 11000;
    background: var(--pico-card-background);
    border: 1px solid var(--pico-card-border-color);
    border-radius: var(--pico-border-radius);
    box-shadow: var(--pico-card-box-shadow);
    backdrop-filter: blur(16px);
    -webkit-backdrop-filter: blur(16px);
    padding: 6px;
    min-width: 210px;
    display: flex;
    flex-direction: column;
  }
  .bg-context-menu button {
    font-family: var(--pico-font-family);
    font-size: 0.85rem;
    font-weight: 500;
    color: var(--pico-color);
    background: transparent;
    border: none;
    text-align: left;
    padding: 10px 14px;
    border-radius: calc(var(--pico-border-radius) / 2);
    cursor: pointer;
  }
  .bg-context-menu button:hover {
    background: var(--pico-form-element-background);
    color: var(--pico-primary);
  }

  .add-action,
  .start-action {
    color: var(--module-accent, #10b981) !important;
  }
  .stop-action {
    color: orangered !important;
  }
  .layout-action {
    color: var(--pico-primary) !important;
    font-weight: 700;
  }

  :global(#dnd-action-dragged-el) {
    box-shadow: 0 20px 40px rgba(0, 0, 0, 0.6) !important;
    border-radius: 8px !important;
    opacity: 0.95 !important;
    /* Desktop: gap=16, cellW=20 */
    width: calc(
      (var(--card-w) * 20px) + ((var(--card-w) - 1) * 16px)
    ) !important;
    height: calc(
      (var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)
    ) !important;
  }
  @media (max-width: 768px) {
    :global(#dnd-action-dragged-el) {
      /* Mobile: gap=12, cellW=20 */
      width: calc(
        (var(--card-w) * 20px) + ((var(--card-w) - 1) * 12px)
      ) !important;
      height: calc(
        (var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)
      ) !important;
    }
  }
  :global(.unified-grid-zone > div[style*="visibility: hidden"]) {
    visibility: visible !important;
    opacity: 0.15 !important;
    background: rgba(255, 255, 255, 0.02) !important;
    border: 2px dashed var(--pico-primary) !important;
    border-radius: 8px;
    width: 100% !important;
    height: calc(
      (var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)
    ) !important;
  }
  @media (max-width: 768px) {
    :global(.unified-grid-zone > div[style*="visibility: hidden"]) {
      height: calc(
        (var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)
      ) !important;
    }
  }
</style>
