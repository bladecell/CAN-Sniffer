<script lang="ts">
  import { dndzone, SOURCES, TRIGGERS } from "svelte-dnd-action";
  import { flip } from "svelte/animate";
  import { onDestroy } from "svelte";

  import DashboardCardWrapper from "$lib/components/DashboardCardWrapper.svelte";
  import PIDSettingsModal from "$lib/components/PIDSettingsModal.svelte";

  import { dashboardStore } from "$lib/dashboardStore.svelte";
  import { createResizeHandler } from "$lib/resizeLogic.svelte";
  import { canStore } from "$lib/canStore.svelte";
  import type { DashboardItem, CardType } from "$lib/types";

  // --- GRID CONSTANTS ---
  const DESKTOP_COLS = 60;
  const MOBILE_COLS = 24;
  const ROW_HEIGHT_PX = 10;
  const DESKTOP_GAP = 16;
  const MOBILE_GAP = 12;
  const flipDurationMs = 200;

  // --- STATE ---
  let containerWidth = $state(0);
  let dragDisabled = $state(true);
  let previewItem = $state<DashboardItem | null>(null);

  // Modal State
  let isModalOpen = $state(false);
  let modalTargetItem = $state<DashboardItem | null>(null);
  let modalIsNewCard = $state(false);

  // Background Context Menu State
  let bgMenu = $state({ x: 0, y: 0 });
  const isBgMenuOpen = $derived(dashboardStore.activeMenuId === "background");

  // Resizing Subsystem
  const resizeHandler = createResizeHandler(
    () => containerWidth,
    DESKTOP_COLS,
    MOBILE_COLS,
    ROW_HEIGHT_PX,
    DESKTOP_GAP,
    MOBILE_GAP,
  );

  // --- HANDLERS ---
  function handleConsider(
    e: CustomEvent<{
      items: DashboardItem[];
      info: { source: string; trigger: string };
    }>,
  ) {
    if (resizeHandler.state.resizingItemId) return;
    dashboardStore.items = e.detail.items;
    if (
      e.detail.info.source === SOURCES.KEYBOARD &&
      e.detail.info.trigger === TRIGGERS.DRAG_STOPPED
    ) {
      dragDisabled = true;
    }
  }

  function handleFinalize(
    e: CustomEvent<{ items: DashboardItem[]; info: { source: string } }>,
  ) {
    if (resizeHandler.state.resizingItemId) return;
    dashboardStore.setItems(e.detail.items);
    if (e.detail.info.source === SOURCES.POINTER) {
      dragDisabled = true;
    }
  }

  function openEditSettings(item: DashboardItem) {
    modalTargetItem = item;
    modalIsNewCard = false;
    isModalOpen = true;
  }

  function openAddWizard() {
    // Default to adding a PID
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
    if (modalIsNewCard && modalTargetItem) {
      dashboardStore.deleteItem(modalTargetItem.id);
    }
    isModalOpen = false;
    modalTargetItem = null;
    previewItem = null;
    modalIsNewCard = false;
  }

  function handleBgContextMenu(e: MouseEvent) {
    // Only show if we click the actual grid background, not a card
    if (
      (e.target as HTMLElement).classList.contains("unified-grid-zone") ||
      (e.target as HTMLElement).classList.contains("unified-flow-dashboard")
    ) {
      e.preventDefault();
      bgMenu.x = e.clientX;
      bgMenu.y = e.clientY;
      dashboardStore.activeMenuId = "background";
    }
  }

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });

  onDestroy(() => {
    canStore.stopLogging();
  });
</script>

<svelte:window
  onclick={() => (dashboardStore.activeMenuId = null)}
  ontouchstart={() => (dashboardStore.activeMenuId = null)}
/>

<div
  class="unified-flow-dashboard"
  bind:clientWidth={containerWidth}
  class:user-is-resizing={!!resizeHandler.state.resizingItemId}
  oncontextmenu={handleBgContextMenu}
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
      use:dndzone={{
        items: dashboardStore.items,
        flipDurationMs,
        dragDisabled: dragDisabled || !!resizeHandler.state.resizingItemId,
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
            --card-w: {activeItem.w}; 
            --card-h: {activeItem.h};
            --resize-dx: {isResizing ? resizeHandler.state.resizeDeltaX : 0}px;
            --resize-dy: {isResizing ? resizeHandler.state.resizeDeltaY : 0}px;
          "
          animate:flip={{ duration: flipDurationMs }}
        >
          <div class="card-inner-hull">
            <DashboardCardWrapper
              item={activeItem}
              onRequestDrag={() => (dragDisabled = false)}
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
  <div
    class="bg-context-menu"
    style="top: {bgMenu.y}px; left: {bgMenu.x}px;"
    onclick={(e) => e.stopPropagation()}
  >
    <button
      class={canStore.obd2Status?.continuous_running
        ? "stop-action"
        : "start-action"}
      onclick={() => {
        const currentState = canStore.obd2Status?.continuous_running ?? false;
        canStore.setContinuousPolling(!currentState);
        dashboardStore.activeMenuId = null;
      }}
    >
      {canStore.obd2Status?.continuous_running
        ? "Stop Polling"
        : "Start Polling"}
    </button>
    <button class="add-action" onclick={openAddWizard}>Add New Module</button>
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
    grid-template-columns: repeat(60, minmax(0, 1fr)) !important;
    grid-auto-rows: 10px !important;
    grid-auto-flow: dense;
    gap: 16px !important;
    width: 100%;
    min-height: 500px;
    padding: 12px;
    padding-bottom: 200px !important;
    outline: none !important;
  }

  .unified-flow-card {
    grid-column: span var(--card-w, 10);
    grid-row: span var(--card-h, 7) !important;
    width: 100% !important;
    height: calc(
      (var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)
    ) !important;
    transition: transform 0.15s ease;
  }

  .is-resizing-target {
    z-index: 1000 !important;
    position: relative;
    transition: none !important;
    width: calc(100% + var(--resize-dx, 0px)) !important;
    height: calc(
      ((var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)) +
        var(--resize-dy, 0px)
    ) !important;
  }

  @media (max-width: 768px) {
    .unified-grid-zone {
      grid-template-columns: repeat(24, minmax(0, 1fr)) !important;
      gap: 12px !important;
    }
    .unified-flow-card {
      grid-column: span min(24, var(--card-w, 10)) !important;
      height: calc(
        (var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)
      ) !important;
    }
    .is-resizing-target {
      width: calc(100% + var(--resize-dx, 0px)) !important;
      height: calc(
        ((var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)) +
          var(--resize-dy, 0px)
      ) !important;
    }
  }

  .card-inner-hull {
    position: relative;
    width: 100%;
    height: 100% !important;
    min-height: 100% !important;
    border-radius: 8px;
    overflow: hidden;
    background: rgba(30, 30, 35, 0.6);
  }

  .empty-placeholder {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    text-align: center;
    padding: 6rem 2rem;
    border: 2px dashed rgba(255, 255, 255, 0.1);
    border-radius: 12px;
    color: #a0a0a5;
    gap: 1.5rem;
  }

  .bg-context-menu {
    position: fixed;
    z-index: 11000;
    background: rgba(20, 20, 24, 0.95);
    backdrop-filter: blur(12px);
    border: 1px solid rgba(255, 255, 255, 0.08);
    border-radius: 8px;
    padding: 6px;
    min-width: 180px;
    display: flex;
    flex-direction: column;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
  }
  .bg-context-menu button {
    background: transparent;
    color: #10b981;
    border: none;
    text-align: left;
    padding: 12px 14px;
    border-radius: 4px;
    cursor: pointer;
    font-size: 0.9rem;
    font-weight: 600;
    width: 100%;
    white-space: nowrap;
  }
  .bg-context-menu button:hover {
    background: rgba(255, 255, 255, 0.06);
    color: #ffffff;
  }
  .add-action,
  .start-action {
    color: #00b478 !important;
  }
  .stop-action {
    color: #f56b3d !important;
  }

  :global(#dnd-action-dragged-el) {
    box-shadow: 0 20px 40px rgba(0, 0, 0, 0.6) !important;
    border-radius: 8px !important;
    opacity: 0.95 !important;
    width: calc(
      (((100vw - 24px) - (60 - 1) * 16px) / 60 * var(--card-w)) +
        ((var(--card-w) - 1) * 16px)
    ) !important;
    height: calc(
      (var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)
    ) !important;
  }

  @media (max-width: 768px) {
    :global(#dnd-action-dragged-el) {
      width: calc(
        (((100vw - 24px) - (24 - 1) * 12px) / 24 * var(--card-w)) +
          ((var(--card-w) - 1) * 12px)
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
    border: 2px dashed rgba(255, 255, 255, 0.2) !important;
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
