<script lang="ts">
  import { dndzone, SOURCES, TRIGGERS } from "svelte-dnd-action";
  import { flip } from "svelte/animate";
  import { onDestroy } from "svelte";

  import DashboardCardWrapper from "$lib/components/DashboardCardWrapper.svelte";
  import PIDSettingsModal from "$lib/components/PIDSettingsModal.svelte";

  import { dashboardStore } from "$lib/dashboardStore.svelte";
  import { createResizeHandler } from "$lib/resizeLogic.svelte";
  import { canStore } from "$lib/canStore.svelte.js";
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
  let isAddMenuOpen = $state(false);
  let dragDisabled = $state(true);
  let previewItem = $state<DashboardItem | null>(null);

  // Modal State
  let isModalOpen = $state(false);
  let modalTargetItem = $state<DashboardItem | null>(null);
  let modalIsNewCard = $state(false);

  // Resizing Subsystem
  const resizeHandler = createResizeHandler(
    () => containerWidth,
    DESKTOP_COLS,
    MOBILE_COLS,
    ROW_HEIGHT_PX,
    DESKTOP_GAP,
    MOBILE_GAP
  );

  // --- HANDLERS ---
  function handleConsider(e: CustomEvent<{ items: DashboardItem[]; info: { source: string; trigger: string } }>) {
    if (resizeHandler.state.resizingItemId) return;
    dashboardStore.items = e.detail.items;
    if (e.detail.info.source === SOURCES.KEYBOARD && e.detail.info.trigger === TRIGGERS.DRAG_STOPPED) {
      dragDisabled = true;
    }
  }

  function handleFinalize(e: CustomEvent<{ items: DashboardItem[]; info: { source: string } }>) {
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

  function openAddPresetSettings(type: CardType) {
    modalTargetItem = dashboardStore.addItem(type);
    modalIsNewCard = true;
    isModalOpen = true;
    isAddMenuOpen = false;
  }

  function handleModalSave(item: DashboardItem) {
    modalIsNewCard = false; // Prevent deletion in handleModalClose
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
    modalIsNewCard = false; // Reset flag
  }

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });

  onDestroy(() => {
    canStore.stopLogging();
  });
</script>

<svelte:window onscroll={() => (isAddMenuOpen = false)} />

<div
  class="unified-flow-dashboard"
  bind:clientWidth={containerWidth}
  class:user-is-resizing={!!resizeHandler.state.resizingItemId}
>
  {#if !dashboardStore.isInitialized}
    <div aria-busy="true">Initialising dashboard telemetry channels...</div>
  {:else if dashboardStore.items.length === 0}
    <div class="empty-placeholder">
      <p>Dashboard canvas workspace is empty. Click the action button below to map modules.</p>
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
              onDelete={(id) => dashboardStore.deleteItem(id)}
            />
          </div>
        </div>
      {/each}
    </div>
  {/if}
</div>

<PIDSettingsModal
  isOpen={isModalOpen}
  item={modalTargetItem}
  isNewCard={modalIsNewCard}
  bind:previewItem
  onSave={handleModalSave}
  onClose={handleModalClose}
/>

<div class="fab-container">
  {#if isAddMenuOpen}
    <div class="fab-menu-popover" onclick={(e) => e.stopPropagation()}>
      <div class="popover-section-title">New System Modules</div>
      <button class="fab-menu-item" onclick={() => openAddPresetSettings("battery")}>Battery Monitor</button>
      <button class="fab-menu-item" onclick={() => openAddPresetSettings("dtcs")}>DTC Trouble Log</button>
      <button class="fab-menu-item" onclick={() => openAddPresetSettings("overview")}>Performance Panel</button>
      <div class="popover-divider"></div>
      <button class="fab-menu-item setup-pid-highlight" onclick={() => openAddPresetSettings("pid")}>
        <span>＋ Add Dynamic Vehicle PID</span>
      </button>
    </div>
  {/if}
  <button
    class="fab-trigger"
    class:active={isAddMenuOpen}
    onclick={(e) => {
      e.stopPropagation();
      isAddMenuOpen = !isAddMenuOpen;
    }}
  >
    <span class="trigger-icon">＋</span>
  </button>
</div>

<style>
  .unified-flow-dashboard {
    width: 100%;
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
    min-height: 300px;
    padding: 12px;
    outline: none !important;
  }

  .unified-flow-card {
    grid-column: span var(--card-w, 10);
    grid-row: span var(--card-h, 7) !important;
    width: 100% !important;
    height: calc((var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)) !important;
    transition: transform 0.15s ease;
  }

  .is-resizing-target {
    z-index: 1000 !important;
    position: relative;
    transition: none !important;
    width: calc(100% + var(--resize-dx, 0px)) !important;
    height: calc(((var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)) + var(--resize-dy, 0px)) !important;
  }

  @media (max-width: 768px) {
    .unified-grid-zone {
      grid-template-columns: repeat(24, minmax(0, 1fr)) !important;
      gap: 12px !important;
      padding-bottom: 200px !important; /* Added extra space for mobile scrolling */
    }
    .unified-flow-card {
      grid-column: span min(24, var(--card-w, 10)) !important;
      height: calc((var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)) !important;
    }
    .is-resizing-target {
      width: calc(100% + var(--resize-dx, 0px)) !important;
      height: calc(((var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)) + var(--resize-dy, 0px)) !important;
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
    border: 1px solid rgba(255, 255, 255, 0.05);
  }

  .empty-placeholder {
    display: flex;
    justify-content: center;
    text-align: center;
    padding: 4rem 2rem;
    border: 2px dashed rgba(255, 255, 255, 0.1);
    border-radius: 12px;
    color: #a0a0a5;
  }

  .fab-container {
    position: fixed;
    bottom: 2rem;
    right: 2rem;
    z-index: 9999;
    display: flex;
    flex-direction: column;
    align-items: flex-end;
    pointer-events: none;
  }
  .fab-trigger,
  .fab-menu-popover {
    pointer-events: auto;
  }
  .fab-trigger {
    width: 56px;
    height: 56px;
    border-radius: 50%;
    background: #10b981;
    color: #ffffff;
    border: none;
    display: flex;
    align-items: center;
    justify-content: center;
    box-shadow: 0 8px 24px rgba(0, 0, 0, 0.4);
    cursor: pointer;
    transition: transform 0.2s, background 0.2s;
  }
  .fab-trigger.active {
    background: #ef4444;
    transform: rotate(135deg);
  }
  .fab-menu-popover {
    position: absolute;
    bottom: 72px;
    right: 0;
    background: #18181b;
    border: 1px solid rgba(255, 255, 255, 0.08);
    border-radius: 12px;
    padding: 12px;
    min-width: 240px;
    display: flex;
    flex-direction: column;
    gap: 4px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.4);
  }
  .popover-section-title {
    font-size: 0.75rem;
    text-transform: uppercase;
    color: #a0a0a5;
    padding: 4px 8px;
    font-weight: 700;
  }
  .popover-divider {
    height: 1px;
    background: rgba(255, 255, 255, 0.08);
    margin: 6px 4px;
  }
  .fab-menu-item {
    display: flex;
    align-items: center;
    width: 100%;
    background: transparent;
    border: none;
    color: #e4e4e7;
    padding: 10px 12px;
    text-align: left;
    border-radius: 6px;
    cursor: pointer;
    font-size: 0.9rem;
    font-weight: 500;
  }
  .fab-menu-item:hover {
    background: rgba(255, 255, 255, 0.05);
    color: #ffffff;
  }
  .setup-pid-highlight {
    color: #10b981;
    font-weight: 600;
  }
  .setup-pid-highlight:hover {
    background: rgba(16, 185, 129, 0.1);
    color: #34d399;
  }

  :global(#dnd-action-dragged-el) {
    box-shadow: 0 20px 40px rgba(0, 0, 0, 0.6) !important;
    border-radius: 8px !important;
    opacity: 0.95 !important;
    width: calc((((100vw - 24px) - (60 - 1) * 16px) / 60 * var(--card-w)) + ((var(--card-w) - 1) * 16px)) !important;
    height: calc((var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)) !important;
  }

  @media (max-width: 768px) {
    :global(#dnd-action-dragged-el) {
      width: calc((((100vw - 24px) - (24 - 1) * 12px) / 24 * var(--card-w)) + ((var(--card-w) - 1) * 12px)) !important;
      height: calc((var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)) !important;
    }
  }

  :global(.unified-grid-zone > div[style*="visibility: hidden"]) {
    visibility: visible !important;
    opacity: 0.15 !important;
    background: rgba(255, 255, 255, 0.02) !important;
    border: 2px dashed rgba(255, 255, 255, 0.2) !important;
    border-radius: 8px;
    width: 100% !important;
    height: calc((var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)) !important;
  }

  @media (max-width: 768px) {
    :global(.unified-grid-zone > div[style*="visibility: hidden"]) {
      height: calc((var(--card-h) * 10px) + ((var(--card-h) - 1) * 12px)) !important;
    }
  }
</style>
