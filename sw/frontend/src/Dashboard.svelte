<script lang="ts">
  import { dndzone, SOURCES, TRIGGERS } from "svelte-dnd-action";
  import { flip } from "svelte/animate";
  import PIDCardWrapper from "$lib/components/PIDCardWrapper.svelte";
  import PIDSettingsModal from "$lib/components/PIDSettingsModal.svelte";

  import { canStore } from "$lib/canStore.svelte.js";
  import { onDestroy, untrack } from "svelte";

  // --- LOCKED CONFIGURABLE TIMING CONSTANTS (IN MILLISECONDS) ---
  const DRAG_HOLD_DELAY_MS = 20; // Fast PC pickup / Snappy mobile hold track
  const POPUP_HOLD_DELAY_MS = 1000; // 1 second deliberate hold window for popup toggle

  const DESKTOP_COLS = 60;
  const MOBILE_COLS = 24;
  const ROW_HEIGHT_PX = 10;
  const DESKTOP_GAP = 16;
  const MOBILE_GAP = 12;
  const flipDurationMs = 200;

  let containerWidth = $state(0);
  let isInitialized = $state(false);
  let isAddMenuOpen = $state(false);
  let items = $state<any[]>([]);
  let previewItem = $state<any>(null);

  // Controlled reactively via our long-press timers to prevent event clashing
  let dragDisabled = $state(true);

  let isModalOpen = $state(false);
  let modalTargetItem = $state<any | null>(null);
  let modalIsNewCard = $state(false);

  let resizingItemId = $state<string | null>(null);
  let initialMouseX = 0;
  let initialMouseY = 0;
  let initialWidthPx = 0;
  let initialHeightPx = 0;
  let resizeDeltaX = $state(0);
  let resizeDeltaY = $state(0);

  let animationFrameToken: number | null = null;
  let latestPointerEvent = $state<PointerEvent | null>(null);
  let lastRenderTime = 0;
  const RENDER_INTERVAL_MS = 1000 / 30;

  $effect(() => {
    if (containerWidth > 0 && !isInitialized) {
      untrack(() => {
        const storedLayout = localStorage.getItem("dashboard-unified-flow");
        if (storedLayout) {
          try {
            items = JSON.parse(storedLayout);
          } catch (e) {
            items = [];
          }
        } else {
          items = [];
        }
        isInitialized = true;
      });
    }
  });

  function saveLayout() {
    if (!isInitialized) return;
    localStorage.setItem("dashboard-unified-flow", JSON.stringify(items));
  }

  function handleConsider(
    e: CustomEvent<{ items: any[]; info: { source: string; trigger: string } }>,
  ) {
    if (resizingItemId) return;
    items = e.detail.items;
    const { source, trigger } = e.detail.info;
    if (source === SOURCES.KEYBOARD && trigger === TRIGGERS.DRAG_STOPPED) {
      dragDisabled = true;
    }
  }

  function handleFinalize(
    e: CustomEvent<{ items: any[]; info: { source: string } }>,
  ) {
    if (resizingItemId) return;
    items = e.detail.items;
    const { source } = e.detail.info;
    if (source === SOURCES.POINTER) {
      dragDisabled = true; // Instantly lock dragging after a card drops
    }
    saveLayout();
  }

  // Exposed callback to allow child items to unlock the grid's engine
  function triggerDragActivation() {
    dragDisabled = false;
  }

  function startResizing(event: any, id: string) {
    dragDisabled = true; // Blocks dragging actions while adjusting metrics

    const clientX = event.touches ? event.touches[0].clientX : event.clientX;
    const clientY = event.touches ? event.touches[0].clientY : event.clientY;

    const targetEl = event.target as HTMLElement;
    if (!targetEl) return;

    const cardElement = targetEl.closest(".unified-flow-card");
    if (!cardElement) return;

    resizingItemId = id;
    initialMouseX = clientX;
    initialMouseY = clientY;

    const rect = cardElement.getBoundingClientRect();
    initialWidthPx = rect.width;
    initialHeightPx = rect.height;

    resizeDeltaX = 0;
    resizeDeltaY = 0;
    lastRenderTime = performance.now();

    if (event.touches) {
      window.addEventListener("touchmove", queueResizeUpdateTouch, {
        passive: false,
      });
      window.addEventListener("touchend", stopResizing);
    } else {
      window.addEventListener("pointermove", queueResizeUpdate);
      window.addEventListener("pointerup", stopResizing);
    }
  }

  function queueResizeUpdateTouch(e: TouchEvent) {
    if (resizingItemId) e.preventDefault(); // Lock mobile window scrolling
    latestPointerEvent = e.touches[0] as unknown as PointerEvent;
    if (!animationFrameToken) {
      animationFrameToken = requestAnimationFrame(processThrottledResize);
    }
  }

  function queueResizeUpdate(event: PointerEvent) {
    latestPointerEvent = event;
    if (!animationFrameToken) {
      animationFrameToken = requestAnimationFrame(processThrottledResize);
    }
  }

  function processThrottledResize() {
    animationFrameToken = null;
    if (!resizingItemId || !latestPointerEvent) return;

    const currentTime = performance.now();
    const timeElapsed = currentTime - lastRenderTime;

    if (timeElapsed < RENDER_INTERVAL_MS) {
      animationFrameToken = requestAnimationFrame(processThrottledResize);
      return;
    }

    lastRenderTime = currentTime - (timeElapsed % RENDER_INTERVAL_MS);
    resizeDeltaX = latestPointerEvent.clientX - initialMouseX;
    resizeDeltaY = latestPointerEvent.clientY - initialMouseY;
  }

  function stopResizing() {
    if (resizingItemId && containerWidth > 0) {
      const item = items.find((i) => i.id === resizingItemId);
      if (item) {
        const isMobile = window.innerWidth <= 768;
        const activeColsCount = isMobile ? MOBILE_COLS : DESKTOP_COLS;
        const currentGap = isMobile ? MOBILE_GAP : DESKTOP_GAP;

        const singleCellWidthPx =
          (containerWidth - (activeColsCount - 1) * currentGap) /
          activeColsCount;
        const finalWidthPx = initialWidthPx + resizeDeltaX;
        const finalHeightPx = initialHeightPx + resizeDeltaY;

        const finalW = Math.round(
          (finalWidthPx + currentGap) / (singleCellWidthPx + currentGap),
        );
        const finalH = Math.round(
          (finalHeightPx + currentGap) / (ROW_HEIGHT_PX + currentGap),
        );

        item.w = Math.max(4, Math.min(activeColsCount, finalW));
        item.h = Math.max(2, Math.min(50, finalH));
        items = [...items];
      }
    }
    cleanupResizeListeners();
    saveLayout();
  }

  function cleanupResizeListeners() {
    resizingItemId = null;
    latestPointerEvent = null;
    resizeDeltaX = 0;
    resizeDeltaY = 0;
    if (animationFrameToken) {
      cancelAnimationFrame(animationFrameToken);
      animationFrameToken = null;
    }
    window.removeEventListener("pointermove", queueResizeUpdate);
    window.removeEventListener("pointerup", stopResizing);
    window.removeEventListener("touchmove", queueResizeUpdateTouch);
    window.removeEventListener("touchend", stopResizing);
  }

  function openEditSettings(item: any) {
    modalTargetItem = item;
    modalIsNewCard = false;
    isModalOpen = true;
  }

  function openAddPresetSettings(
    type: "pid" | "battery" | "dtcs" | "overview",
  ) {
    modalTargetItem = {
      cardType: type,
      w: type === "pid" ? 10 : 6,
      h: type === "pid" ? 7 : 6,
    };
    modalIsNewCard = true;
    isModalOpen = true;
    isAddMenuOpen = false;
  }

  function handleModalSave(configuredItem: any) {
    if (modalIsNewCard) {
      items = [...items, configuredItem];
    } else {
      const index = items.findIndex((i) => i.id === configuredItem.id);
      if (index !== -1) {
        items[index] = configuredItem;
        items = [...items];
      }
    }
    isModalOpen = false;
    modalTargetItem = null;
    saveLayout();
  }

  function deleteCard(id: string) {
    items = items.filter((item) => item.id !== id);
    saveLayout();
  }

  $effect(() => {
    if (canStore.connected) {
      canStore.startLogging();
    }
  });

  onDestroy(() => {
    canStore.stopLogging();
    cleanupResizeListeners();
  });
</script>

<svelte:window onscroll={() => (isAddMenuOpen = false)} />

<div
  class="unified-flow-dashboard"
  bind:clientWidth={containerWidth}
  class:user-is-resizing={!!resizingItemId}
>
  {#if !isInitialized}
    <div aria-busy="true">Initialising dashboard telemetry channels...</div>
  {:else if items.length === 0}
    <div class="empty-placeholder">
      <p>
        Dashboard canvas workspace is empty. Click the action button below to
        map modules.
      </p>
    </div>
  {:else}
    <div
      class="unified-grid-zone"
      use:dndzone={{
        items,
        flipDurationMs,
        dragDisabled: dragDisabled || !!resizingItemId,
        dropTargetStyle: {},
        type: "dashboard",
        /* STABLE DIMENSIONS DURING DRAG: 
           Injects grid tokens explicitly as local inline CSS custom properties 
           onto the floating clone container before it leaves the grid context. */
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
      {#each items as item (item.id)}
        {@const activeItem = previewItem?.id === item.id ? previewItem : item}
        <div
          class="unified-flow-card"
          class:is-resizing-target={resizingItemId === item.id}
          style="
            --card-w: {activeItem.w || 10}; 
            --card-h: {activeItem.h || 7};
            --resize-dx: {resizingItemId === item.id ? resizeDeltaX : 0}px;
            --resize-dy: {resizingItemId === item.id ? resizeDeltaY : 0}px;
          "
          animate:flip={{ duration: flipDurationMs }}
        >
          <div class="card-inner-hull">
            {#if item.cardType === "pid"}
              <PIDCardWrapper
                item={activeItem}
                dragDelay={DRAG_HOLD_DELAY_MS}
                popupDelay={POPUP_HOLD_DELAY_MS}
                onRequestDrag={triggerDragActivation}
                resizeStart={(e) => startResizing(e, item.id)}
                onOpenSettings={openEditSettings}
                onDelete={deleteCard}
              />
            {/if}
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
  onClose={() => {
    isModalOpen = false;
    modalTargetItem = null;
    previewItem = null;
  }}
/>

<div class="fab-container">
  {#if isAddMenuOpen}
    <div class="fab-menu-popover" onclick={(e) => e.stopPropagation()}>
      <div class="popover-section-title">New System Modules</div>
      <button
        class="fab-menu-item"
        onclick={() => openAddPresetSettings("battery")}>Battery Monitor</button
      >
      <button
        class="fab-menu-item"
        onclick={() => openAddPresetSettings("dtcs")}>DTC Trouble Log</button
      >
      <button
        class="fab-menu-item"
        onclick={() => openAddPresetSettings("overview")}
        >Performance Panel</button
      >
      <div class="popover-divider"></div>
      <button
        class="fab-menu-item setup-pid-highlight"
        onclick={() => openAddPresetSettings("pid")}
      >
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

  /* RESIZING LAYOUT STATE LOCKS */
  .user-is-resizing,
  .user-is-resizing * {
    cursor: se-resize !important;
    user-select: none !important;
    pointer-events: none !important;
  }
  .user-is-resizing .unified-grid-zone {
    pointer-events: auto !important;
  }

  /* BASE GRID AREA CANVAS CONFIGURATIONS */
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
    width: 100% !important; /* Locks grid scale smoothly when idle */
    height: calc(
      (var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)
    ) !important;
    transition: transform 0.15s ease;
  }

  /* ACTIVE LIVE RESIZING TRANSFORMS:
     Bypasses layout grid column restrictions dynamically mid-gesture 
     by appending raw calculated pixel deltas directly onto the axes. */
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

  /* MOBILE 24-COLUMN RESPONSIVE LAYOUT MATRIX */
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

  /* FLOATING ACTION INTERFACE CONTROLS */
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
    transition:
      transform 0.2s,
      background 0.2s;
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

  .unified-grid-zone:focus,
  .unified-grid-zone:focus-within,
  :global(#dnd-action-dragged-el),
  :global([class*="dndzone"]) {
    outline: none !important;
    box-shadow: none !important;
  }

  /* --- DRAGGED CLONE & PLACEHOLDER STRUCTURAL LOCKS --- */

  /* Target the active floating element currently being dragged by the user */
  :global(#dnd-action-dragged-el) {
    box-shadow: 0 20px 40px rgba(0, 0, 0, 0.6) !important;
    border-radius: 8px !important;
    opacity: 0.95 !important;

    /* FIXED HORIZONTAL WIDTH STABILITY PRESERVATION:
       Translates column spans into standalone absolute pixel values while detached.
       Formula parameters: (100% viewport width minus layout padding and gap boundaries) 
       divided by total grid column tracks, scaled by column span variables.
    */
    width: calc(
      (((100vw - 24px) - (60 - 1) * 16px) / 60 * var(--card-w)) +
        ((var(--card-w) - 1) * 16px)
    ) !important;

    /* Fixed vertical height stability preservation */
    height: calc(
      (var(--card-h) * 10px) + ((var(--card-h) - 1) * 16px)
    ) !important;
  }

  /* Modifies standalone floating drag parameters matching mobile 24-column grid rules */
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

  /* Target the underlying ghost slot layout left behind in the active grid track array */
  :global(.unified-grid-zone > div[style*="visibility: hidden"]) {
    visibility: visible !important;
    opacity: 0.15 !important;
    background: rgba(255, 255, 255, 0.02) !important;
    border: 2px dashed rgba(255, 255, 255, 0.2) !important;
    border-radius: 8px;
    width: 100% !important; /* Ghost frames inherit correctly inside the live grid stream */
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
