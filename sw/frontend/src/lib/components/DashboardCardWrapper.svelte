<script lang="ts">
  import { onDestroy } from "svelte";
  import { fade } from "svelte/transition";

  import PIDCard from "$lib/components/PIDCard.svelte";
  import PIDChartCard from "$lib/components/PIDChart.svelte";
  import PIDGauge from "./PIDGauge.svelte";
  import PIDBar from "./PIDBar.svelte";
  import OverviewCard from "./OverviewCard.svelte";
  import DTCCard from "./DTCCard.svelte";

  import type {
    DashboardItem,
    PidGridItem,
    OverviewGridItem,
    SpecialGridItem,
  } from "$lib/types";
  import { dashboardStore } from "$lib/dashboardStore.svelte.ts";
  import { canStore } from "$lib/canStore.svelte";

  interface Props {
    item: DashboardItem;
    onRequestDrag: () => void;
    resizeStart: (e: PointerEvent) => void;
    onOpenSettings: (item: DashboardItem) => void;
    onAddNew: () => void;
    onDelete: (id: string) => void;
  }

  let { item, onRequestDrag, resizeStart, onOpenSettings, onAddNew, onDelete } =
    $props();

  let menuPos = $state({ x: 0, y: 0 });

  const isMenuOpen = $derived(dashboardStore.activeMenuId === item.id);
  const isEditing = $derived(dashboardStore.isEditMode);

  function triggerHaptic(ms: number | number[] = 25) {
    if ("vibrate" in navigator) navigator.vibrate(ms);
  }

  function openMenu(x: number, y: number) {
    const MENU_W = 200,
      MENU_H = 220;
    menuPos = {
      x: Math.min(Math.max(10, x), window.innerWidth - MENU_W - 10),
      y: Math.min(Math.max(10, y), window.innerHeight - MENU_H - 10),
    };
    dashboardStore.activeMenuId = item.id;
    triggerHaptic(30);
  }

  // --- SAFE PC RIGHT-CLICK ---
  function handlePcContext(e: MouseEvent) {
    if (isEditing) return;
    e.preventDefault();
    e.stopPropagation(); // Kills background canvas menu trigger
    openMenu(e.clientX, e.clientY);
  }

  function resizeTouch(node: HTMLElement) {
    const onTouchStart = (e: TouchEvent) => {
      e.preventDefault(); // intercepts BEFORE browser scroll decision
      e.stopPropagation();
    };

    const onPointerDown = (e: PointerEvent) => {
      dispatchResize(e);
    };

    // non-passive touchstart is the only way to reliably block scroll intent on iOS
    node.addEventListener("touchstart", onTouchStart, { passive: false });
    node.addEventListener("pointerdown", onPointerDown);

    return {
      destroy() {
        node.removeEventListener("touchstart", onTouchStart);
        node.removeEventListener("pointerdown", onPointerDown);
      },
    };
  }

  // --- VANILLA MOBILE LONG-PRESS ACTION ---
  function mobileLongPress(node: HTMLElement, duration = 400) {
    let timer: ReturnType<typeof setTimeout>;
    let startX = 0,
      startY = 0;

    const handleDown = (e: PointerEvent) => {
      // Ignore PC mouse (handled by contextmenu) and ignore if Edit mode is active
      if (dashboardStore.isEditMode || e.pointerType === "mouse") return;
      startX = e.clientX;
      startY = e.clientY;

      timer = setTimeout(() => {
        openMenu(startX, startY);
      }, duration);
    };

    const handleMove = (e: PointerEvent) => {
      // If finger drifts more than 10px, they are scrolling the page; cancel stopwatch
      if (
        Math.abs(e.clientX - startX) > 10 ||
        Math.abs(e.clientY - startY) > 10
      )
        clearTimeout(timer);
    };

    const handleUp = () => clearTimeout(timer);

    node.addEventListener("pointerdown", handleDown);
    node.addEventListener("pointermove", handleMove);
    node.addEventListener("pointerup", handleUp);
    node.addEventListener("pointercancel", handleUp);

    return {
      destroy() {
        node.removeEventListener("pointerdown", handleDown);
        node.removeEventListener("pointermove", handleMove);
        node.removeEventListener("pointerup", handleUp);
        node.removeEventListener("pointercancel", handleUp);
      },
    };
  }

  function dispatchDrag(e: PointerEvent) {
    if (!e.isPrimary) return;
    e.stopPropagation();
    triggerHaptic(20);
    onRequestDrag(); // Tells parent DND zone: "Unfreeze this item now"
  }

  function dispatchResize(e: PointerEvent) {
    if (!e.isPrimary) return;
    e.stopPropagation();
    e.preventDefault();
    triggerHaptic([20, 30]);
    resizeStart(e);
  }
</script>

<div
  class="dashboard-card-wrapper"
  oncontextmenu={handlePcContext}
  use:mobileLongPress
>
  <div
    class="card-payload-hull"
    style:pointer-events={isEditing ? "none" : "auto"}
  >
    {#if item.cardType === "pid"}
      {@const pidItem = item as PidGridItem}
      {#if pidItem.displayMode === "chart"}
        <PIDChartCard item={pidItem} />
      {:else if pidItem.displayMode === "gauge"}
        <PIDGauge item={pidItem} />
      {:else if pidItem.displayMode === "bar"}
        <PIDBar item={pidItem} />
      {:else}
        <PIDCard item={pidItem} />
      {/if}
    {:else if item.cardType === "overview"}
      <OverviewCard item={item as OverviewGridItem} />
    {:else if item.cardType === "battery"}
      <div class="static-panel-placeholder">Battery Monitor</div>
    {:else if item.cardType === "dtcs"}
      <DTCCard item={item as SpecialGridItem} />
    {/if}
  </div>

  {#if isMenuOpen && !isEditing}
    <div
      class="local-context-menu"
      style="top: {menuPos.y}px; left: {menuPos.x}px;"
    >
      <button
        onclick={() => {
          onOpenSettings(item);
          dashboardStore.activeMenuId = null;
        }}>Modify Component</button
      >
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
          ? "Stop Polling"
          : "Start Polling"}
      </button>
      <button
        class="add-action"
        onclick={() => {
          onAddNew();
          dashboardStore.activeMenuId = null;
        }}>Add New Module</button
      >
      <button
        class="edit-action"
        onclick={() => {
          dashboardStore.isEditMode = true;
          dashboardStore.activeMenuId = null;
        }}>Customize Layout</button
      >
      <button
        class="delete-action"
        onclick={() => {
          triggerHaptic([30, 40]);
          onDelete(item.id);
          dashboardStore.activeMenuId = null;
        }}>Remove Module</button
      >
    </div>
  {/if}

  {#if isEditing}
    <div class="edit-blueprint-overlay" transition:fade={{ duration: 100 }}>
      <div class="drag-handle-bar" onpointerdown={dispatchDrag}>
        <div class="pips"></div>
        <span>{item.cardType}</span>
        <div class="pips"></div>
      </div>

      <div class="blueprint-body" onpointerdown={(e) => e.stopPropagation()}>
        <div class="quick-btn-tray">
          <button class="pico-micro-btn" onclick={() => onOpenSettings(item)}
            >Edit</button
          >
          <button class="pico-micro-btn del" onclick={() => onDelete(item.id)}
            >Delete</button
          >
        </div>
      </div>

      <button class="grid-item-resizer" use:resizeTouch></button>
    </div>
  {/if}
</div>

<style>
  .dashboard-card-wrapper {
    position: relative;
    width: 100%;
    height: 100%;
    user-select: none;
    -webkit-user-select: none;
    -webkit-touch-callout: none;
    border-radius: inherit;
    overflow: hidden;
    display: flex;
    flex-direction: column;
  }

  .card-payload-hull {
    width: 100%;
    height: 100%;
    flex: 1 1 auto;
    overflow: hidden;
    display: flex;
    flex-direction: column;
  }

  .static-panel-placeholder {
    display: flex;
    align-items: center;
    justify-content: center;
    flex: 1;
    color: var(--pico-muted-color);
    font-size: 0.85rem;
    font-weight: 500;
    background: color-mix(
      in srgb,
      var(--pico-card-background) 60%,
      transparent
    );
  }

  .grid-item-resizer {
    touch-action: none; /* tells browser: don't scroll from this element */
  }

  /* --- PICO MENU --- */
  .local-context-menu {
    position: fixed;
    z-index: 11000;
    background: var(--pico-card-background);
    border: 1px solid var(--pico-card-border-color);
    border-radius: var(--pico-border-radius);
    box-shadow: var(--pico-card-box-shadow);
    backdrop-filter: blur(16px);
    -webkit-backdrop-filter: blur(16px);
    padding: 6px;
    display: flex;
    flex-direction: column;
    min-width: 190px;
  }
  .local-context-menu button {
    font-family: var(--pico-font-family);
    font-size: 0.85rem;
    font-weight: 500;
    color: var(--pico-color);
    background: transparent;
    border: none;
    padding: 10px 14px;
    border-radius: calc(var(--pico-border-radius) / 2);
    text-align: left;
    cursor: pointer;
  }
  .local-context-menu button:hover {
    background-color: var(--pico-form-element-background);
    color: var(--pico-primary);
  }

  .add-action,
  .start-action {
    color: var(--module-accent, #10b981) !important;
  }
  .stop-action {
    color: orangered !important;
  }
  .edit-action {
    color: var(--pico-primary) !important;
    font-weight: 700;
  }
  .delete-action {
    color: var(--pico-del-color, #ef4444) !important;
  }

  /* --- BLUEPRINT OVERLAY --- */
  .edit-blueprint-overlay {
    position: absolute;
    inset: 0;
    z-index: 999;
    background: color-mix(
      in srgb,
      var(--pico-card-background, #1a202b) 88%,
      transparent
    );
    backdrop-filter: blur(3px);
    -webkit-backdrop-filter: blur(3px);
    display: flex;
    flex-direction: column;
    border: 2px dashed var(--pico-primary);
    border-radius: inherit;
    touch-action: none;
  }

  .drag-handle-bar {
    height: 25%;
    min-height: 34px;
    background: color-mix(in srgb, var(--pico-primary) 15%, transparent);
    border-bottom: 1px solid
      color-mix(in srgb, var(--pico-primary) 30%, transparent);
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 0 14px;
    cursor: grab;
    touch-action: none !important;
  }
  .drag-handle-bar:active {
    cursor: grabbing;
    background: color-mix(in srgb, var(--pico-primary) 25%, transparent);
  }

  .blueprint-body {
    flex-grow: 1;
    display: flex;
    align-items: center;
    justify-content: center;
    touch-action: none; /* was pan-y — kills scroll competition */
  }

  .grid-item-resizer {
    position: absolute;
    bottom: 0;
    right: 0;
    width: 36px;
    height: 36px;
    background: var(--pico-primary);
    border: none;
    border-radius: 6px 0 6px 0;
    cursor: se-resize;
    touch-action: none; /* critical */
    -webkit-user-select: none;
    user-select: none;
    z-index: 10;
  }

  .pips {
    width: 22px;
    height: 3px;
    border-top: 1px solid var(--pico-primary);
    border-bottom: 1px solid var(--pico-primary);
    opacity: 0.4;
  }
  .drag-handle-bar span {
    font-family: var(--pico-font-family-monospace);
    font-size: 0.75rem;
    font-weight: 800;
    color: var(--pico-primary);
    text-transform: uppercase;
  }

  .blueprint-body {
    flex-grow: 1;
    display: flex;
    align-items: center;
    justify-content: center;
    touch-action: none; /* was pan-y — must be none so DND owns the touch stream */
  }

  .quick-btn-tray {
    display: flex;
    gap: 10px;
  }
  .pico-micro-btn {
    font-family: var(--pico-font-family);
    font-size: 0.8rem;
    font-weight: 700;
    padding: 6px 14px;
    border-radius: var(--pico-border-radius);
    background: var(--pico-card-background);
    border: 1px solid var(--pico-border-color);
    color: var(--pico-color);
    cursor: pointer;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
  }
  .pico-micro-btn:active {
    transform: scale(0.95);
  }
  .pico-micro-btn:hover {
    border-color: var(--pico-primary);
    color: var(--pico-primary);
  }
  .pico-micro-btn.del:hover {
    border-color: orangered;
    color: orangered;
  }
</style>
