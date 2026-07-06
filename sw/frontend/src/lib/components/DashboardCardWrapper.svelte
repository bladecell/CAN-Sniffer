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

  function openMenu(x: number, y: number) {
    const MENU_W = 200,
      MENU_H = 220;
    menuPos = {
      x: Math.min(Math.max(10, x), window.innerWidth - MENU_W - 10),
      y: Math.min(Math.max(10, y), window.innerHeight - MENU_H - 10),
    };
    dashboardStore.activeMenuId = item.id;
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
    onRequestDrag(); // Tells parent DND zone: "Unfreeze this item now"
  }

  function dispatchResize(e: PointerEvent) {
    if (!e.isPrimary) return;
    e.stopPropagation();
    e.preventDefault();
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
          ? "Stop Logging"
          : "Start Logging"}
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
          onDelete(item.id);
          dashboardStore.activeMenuId = null;
        }}>Remove Module</button
      >
    </div>
  {/if}

  {#if isEditing}
    <div
      class="edit-blueprint-overlay"
      transition:fade={{ duration: 100 }}
      onpointerdown={dispatchDrag}
    >
      <span class="watermark-badge">{item.cardType}</span>

      <div class="action-pill" onpointerdown={(e) => e.stopPropagation()}>
        <button class="pill-btn edit" onclick={() => onOpenSettings(item)}>
          <svg
            viewBox="0 0 24 24"
            width="16"
            height="16"
            stroke="currentColor"
            stroke-width="2"
            fill="none"
            ><path d="M12 20h9" /><path
              d="M16.5 3.5a2.121 2.121 0 0 1 3 3L7 19l-4 1 1-4L16.5 3.5z"
            /></svg
          >
          Edit
        </button>
        <div class="pill-divider"></div>
        <button class="pill-btn delete" onclick={() => onDelete(item.id)}>
          <svg
            viewBox="0 0 24 24"
            width="18"
            height="18"
            stroke="currentColor"
            stroke-width="2"
            fill="none"
            ><path d="M3 6h18" /><path
              d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"
            /></svg
          >
        </button>
      </div>

      <div class="resize-touch-target" use:resizeTouch>
        <div class="resize-visual"></div>
      </div>
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

  /* --- CREATIVE BLUEPRINT OVERLAY --- */
  .edit-blueprint-overlay {
    position: absolute;
    inset: 0;
    z-index: 999;
    /* Engineering Blueprint Grid Effect */
    background-color: color-mix(
      in srgb,
      var(--pico-card-background, #1a202b) 85%,
      transparent
    );
    background-image: linear-gradient(
        color-mix(in srgb, var(--pico-primary) 12%, transparent) 1px,
        transparent 1px
      ),
      linear-gradient(
        90deg,
        color-mix(in srgb, var(--pico-primary) 12%, transparent) 1px,
        transparent 1px
      );
    background-size: 20px 20px;
    backdrop-filter: blur(4px);
    -webkit-backdrop-filter: blur(4px);
    border: 2px dashed var(--pico-primary);
    border-radius: inherit;
    display: flex;
    align-items: center;
    justify-content: center;

    /* Entire card becomes the grab target */
    cursor: grab;
    touch-action: none;
  }
  .edit-blueprint-overlay:active {
    cursor: grabbing;
    background-color: color-mix(
      in srgb,
      var(--pico-card-background, #1a202b) 75%,
      transparent
    );
  }

  .watermark-badge {
    position: absolute;
    top: 10px;
    left: 14px;
    font-family: var(--pico-font-family-monospace);
    font-size: 0.8rem;
    font-weight: 800;
    letter-spacing: 0.1em;
    color: var(--pico-primary);
    text-transform: uppercase;
    opacity: 0.8;
  }

  /* --- FLOATING ACTION PILL --- */
  .action-pill {
    display: flex;
    align-items: center;
    background: var(--pico-card-background);
    border: 1px solid var(--pico-primary);
    border-radius: 50px;
    padding: 4px;
    box-shadow: 0 8px 24px rgba(0, 0, 0, 0.4);
    /* Restore pointer events so buttons are clickable */
    pointer-events: auto;
    cursor: default;
  }

  .pill-divider {
    width: 1px;
    height: 20px;
    background: var(--pico-muted-border-color);
    margin: 0 4px;
  }

  .pill-btn {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
    font-family: var(--pico-font-family);
    font-size: 0.85rem;
    font-weight: 700;
    color: var(--pico-color);
    background: transparent;
    border: none;
    padding: 8px 16px;
    border-radius: 40px;
    cursor: pointer;
    transition: all 0.15s ease;
  }
  .pill-btn.delete {
    padding: 8px 12px;
    color: var(--pico-muted-color);
  }

  .pill-btn:hover {
    background: color-mix(in srgb, var(--pico-primary) 15%, transparent);
    color: var(--pico-primary);
  }
  .pill-btn.delete:hover {
    background: color-mix(in srgb, orangered 15%, transparent);
    color: orangered;
  }
  .pill-btn:active {
    transform: scale(0.95);
  }

  /* --- INVISIBLE FAT-FINGER RESIZER --- */
  .resize-touch-target {
    position: absolute;
    bottom: -6px;
    right: -6px;
    width: 48px; /* Massive 48x48px hit area for sloppy mobile thumbs */
    height: 48px;
    cursor: se-resize;
    touch-action: none;
    -webkit-user-select: none;
    user-select: none;
    z-index: 10;
  }

  /* --- SLEEK VISUAL CHEVRON --- */
  .resize-visual {
    position: absolute;
    bottom: 12px;
    right: 12px;
    width: 12px;
    height: 12px;
    border-right: 3px solid var(--pico-primary);
    border-bottom: 3px solid var(--pico-primary);
    border-radius: 2px;
    opacity: 0.8;
    transition:
      transform 0.1s ease,
      opacity 0.1s ease;
  }

  .resize-touch-target:active .resize-visual {
    opacity: 1;
    transform: scale(1.2) translate(-2px, -2px);
    border-color: #fff;
    box-shadow: 2px 2px 8px
      color-mix(in srgb, var(--pico-primary) 50%, transparent);
  }
</style>
