<script lang="ts" module>
  // Global state to ensure only one menu is open across all dashboard instances
  let activeMenuId = $state<string | null>(null);
</script>

<script lang="ts">
  import { onDestroy } from "svelte";
  import PIDCard from "$lib/components/PIDCard.svelte";
  import PIDChartCard from "$lib/components/PIDChart.svelte";
  import PIDGauge from "./PIDGauge.svelte";
  import PIDBar from "./PIDBar.svelte";
  import OverviewCard from "./OverviewCard.svelte";
  import type { DashboardItem, PidGridItem } from "$lib/types";

  interface Props {
    item: DashboardItem;
    onRequestDrag: () => void;
    resizeStart: (e: any) => void;
    onOpenSettings: (item: DashboardItem) => void;
    onDelete: (id: string) => void;
  }

  let { item, onRequestDrag, resizeStart, onOpenSettings, onDelete }: Props =
    $props();

  // --- LOCAL STATE ---
  let wrapperElement = $state<HTMLElement | null>(null);
  let menuPos = $state({ x: 0, y: 0 });
  let isMenuOpen = $state(false);

  // Timers and Tracking
  let dragTimer: ReturnType<typeof setTimeout>;
  let popupTimer: ReturnType<typeof setTimeout>;
  let startCoords = { x: 0, y: 0 };
  let isTouchDevice = false;
  let isDestroyed = false;

  const MOVE_SLOP_LIMIT = 15;
  const DRAG_DELAY = 150;
  const POPUP_DELAY = 1000;

  // Sync visibility with global module state
  $effect(() => {
    if (!isDestroyed) isMenuOpen = activeMenuId === item.id;
  });

  onDestroy(() => {
    isDestroyed = true;
    clearTimers();
    if (activeMenuId === item.id) activeMenuId = null;
  });

  function triggerHaptic(duration: number | number[]) {
    if (typeof navigator !== "undefined" && navigator.vibrate) {
      navigator.vibrate(0);
      navigator.vibrate(duration);
    }
  }

  function engageHoldTracking(clientX: number, clientY: number, e: any) {
    if (isDestroyed) return;
    if (e.button && e.button !== 0) return;
    if (e.target.closest("button, select, .minimal-ui-stepper, .resize-handle"))
      return;

    isTouchDevice = e.type === "touchstart" || e.pointerType === "touch";
    if (isTouchDevice) triggerHaptic(20);

    startCoords = { x: clientX, y: clientY };
    clearTimers();

    if (!isTouchDevice) {
      onRequestDrag();
    } else {
      dragTimer = setTimeout(() => {
        if (!isDestroyed && !isMenuOpen) {
          triggerHaptic(50);
          onRequestDrag();
        }
      }, DRAG_DELAY);
    }

    popupTimer = setTimeout(() => {
      if (!isDestroyed) {
        triggerHaptic(isMenuOpen ? [30, 30, 30] : [70, 40, 70]);
        openMenu(clientX, clientY);
      }
      clearTimers();
    }, POPUP_DELAY);
  }

  function monitorMovement(clientX: number, clientY: number) {
    if (isDestroyed || !isTouchDevice) return;
    if (
      Math.abs(clientX - startCoords.x) > MOVE_SLOP_LIMIT ||
      Math.abs(clientY - startCoords.y) > MOVE_SLOP_LIMIT
    ) {
      clearTimers();
    }
  }

  function clearTimers() {
    clearTimeout(dragTimer);
    clearTimeout(popupTimer);
  }

  function openMenu(x: number, y: number) {
    const W = 220;
    const H = 110;
    let finalX = x + W > window.innerWidth ? x - W : x;
    let finalY = y + H > window.innerHeight ? window.innerHeight - H - 12 : y;

    menuPos = { x: Math.max(12, finalX), y: Math.max(12, finalY) };
    activeMenuId = item.id;
  }

  function handleClose(e: Event) {
    if (
      isMenuOpen &&
      wrapperElement &&
      !wrapperElement.contains(e.target as Node)
    ) {
      activeMenuId = null;
    }
  }
</script>

<svelte:window
  onclick={handleClose}
  ontouchstart={handleClose}
  onscroll={() => isMenuOpen && (activeMenuId = null)}
/>

<div
  bind:this={wrapperElement}
  class="dashboard-card-wrapper"
  oncontextmenu={(e) => {
    e.preventDefault();
    openMenu(e.clientX, e.clientY);
  }}
  onpointerdown={(e) => engageHoldTracking(e.clientX, e.clientY, e)}
  ontouchstart={(e) => {
    isTouchDevice = true;
    engageHoldTracking(e.touches[0].clientX, e.touches[0].clientY, e);
  }}
  onpointermove={(e) => monitorMovement(e.clientX, e.clientY)}
  onpointerup={clearTimers}
  ontouchend={clearTimers}
  onpointercancel={clearTimers}
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
    <OverviewCard />
  {:else if item.cardType === "battery"}
    <div class="static-panel-placeholder">Battery Monitor Pane</div>
  {:else if item.cardType === "dtcs"}
    <div class="static-panel-placeholder">DTC Trouble Log</div>
  {/if}

  <div
    class="resize-handle"
    onpointerdown={(e) => {
      e.stopPropagation();
      resizeStart(e);
    }}
    ontouchstart={(e) => {
      e.stopPropagation();
      popupTimer = setTimeout(() => {
        triggerHaptic(30);
        resizeStart(e);
      }, POPUP_DELAY);
    }}
  ></div>

  {#if isMenuOpen}
    <div
      class="local-context-menu"
      style="top: {menuPos.y}px; left: {menuPos.x}px;"
      onclick={(e) => e.stopPropagation()}
    >
      <button
        onclick={() => {
          onOpenSettings(item);
          activeMenuId = null;
        }}>Modify Proportions</button
      >
      <button
        class="delete-action"
        onclick={() => {
          triggerHaptic([30, 40, 30]);
          onDelete(item.id);
          activeMenuId = null;
        }}>Remove Module</button
      >
    </div>
  {/if}
</div>

<style>
  .dashboard-card-wrapper {
    width: 100%;
    height: 100%;
    position: relative;
    user-select: none;
    -webkit-user-select: none;
  }
  .resize-handle {
    position: absolute;
    bottom: 0;
    right: 0;
    width: 28px;
    height: 28px;
    cursor: se-resize;
    z-index: 100;
  }
  .resize-handle::after {
    content: "";
    position: absolute;
    bottom: 6px;
    right: 6px;
    width: 10px;
    height: 10px;
    border-right: 2px solid rgba(255, 255, 255, 0.4);
    border-bottom: 2px solid rgba(255, 255, 255, 0.4);
  }
  .static-panel-placeholder {
    display: flex;
    align-items: center;
    justify-content: center;
    width: 100%;
    height: 100%;
    color: #a0a0a5;
    font-size: 0.85rem;
    font-weight: 500;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    background: rgba(30, 30, 35, 0.4);
    border-radius: 8px;
  }
  .local-context-menu {
    position: fixed;
    z-index: 11000;
    background: rgba(20, 20, 24, 0.95);
    backdrop-filter: blur(12px);
    border: 1px solid rgba(255, 255, 255, 0.08);
    border-radius: 8px;
    padding: 6px;
    display: flex;
    flex-direction: column;
    min-width: 200px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
  }
  .local-context-menu button {
    background: transparent;
    color: #e4e4e7;
    border: none;
    text-align: left;
    padding: 10px 14px;
    border-radius: 4px;
    cursor: pointer;
    font-size: 0.9rem;
  }
  .local-context-menu button:hover {
    background: rgba(255, 255, 255, 0.06);
    color: #ffffff;
  }
  .delete-action {
    color: #f87171 !important;
  }
</style>
