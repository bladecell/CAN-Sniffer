<script lang="ts" module>
  // Svelte 5 module state: ensures only one menu is open across all dashboard instances
  let activeMenuId = $state<string | null>(null);
</script>

<script lang="ts">
  import PIDCard from "$lib/components/PIDCard.svelte";
  import PIDChartCard from "$lib/components/PIDChart.svelte";
  import PIDGauge from "./PIDGauge.svelte";
  import PIDBar from "./PIDBar.svelte";
  import type { PidGridItem } from "$lib/types";
  import { onDestroy } from "svelte";

  interface Props {
    item: PidGridItem;
    dragDelay: number;
    popupDelay: number;
    onRequestDrag: () => void;
    resizeStart?: ((e: any) => void) | null;
    onOpenSettings?: (item: PidGridItem) => void;
    onDelete?: (id: string) => void;
  }

  let {
    item = {} as PidGridItem, // SAFETY FIX: Provide empty object fallback shape to prevent destructure exceptions
    dragDelay,
    popupDelay,
    onRequestDrag,
    resizeStart = null,
    onOpenSettings,
    onDelete,
  }: Props = $props();

  let wrapperElement = $state<HTMLElement | null>(null);

  // Local state for coordinates
  let menuPos = $state({ x: 0, y: 0 });

  // FIXED: Converted from a global $derived expression to local state.
  // This completely eliminates "derived_inert" stale memory errors when elements unmount.
  let isMenuOpen = $state(false);

  let dragTimer: ReturnType<typeof setTimeout>;
  let popupTimer: ReturnType<typeof setTimeout>;
  let startCoords = { x: 0, y: 0 };
  let isTouchDevice = false;
  const MOVE_SLOP_LIMIT = 15; // Increased for better mobile stability

  // Lifecycle Tracker: Prevents timers and coordinate updates from executing on orphaned objects mid-drag
  let isDestroyed = false;

  onDestroy(() => {
    isDestroyed = true;
    clearAllTimers();
    // Cleanly yield the global menu lock if this card is deleted or re-rendered
    if (activeMenuId === item?.id) {
      activeMenuId = null;
    }
  });

  // Keep local visibility in perfect sync with the global module state via an effect
  $effect(() => {
    if (isDestroyed || !item?.id) return;
    isMenuOpen = activeMenuId === item.id;
  });

  function triggerHaptic(duration: number | number[]) {
    if (typeof navigator !== "undefined" && navigator.vibrate) {
      navigator.vibrate(0);
      navigator.vibrate(duration);
    }
  }

  function engageHoldTracking(
    clientX: number,
    clientY: number,
    originalEvent: any,
  ) {
    if (isDestroyed || !item || !item.id) return; // Prevent tracking on incomplete or dead structures
    if (originalEvent.button && originalEvent.button !== 0) return;

    const targetPath = originalEvent.target as HTMLElement;
    if (
      targetPath.closest("button, select, .minimal-ui-stepper, .resize-handle")
    )
      return;

    isTouchDevice =
      originalEvent.type === "touchstart" ||
      originalEvent.pointerType === "touch";

    if (isTouchDevice) {
      triggerHaptic(20); // Immediate feedback to prime the engine
    }

    startCoords = { x: clientX, y: clientY };
    clearAllTimers();

    if (!isTouchDevice) {
      onRequestDrag();
    } else {
      dragTimer = setTimeout(() => {
        if (isDestroyed) return;
        if (!isMenuOpen) triggerHaptic(50);
        onRequestDrag();
      }, dragDelay);
    }

    popupTimer = setTimeout(() => {
      if (isDestroyed) return;
      triggerHaptic(isMenuOpen ? [30, 30, 30] : [70, 40, 70]);
      // FIX: Secure pointer track coordinates safely BEFORE updating visibility flags
      triggerMenu(clientX, clientY);
      clearAllTimers();
    }, popupDelay);
  }

  function monitorMovement(clientX: number, clientY: number) {
    if (isDestroyed || !isTouchDevice) return;

    const totalTravelX = Math.abs(clientX - startCoords.x);
    const totalTravelY = Math.abs(clientY - startCoords.y);

    if (totalTravelX > MOVE_SLOP_LIMIT || totalTravelY > MOVE_SLOP_LIMIT) {
      clearAllTimers();
    }
  }

  function clearAllTimers() {
    clearTimeout(dragTimer);
    clearTimeout(popupTimer);
  }

  function handleContextMenu(e: MouseEvent) {
    if (isDestroyed || !item || !item.id) return;
    e.preventDefault();
    e.stopPropagation();
    triggerMenu(e.clientX, e.clientY);
  }

  function triggerMenu(x: number, y: number) {
    if (isDestroyed || !item || !item.id) return;

    if (isMenuOpen) {
      activeMenuId = null;
      isMenuOpen = false;
    } else {
      const MENU_WIDTH = 220;
      const MENU_HEIGHT = 110;

      let finalX = x;
      let finalY = y;

      if (x + MENU_WIDTH > window.innerWidth) {
        finalX = x - MENU_WIDTH;
      }

      if (y + MENU_HEIGHT > window.innerHeight) {
        finalY = window.innerHeight - MENU_HEIGHT - 12;
      }

      // Lock positions local to actual user pointer parameters
      menuPos.x = Math.max(12, finalX);
      menuPos.y = Math.max(12, finalY);

      activeMenuId = item.id;
      isMenuOpen = true;
    }
  }

  function handleWindowCloseEvent(e: Event) {
    if (isDestroyed) return;
    if (
      isMenuOpen &&
      wrapperElement &&
      !wrapperElement.contains(e.target as Node)
    ) {
      activeMenuId = null;
      isMenuOpen = false;
    }
  }
</script>

<svelte:window
  onclick={handleWindowCloseEvent}
  onscroll={() => {
    if (isMenuOpen) activeMenuId = null;
  }}
  ontouchstart={handleWindowCloseEvent}
/>

<div
  bind:this={wrapperElement}
  class="pid-card-wrapper"
  oncontextmenu={handleContextMenu}
  onpointerdown={(e) => engageHoldTracking(e.clientX, e.clientY, e)}
  ontouchstart={(e) => {
    isTouchDevice = true;
    const touch = e.touches[0];
    engageHoldTracking(touch.clientX, touch.clientY, e);
  }}
  onpointermove={(e) => monitorMovement(e.clientX, e.clientY)}
  onpointerup={clearAllTimers}
  ontouchend={clearAllTimers}
  onpointercancel={clearAllTimers}
>
  {#if item && item.cardType}
    {#if item.cardType === "pid"}
      {#if item.displayMode === "chart"}
        <PIDChartCard {item} />
      {:else if item.displayMode === "gauge"}
        <PIDGauge {item} />
      {:else if item.displayMode === "bar"}
        <PIDBar {item} />
      {:else}
        <PIDCard {item} />
      {/if}
    {:else if item.cardType === "battery"}
      <div class="static-panel-placeholder">Battery Monitor Pane</div>
    {:else if item.cardType === "dtcs"}
      <div class="static-panel-placeholder">DTC Trouble Log</div>
    {:else if item.cardType === "overview"}
      <div class="static-panel-placeholder">Performance Panel</div>
    {/if}
  {:else}
    <div class="card-loading-shimmer">Initializing layout channel...</div>
  {/if}

  {#if resizeStart && item && item.id}
    <div
      class="resize-handle"
      onpointerdown={(e) => {
        e.stopPropagation();
        resizeStart?.(e);
      }}
      ontouchstart={(e) => {
        e.stopPropagation();
        popupTimer = setTimeout(() => {
          triggerHaptic(30);
          resizeStart?.(e);
        }, popupDelay);
      }}
      ontouchend={() => clearTimeout(popupTimer)}
      ontouchmove={() => clearTimeout(popupTimer)}
      title="Drag to resize"
    ></div>
  {/if}

  {#if isMenuOpen && item && item.id}
    <div
      class="local-context-menu"
      style="top: {menuPos.y}px; left: {menuPos.x}px;"
      onclick={(e) => e.stopPropagation()}
      ontouchstart={(e) => e.stopPropagation()}
    >
      <button
        onclick={() => {
          onOpenSettings?.(item);
          activeMenuId = null;
          isMenuOpen = false;
        }}
      >
        Modify Component Proportions
      </button>
      <button
        class="delete-action"
        onclick={() => {
          triggerHaptic([30, 40, 30]);
          onDelete?.(item.id);
          activeMenuId = null;
          isMenuOpen = false;
        }}
      >
        Remove from Dashboard
      </button>
    </div>
  {/if}
</div>

<style>
  .pid-card-wrapper {
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
    background: transparent;
    z-index: 100;
    touch-action: none !important;
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
  }
  .card-loading-shimmer {
    display: flex;
    align-items: center;
    justify-content: center;
    width: 100%;
    height: 100%;
    font-size: 0.8rem;
    color: rgba(255, 255, 255, 0.3);
    font-style: italic;
  }
  .local-context-menu {
    position: fixed;
    z-index: 11000;
    background: rgba(20, 20, 24, 0.95);
    backdrop-filter: blur(12px);
    border: 1px solid rgba(255, 255, 255, 0.08);
    border-radius: 8px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
    padding: 6px;
    display: flex;
    flex-direction: column;
    gap: 4px;
    min-width: 210px;
  }
  .local-context-menu button {
    background: transparent;
    color: #e4e4e7;
    border: none;
    text-align: left;
    padding: 12px 14px;
    border-radius: 4px;
    cursor: pointer;
    font-size: 0.9rem;
    font-weight: 500;
  }
  .local-context-menu button:hover {
    background: rgba(255, 255, 255, 0.06);
    color: #ffffff;
  }
  .local-context-menu button.delete-action {
    color: #f87171;
  }
</style>
