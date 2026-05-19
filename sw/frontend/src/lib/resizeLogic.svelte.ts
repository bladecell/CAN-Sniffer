// src/lib/resizeLogic.svelte.ts
import { dashboardStore } from "./dashboardStore.svelte";
import { getModuleBounds } from "./types";
import type { DashboardItem } from "./types";

export interface ResizeState {
  resizingItemId: string | null;
  resizeDeltaX: number;
  resizeDeltaY: number;
  activeConfig: { minW: number; maxW: number; minH: number; maxH: number };
}

export function createResizeHandler(
  containerWidth: () => number,
  desktopCols: number,
  mobileCols: number,
  rowHeight: number,
  desktopGap: number,
  mobileGap: number
) {
  let state = $state<ResizeState>({
    resizingItemId: null,
    resizeDeltaX: 0,
    resizeDeltaY: 0,
    activeConfig: { minW: 4, maxW: 60, minH: 2, maxH: 50 }
  });

  let initialMouseX = 0;
  let initialMouseY = 0;
  let initialWidthPx = 0;
  let initialHeightPx = 0;
  let latestPointerEvent: PointerEvent | null = null;
  let animationFrameToken: number | null = null;
  let lastRenderTime = 0;
  const RENDER_INTERVAL_MS = 1000 / 30;

  function start(event: any, id: string) {
    const item = dashboardStore.items.find(i => i.id === id);
    if (!item) return;

    const bounds = getModuleBounds(item);
    if (bounds && bounds.min && bounds.max) {
      state.activeConfig = {
        minW: bounds.min.w,
        maxW: bounds.max.w,
        minH: bounds.min.h,
        maxH: bounds.max.h
      };
    } else {
      // Emergency fallback if even getModuleBounds fails
      state.activeConfig = { minW: 4, maxW: 60, minH: 2, maxH: 50 };
    }

    const clientX = event.touches ? event.touches[0].clientX : event.clientX;
    const clientY = event.touches ? event.touches[0].clientY : event.clientY;

    const targetEl = event.target as HTMLElement;
    const cardElement = targetEl.closest(".unified-flow-card");
    if (!cardElement) return;

    const rect = cardElement.getBoundingClientRect();
    initialWidthPx = rect.width;
    initialHeightPx = rect.height;
    initialMouseX = clientX;
    initialMouseY = clientY;
    state.resizingItemId = id;
    state.resizeDeltaX = 0;
    state.resizeDeltaY = 0;
    lastRenderTime = performance.now();

    if (event.touches) {
      window.addEventListener("touchmove", queueResizeUpdateTouch, { passive: false });
      window.addEventListener("touchend", stop);
    } else {
      window.addEventListener("pointermove", queueResizeUpdate);
      window.addEventListener("pointerup", stop);
    }
  }

  function queueResizeUpdateTouch(e: TouchEvent) {
    if (state.resizingItemId) e.preventDefault();
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
    if (!state.resizingItemId || !latestPointerEvent) return;

    const currentTime = performance.now();
    const timeElapsed = currentTime - lastRenderTime;

    if (timeElapsed < RENDER_INTERVAL_MS) {
      animationFrameToken = requestAnimationFrame(processThrottledResize);
      return;
    }

    lastRenderTime = currentTime - (timeElapsed % RENDER_INTERVAL_MS);

    const isMobile = window.innerWidth <= 768;
    const cols = isMobile ? mobileCols : desktopCols;
    const gap = isMobile ? mobileGap : desktopGap;

    const cellWidth = (containerWidth() - (cols - 1) * gap) / cols;

    const rawDX = latestPointerEvent.clientX - initialMouseX;
    const rawDY = latestPointerEvent.clientY - initialMouseY;

    const projectedW = Math.round((initialWidthPx + rawDX + gap) / (cellWidth + gap));
    const projectedH = Math.round((initialHeightPx + rawDY + gap) / (rowHeight + gap));

    const dynamicMaxW = Math.min(cols, state.activeConfig.maxW);

    if (projectedW < state.activeConfig.minW) {
      state.resizeDeltaX = state.activeConfig.minW * cellWidth + (state.activeConfig.minW - 1) * gap - initialWidthPx;
    } else if (projectedW > dynamicMaxW) {
      state.resizeDeltaX = dynamicMaxW * cellWidth + (dynamicMaxW - 1) * gap - initialWidthPx;
    } else {
      state.resizeDeltaX = rawDX;
    }

    if (projectedH < state.activeConfig.minH) {
      state.resizeDeltaY = state.activeConfig.minH * rowHeight + (state.activeConfig.minH - 1) * gap - initialHeightPx;
    } else if (projectedH > state.activeConfig.maxH) {
      state.resizeDeltaY = state.activeConfig.maxH * rowHeight + (state.activeConfig.maxH - 1) * gap - initialHeightPx;
    } else {
      state.resizeDeltaY = rawDY;
    }
  }

  function stop() {
    if (state.resizingItemId && containerWidth() > 0) {
      const item = dashboardStore.items.find(i => i.id === state.resizingItemId);
      if (item) {
        const isMobile = window.innerWidth <= 768;
        const cols = isMobile ? mobileCols : desktopCols;
        const gap = isMobile ? mobileGap : desktopGap;
        const cellWidth = (containerWidth() - (cols - 1) * gap) / cols;

        const finalW = Math.round((initialWidthPx + state.resizeDeltaX + gap) / (cellWidth + gap));
        const finalH = Math.round((initialHeightPx + state.resizeDeltaY + gap) / (rowHeight + gap));

        const dynamicMaxW = Math.min(cols, state.activeConfig.maxW);
        item.w = Math.max(state.activeConfig.minW, Math.min(dynamicMaxW, finalW));
        item.h = Math.max(state.activeConfig.minH, Math.min(state.activeConfig.maxH, finalH));
        dashboardStore.updateItem(item);
      }
    }
    cleanup();
  }

  function cleanup() {
    state.resizingItemId = null;
    state.resizeDeltaX = 0;
    state.resizeDeltaY = 0;
    latestPointerEvent = null;
    if (animationFrameToken) cancelAnimationFrame(animationFrameToken);
    animationFrameToken = null;
    window.removeEventListener("pointermove", queueResizeUpdate);
    window.removeEventListener("pointerup", stop);
    window.removeEventListener("touchmove", queueResizeUpdateTouch);
    window.removeEventListener("touchend", stop);
  }

  return {
    state,
    start
  };
}
