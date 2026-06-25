import { dashboardStore } from "./dashboardStore.svelte.ts";
import { getModuleBounds } from "./types";

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

  function start(event: PointerEvent, id: string) {
    const item = dashboardStore.items.find(i => i.id === id);
    if (!item) return;

    // Set bounds
    const bounds = getModuleBounds(item);
    state.activeConfig = {
      minW: bounds.min.w, maxW: bounds.max.w,
      minH: bounds.min.h, maxH: bounds.max.h
    };

    const cardElement = (event.target as HTMLElement).closest(".unified-flow-card");
    if (!cardElement) return;

    const rect = cardElement.getBoundingClientRect();
    initialWidthPx = rect.width;
    initialHeightPx = rect.height;
    initialMouseX = event.clientX;
    initialMouseY = event.clientY;
    
    state.resizingItemId = id;
    state.resizeDeltaX = 0;
    state.resizeDeltaY = 0;

    // Use Pointer Capture to ensure the resize handle keeps the event 
    // even if the user's thumb drifts outside the visual box
    (event.target as HTMLElement).setPointerCapture(event.pointerId);

    window.addEventListener("pointermove", handlePointerMove, { passive: false });
    window.addEventListener("pointerup", stop);
  }

  function handlePointerMove(e: PointerEvent) {
    if (!state.resizingItemId) return;
    e.preventDefault();
    const isMobile = containerWidth() <= 768;
    const gap = isMobile ? mobileGap : desktopGap;
    const padding = 12;
    const cellW = 20;
    const cellH = 10;

    const availableWidth = containerWidth() - (2 * padding);
    const dynamicCols = Math.max(1, Math.floor((availableWidth + gap) / (cellW + gap)));

    const rawDX = e.clientX - initialMouseX;
    const rawDY = e.clientY - initialMouseY;

    // Math: Convert pixels back to grid units
    const projectedW = Math.round((initialWidthPx + rawDX + gap) / (cellW + gap));
    const projectedH = Math.round((initialHeightPx + rawDY + gap) / (cellH + gap));

    const currentMaxW = Math.min(dynamicCols, state.activeConfig.maxW);

    // Apply Constraints
    state.resizeDeltaX = Math.max(
      state.activeConfig.minW * cellW + (state.activeConfig.minW - 1) * gap - initialWidthPx,
      Math.min(currentMaxW * cellW + (currentMaxW - 1) * gap - initialWidthPx, rawDX)
    );

    state.resizeDeltaY = Math.max(
      state.activeConfig.minH * cellH + (state.activeConfig.minH - 1) * gap - initialHeightPx,
      Math.min(state.activeConfig.maxH * cellH + (state.activeConfig.maxH - 1) * gap - initialHeightPx, rawDY)
    );
  }

function stop(e: PointerEvent) {
  if (state.resizingItemId) {
    const item = dashboardStore.items.find(i => i.id === state.resizingItemId);
    if (item) {
      const isMobile = containerWidth() <= 768; // also use containerWidth() here for consistency
      const gap = isMobile ? mobileGap : desktopGap;
      const cellW = 20;
      const cellH = 10;

      item.w = Math.round((initialWidthPx + state.resizeDeltaX + gap) / (cellW + gap));
      item.h = Math.round((initialHeightPx + state.resizeDeltaY + gap) / (cellH + gap));
      dashboardStore.updateItem(item);
    }
  }

  state.resizingItemId = null;
  window.removeEventListener("pointermove", handlePointerMove); // ← was addEventListener
  window.removeEventListener("pointerup", stop);
}

  return {
    get state() { return state; },
    start
  };
}