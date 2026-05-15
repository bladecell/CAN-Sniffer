<script lang="ts">
  import PIDCard from "$lib/components/PIDCard.svelte";
  import PIDChartCard from "$lib/components/PIDChart.svelte";
  import PIDGauge from "./PIDGauge.svelte";
  import PIDBar from "./PIDBar.svelte";

  type DisplayMode = "card" | "chart" | "gauge" | "bar";

  interface Props {
    displayMode?: DisplayMode;
    moveStart?: ((e: PointerEvent) => void) | null;
    resizeStart?: ((e: PointerEvent) => void) | null;
    [key: string]: unknown;
  }

  let {
    displayMode = "card",
    moveStart = null,
    resizeStart = null,
    ...rest
  }: Props = $props();

  let longPressTimer: ReturnType<typeof setTimeout>;
  let isHolding = $state(false);

  function handlePointerDown(e: PointerEvent) {
    longPressTimer = setTimeout(() => {
      isHolding = true;
      moveStart?.(e);
    }, 200);
  }

  function handlePointerUp() {
    clearTimeout(longPressTimer);
    isHolding = false;
  }

  function handleResizeDown(e: PointerEvent) {
    clearTimeout(longPressTimer);
    isHolding = false;
    resizeStart?.(e);
  }
</script>

<!-- svelte-ignore a11y_no_static_element_interactions -->
<div
  class="pid-card-wrapper"
  onpointerdown={moveStart ? handlePointerDown : null}
  onpointerup={handlePointerUp}
  onpointermove={handlePointerUp}
  style="cursor: {isHolding ? 'grab' : 'default'}; touch-action: pan-y;"
>
  {#if displayMode === "chart"}
    <PIDChartCard {...rest} />
  {:else if displayMode === "gauge"}
    <PIDGauge {...rest} />
  {:else if displayMode === "bar"}
    <PIDBar {...rest} />
  {:else}
    <PIDCard {...rest} />
  {/if}

  {#if resizeStart}
    <div class="resize-handle" onpointerdown={handleResizeDown}></div>
  {/if}
</div>

<style>
  .pid-card-wrapper {
    width: 100%;
    height: 100%;
    position: relative;
  }

  .resize-handle {
    position: absolute;
    bottom: 0;
    right: 0;
    width: 18px;
    height: 18px;
    cursor: se-resize;
    background: transparent;
  }

  .resize-handle::after {
    content: "";
    position: absolute;
    bottom: 4px;
    right: 4px;
    width: 8px;
    height: 8px;
    border-right: 2px solid rgba(255, 255, 255, 0.3);
    border-bottom: 2px solid rgba(255, 255, 255, 0.3);
  }
</style>
