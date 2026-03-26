<script>
  import PIDCardWrapper from "$lib/components/PIDCardWrapper.svelte";
  import { canStore } from "./lib/canStore.svelte.js";
  import { untrack, onDestroy } from "svelte";

  function createCardFromPID(pidData) {
    return {
      pid: pidData.pid,
      label: pidData.name,
      description: pidData.description,
      unit: pidData.unit,
      icon: pidData.icon,
      color: `#${pidData.color.toString(16).padStart(6, "0")}`,
      update_interval_ms: pidData.updateIntervalMs,
      min: pidData.minValue || 0,
      max: pidData.maxValue || 100,
    };
  }

  let item = $state();

  // 2. Add a simple state to track which mode we are in
  let displayMode = $state("card");

  $effect(() => {
    const pids = canStore.pidDefinitions;
    if (pids?.length > 0) {
      untrack(() => {
        item = createCardFromPID(pids[2]);
      });
    }
  });

  // 3. A quick helper function to toggle the view
  function toggleMode() {
    displayMode = displayMode === "card" ? "chart" : "card";
  }

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });

  onDestroy(() => {
    canStore.stopLogging();
  });
</script>

<button class="outline" onclick={toggleMode} style="margin-bottom: 1rem;">
  Switch to {displayMode === "card" ? "Chart" : "Normal Card"}
</button>

<div class="pid-container">
  {#if item}
    <PIDCardWrapper {...item} {displayMode} />
  {:else}
    <p>Loading...</p>
  {/if}
</div>

<style>
  .pid-container {
    display: flex;
    gap: 1rem;
    flex-wrap: wrap;
    max-width: 500px;
    width: 500px;
    height: 300px;
  }
</style>
