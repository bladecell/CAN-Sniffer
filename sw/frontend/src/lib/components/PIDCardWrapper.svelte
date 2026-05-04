<script>
  import PIDCard from "$lib/components/PIDCard.svelte";
  import PIDChartCard from "$lib/components/PIDChart.svelte";
  import PIDGauge from "./PIDGauge.svelte";
  import PIDBar from "./PIDBar.svelte";

  // Destructure displayMode, collect everything else into 'rest'
  let { displayMode = "card", ...rest } = $props();
</script>

<div class="pid-card-wrapper">
  {#if displayMode === "chart"}
    <PIDChartCard {...rest} />
  {:else if displayMode === "gauge"}
    <PIDGauge {...rest} />
  {:else if displayMode === "bar"}
    <PIDBar {...rest} />
  {:else}
    <PIDCard {...rest} />
  {/if}
</div>

<style>
  .pid-card-wrapper {
    width: 100%;
    height: 100%;
  }
</style>
