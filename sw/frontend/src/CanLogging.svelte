<script>
  import DashboardCardWrapper from "$lib/components/DashboardCardWrapper.svelte";
  import { canStore } from "./lib/canStore.svelte";
  import { untrack, onDestroy, onMount } from "svelte";
  import OverviewCard from "$lib/components/OverviewCard.svelte";
  import DTCFault from "./lib/components/DTCFault.svelte";

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

  onMount(async () => {
    await canStore.getDTC();
  });

  let item = $state();
  let cards = ["card", "chart", "gauge", "bar"];
  let cardIndex = $state(0);

  // 2. Add a simple state to track which mode we are in
  let displayMode = $state("card");

  $effect(() => {
    const pids = canStore.pidDefinitions;
    if (pids?.length > 0) {
      untrack(() => {
        item = createCardFromPID(pids[0]);
      });
    }
  });

  // 3. A quick helper function to toggle the view
  function toggleMode() {
    if (cardIndex >= cards.length - 1) {
      cardIndex = 0;
    } else {
      cardIndex += 1;
    }
    displayMode = cards[cardIndex];
  }

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });

  onDestroy(() => {
    canStore.stopLogging();
  });

  const pidData1 = $derived(canStore.pids.get(4));
  const currentValue1 = $derived(pidData1?.value ?? 0);
  const timestamp1 = $derived(pidData1?.timestamp ?? 0);

  let previousTimestamp1 = $state(0);

  $effect(() => {
    const t = timestamp1;
    const v = currentValue1;

    untrack(() => {
      if (t > 0 && previousTimestamp1 > 0) {
        const delta = t - previousTimestamp1;

        if (delta > 0) {
          console.log(`Update for PID 4: ${delta}ms, value: ${v}`);
        }
      }

      previousTimestamp1 = t;
    });
  });

  let activeFaults = [
    {
      code: "P0420",
      description: "Catalyst efficiency below threshold (Bank 1)",
      mode: 3,
    },
  ];
</script>

<div class="pid-container">
  {#each activeFaults as fault}
    <DTCFault
      code={fault.code}
      description={fault.description}
      mode={fault.mode}
    />
  {/each}
</div>

<style>
  .pid-container {
    display: flex;
    gap: 1rem;
    flex-wrap: wrap;
    max-width: 500px;
    width: 400px;
    height: 200px;
  }
</style>
