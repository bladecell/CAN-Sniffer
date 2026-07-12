<script lang="ts">
  import DashboardCardWrapper from "$lib/components/DashboardCardWrapper.svelte";
  import { canStore } from "./lib/canStore.svelte";
  import { untrack, onDestroy, onMount } from "svelte";
  import ControlsCard from "$lib/components/ControlsCard.svelte";
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

  import DataTable from "./lib/components/DataTable.svelte";
  import type { Column } from "$lib/types";

  const columns: Column[] = [
    { label: "Module", key: "name", type: "text", width: "200px" },
    { label: "Age", key: "age", type: "number", width: "60px" },
    {
      label: "Voltage",
      key: "voltage",
      type: "number",
      unit: "V",
      width: "90px",
    },
    { label: "DTC", key: "dtc", type: "code", width: "100px" },
    { label: "Status", key: "status", type: "badge", width: "100px" },
    { key: "isActive", label: "Active Tab", type: "checkbox" },
  ];

  const testData = [
    {
      name: "Engine Control",
      age: 12,
      voltage: 12.4,
      dtc: "P0300",
      status: "Active",
    },
    {
      name: "Body Module",
      age: 4,
      voltage: 12.1,
      dtc: "B0260",
      status: "Idle",
    },
    {
      name: "Transmission",
      age: 22,
      voltage: 12.6,
      dtc: "P0700",
      status: "Warning",
    },
    { name: "ABS Unit", age: 8, voltage: 12.3, dtc: "C0234", status: "Active" },
    {
      name: "Airbag Module",
      age: 45,
      voltage: 12.5,
      dtc: "B1001",
      status: "Idle",
    },
  ];

  let activeIndex = $state(-1);
  $effect(() => {
    console.log("Active index changed to:", activeIndex);
  });
</script>

<div class="pid-container">
  <ControlsCard />
</div>

<div class="data-grid-container overflow-auto">
  <DataTable {columns} data={testData} bind:selectedRowIndex={activeIndex} />
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

  .data-grid-container {
    padding: 0;
    margin: 0;
  }
</style>
