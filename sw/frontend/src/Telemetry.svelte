<script lang="ts">
  import { canStore } from "./lib/canStore.svelte";
  import { untrack, onDestroy, onMount } from "svelte";
  import type { DataTableProps, PidValue, PidDefinition } from "$lib/types";
  import DataTable from "./lib/components/DataTable.svelte";
  import type { Column } from "$lib/types";

  let activeIndex = $state(-1);
  $effect(() => {
    console.log("Active index changed to:", activeIndex);
  });

  let piddef = $derived(canStore.pidDefinitions);
  let pidData = $derived(canStore.pids);

  const columns: Column[] = [
    {
      label: "Name",
      key: "name",
      type: "text",
      width: "10%",
      tooltipKey: "moduleDescription",
    },
    { label: "PID", key: "pid", type: "code", width: "10%" },
    {
      label: "Value",
      key: "value",
      type: "number",
      unitKey: "metricUnit",
      width: "10%",
    },
    {
      label: "Update Interval",
      key: "updateInterval",
      type: "number",
      unit: "ms",
      width: "10%",
    },
    {
      label: "Supported",
      key: "supported",
      type: "badge",
      width: "10%",
      colorKey: "badgeColor",
    },
  ];

  let tableData = $derived(
    (piddef || []).map((def) => {
      // Instantly grab the live data for this specific row from the Map
      // Wrapping def.pid in Number() just in case the array stores it as a string ("0x01")
      const data = pidData?.get(Number(def.pid));

      const formattedPid =
        "0x" + Number(def.pid).toString(16).toUpperCase().padStart(2, "0");

      return {
        // --- 1. Static Data (From the Array) ---
        name: def.name, // Matches key: "name"
        moduleDescription: def.description, // Matches tooltipKey
        pid: formattedPid, // Matches key: "pid"
        metricUnit: def.unit, // Matches unitKey
        supported: data?.isSupported ? "Yes" : "No", // Matches key: "supported"

        // --- 2. Live Data (From the SvelteMap) ---
        value: data?.value || "N/A", // Matches key: "value"
        updateInterval: def.update_interval_ms, // Matches key: "updateInterval"
        badgeColor: data?.isSupported
          ? "var(--normal-color)"
          : "var(--error-color)",
      };
    }),
  );

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });
  onDestroy(() => {
    canStore.stopLogging();
  });
</script>

<div class="data-grid-container overflow-auto">
  <DataTable {columns} data={tableData} bind:selectedRowIndex={activeIndex} />
</div>

<style>
  .data-grid-container {
    padding: 0;
    margin: 0;
  }
</style>
