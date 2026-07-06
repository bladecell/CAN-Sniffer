<script lang="ts">
  import { canStore } from "./lib/canStore.svelte";
  import { untrack, onDestroy, onMount } from "svelte";
  import type { DataTableProps } from "$lib/types";
  import DataTable from "./lib/components/DataTable.svelte";
  import type { Column } from "$lib/types";

  let activeIndex = $state(-1);
  $effect(() => {
    console.log("Active index changed to:", activeIndex);
  });

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
  ];

  const testData: DataTableProps[] = [
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
</script>

<div class="data-grid-container overflow-auto">
  <DataTable {columns} data={testData} bind:selectedRowIndex={activeIndex} />
</div>

<style>
  .data-grid-container {
    padding: 0;
    margin: 0;
  }
</style>
