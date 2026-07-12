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
      const data = pidData?.get(Number(def.pid));

      const formattedPid =
        "0x" + Number(def.pid).toString(16).toUpperCase().padStart(2, "0");
      let isSupported: boolean =
        canStore.obd2Status?.supported_pids.groups.get(def.pid) ??
        data?.isSupported ??
        false;

      return {
        name: def.name,
        moduleDescription: def.description,
        pid: formattedPid,
        metricUnit: def.unit,
        supported: isSupported ? "Yes" : "No",

        value: data?.value || "N/A",
        updateInterval: def.update_interval_ms,
        badgeColor: isSupported ? "var(--normal-color)" : "var(--error-color)",
      };
    }),
  );

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });
  onDestroy(() => {
    canStore.stopLogging();
  });

  let activeTab = $state("html");
</script>

<div class="segmented-control-wrapper">
  <fieldset class="segmented-control">
    <label class="segment" class:active={activeTab === "html"}>
      <input
        type="radio"
        name="framework"
        value="html"
        bind:group={activeTab}
      />
      HTML
    </label>

    <label class="segment" class:active={activeTab === "react"}>
      <input
        type="radio"
        name="framework"
        value="react"
        bind:group={activeTab}
      />
      React
    </label>

    <label class="segment" class:active={activeTab === "vue"}>
      <input type="radio" name="framework" value="vue" bind:group={activeTab} />
      Vue
    </label>
  </fieldset>

  <div class="tab-content">
    {#if activeTab === "html"}
      <article class="panel fade-in">
        <h4>HTML Content</h4>
        <p>This is the raw, semantic DOM rendering layer.</p>
      </article>
    {:else if activeTab === "react"}
      <article class="panel fade-in">
        <h4>React Content</h4>
        <p>This is the virtual DOM and hook execution context.</p>
      </article>
    {:else if activeTab === "vue"}
      <article class="panel fade-in">
        <h4>Vue Content</h4>
        <p>This is the reactive template and composition API area.</p>
      </article>
    {/if}
  </div>
</div>

<div class="data-grid-container overflow-auto">
  <DataTable {columns} data={tableData} bind:selectedRowIndex={activeIndex} />
</div>

<style>
  .data-grid-container {
    padding: 0;
    margin: 0;
  }

  .segmented-control-wrapper {
    display: flex;
    flex-direction: column;
    gap: 1.5rem;
  }

  /* 1. The Outer Track */
  fieldset.segmented-control {
    display: inline-flex;
    margin: 0;
    padding: 4px;
    background: var(
      --pico-muted-border-color
    ); /* Matches the soft background */
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.05);
    width: fit-content; /* Keeps it wrapped tight around the text */
  }

  /* 2. The Labels (The clickable areas) */
  .segment {
    position: relative;
    margin: 0;
    padding: 8px 24px;
    cursor: pointer;
    font-size: 0.9rem;
    font-weight: 500;
    color: var(--pico-muted-color);
    border-radius: 8px;
    transition: all 0.2s ease;
    user-select: none;
  }

  /* 3. Hide the actual radio circles completely */
  .segment input[type="radio"] {
    position: absolute;
    opacity: 0;
    width: 0;
    height: 0;
    margin: 0;
  }

  /* 4. The Active State (The highlighted pill) */
  .segment.active {
    color: var(--pico-color); /* Make text brighter */
    /* Gives it that raised, frosted glass look from the screenshot */
    background: var(--pico-background-color);
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  }

  /* Hover effect for unselected tabs */
  .segment:not(.active):hover {
    color: var(--pico-primary);
  }

  /* Optional: Smooth fade-in for the content swapping */
  .fade-in {
    animation: fadeIn 0.3s ease-in-out;
  }

  @keyframes fadeIn {
    from {
      opacity: 0;
      transform: translateY(5px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }
</style>
