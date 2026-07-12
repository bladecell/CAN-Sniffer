<script lang="ts">
  import { canStore } from "./lib/canStore.svelte";
  import { onDestroy, onMount } from "svelte";
  import type { DataTableProps, PidValue, PidDefinition } from "$lib/types";
  import DataTable from "./lib/components/DataTable.svelte";
  import type { Column } from "$lib/types";
  import { getModeLabel } from "$lib/pidHelpers.svelte.ts";

  let activeIndex = $state(-1);

  let piddef = $derived(canStore.pidDefinitions);
  let pidData = $derived(canStore.pids);
  let local_piddef = $state<any[]>(
    (piddef || []).map((def) => ({
      ...def,
      loaded: true,
      selected: false,
    })),
  );

  let lastPidDef = $state<any[] | null>(null);

  $effect(() => {
    if (piddef && piddef !== lastPidDef) {
      lastPidDef = piddef;
      local_piddef = piddef.map((def) => {
        const existing = local_piddef.find((l) => l.pid === def.pid);
        return {
          ...def,
          loaded: existing ? existing.loaded : true,
          selected: existing ? existing.selected : false,
        };
      });
    }
  });

  const columns: Column[] = [
    {
      label: "Select",
      key: "selected",
      type: "checkbox",
      width_px: 70, // Fixed: Just wide enough for the checkbox +padg
    },
    {
      label: "Name",
      key: "name",
      type: "text",
      width_px: 160,
      tooltipKey: "moduleDescription",
    },
    {
      label: "PID",
      key: "pid",
      type: "code",
      width_px: 120,
    },
    {
      label: "Mode",
      key: "mode",
      type: "number",
      formatKey: "modeDisplayFormat",
      hidden: true,
      tooltipKey: "modeDescription",
      width_px: 70,
    },
    {
      label: "Length",
      key: "len",
      type: "number",
      width_px: 70,
      hidden: true,
    },
    {
      label: "Formula",
      key: "formula",
      type: "code",
      width_px: 500,
      hidden: true,
    },
    {
      label: "Value",
      key: "value",
      type: "number",
      unitKey: "metricUnit",
      width_px: 140, // Fixed pixel width to prevent subpixel roundingoverw
    },
    {
      label: "Min Value",
      key: "min_val",
      type: "number",
      width_px: 140,
      unitKey: "metricUnit",
      hidden: true,
    },
    {
      label: "Max Value",
      key: "max_val",
      type: "number",
      width_px: 140,
      unitKey: "metricUnit",
      hidden: true,
    },
    {
      label: "Update Interval",
      key: "updateInterval",
      type: "number",
      unit: "ms",
      width_px: 120,
    },
    {
      label: "Priority",
      key: "priority",
      type: "number",
      width_px: 80,
      hidden: true,
    },
    {
      label: "Supported",
      key: "supported",
      type: "badge",
      width_px: 100,
      colorKey: "badgeColor",
    },
    {
      label: "Loaded",
      key: "loaded",
      type: "toggle",
      width_px: 80,
    },
  ];

  let tableData = $derived(
    (local_piddef || []).map((def) => {
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
        formula: def.formula,
        priority: def.priority,
        mode: def.mode,
        modeDisplayFormat: "hex",
        modeDescription: getModeLabel(def.mode),
        min_val: def.minValue,
        max_val: def.maxValue,
        len: def.length,
        get selected() {
          return def.selected;
        },
        set selected(val) {
          def.selected = val;
        },
        get loaded() {
          return def.loaded;
        },
        set loaded(val) {
          def.loaded = val;
        },

        value: data?.value || "N/A",
        updateInterval: def.update_interval_ms,
        badgeColor: isSupported ? "var(--normal-color)" : "var(--error-color)",
      };
    }),
  );

  $effect(() => {
    console.log("Active index changed to:", activeIndex);
  });

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });
  onDestroy(() => {
    canStore.stopLogging();
  });

  let activeTab = $state("info");
</script>

<div class="segmented-control-wrapper">
  <fieldset class="segmented-control">
    <label class="segment" class:active={activeTab === "info"}>
      <input
        type="radio"
        name="framework"
        value="info"
        bind:group={activeTab}
      />
      Info
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
    {#if activeTab === "info"}
      <div
        class="dashboard-card header-container"
        style="--module-accent: #01AAFF;"
      >
        <div class="header-title-row">
          <div class="status-subtitle">
            <span class="label">Loaded PIDs:</span>
            <span class="value">{canStore.obd2Status?.pid_def_count}</span>
          </div>
          <div class="status-subtitle">
            <span class="label">Supported PIDs:</span>
            <span class="value"
              >{canStore.obd2Status?.supported_pids.count}</span
            >
          </div>
        </div>
      </div>
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

<DataTable {columns} data={tableData} bind:selectedRowIndex={activeIndex} />

<style>
  .header-container {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 1.5rem;
    width: 100%;
    padding: 1.5rem;
    margin-bottom: 2rem;
    box-sizing: border-box;
  }

  .dashboard-card.header-container {
    border: none !important;
  }

  .dashboard-card.header-container:hover {
    box-shadow: none;
    border-color: var(--pico-muted-border-color) !important;
  }

  .header-title-row {
    display: grid;
    grid-template-columns: max-content 1fr;
    column-gap: 0.75rem; /* Space between label and value */
    row-gap: 0.25rem; /* Space between the two rows */
    align-items: start;
  }

  /* 2. Visually strip away the wrappers so children join the master grid */
  .status-subtitle {
    display: contents;
  }

  /* 3. Style the text as normal */
  .status-subtitle .label {
    color: var(--pico-muted-color);
    font-weight: 500;
  }

  .status-subtitle .value {
    color: var(--pico-color);
    text-align: left;
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
