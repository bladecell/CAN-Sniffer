<script lang="ts">
  import { canStore } from "./lib/canStore.svelte";
  import {
    telemetryStore,
    isValidHex,
    isValidName,
    isValidDescription,
    isModeValidForPid,
    isLengthValidForPid,
    isIdValidForMode,
    isValidFormula,
  } from "./lib/telemetryStore.svelte";
  import { onDestroy, onMount } from "svelte";
  import type { DataTableProps, PidValue, PidDefinition } from "$lib/types";
  import DataTable from "./lib/components/DataTable.svelte";
  import Icon from "./lib/Icon.svelte";
  import type { Column } from "$lib/types";
  import { getModeLabel } from "$lib/pidHelpers.svelte.ts";
  import { SvelteMap } from "svelte/reactivity";

  let activeIndex = $state(-1);

  let piddef = $derived(canStore.pidDefinitions);

  function createRowObject(def: any, loaded: boolean, selected: boolean) {
    const rawPid = Number(def.pid);
    return {
      // Static fields
      name: def.name,
      moduleDescription: def.description,
      pid: "0x" + rawPid.toString(16).toUpperCase().padStart(2, "0"),
      metricUnit: def.unit,
      formula: def.formula,
      priority: def.priority,
      mode: def.mode,
      modeDisplayFormat: "hex",
      modeDescription: getModeLabel(def.mode),
      min_val: def.minValue,
      max_val: def.maxValue,
      len: def.length,
      updateInterval: def.update_interval_ms,

      // Stateful editable fields
      loaded: loaded,
      selected: selected,

      // Dynamic reactive getters
      get value() {
        return canStore.pids.get(rawPid)?.value ?? "N/A";
      },
      get isSupported() {
        return (
          canStore.obd2Status?.supported_pids.groups.get(rawPid) ??
          canStore.pids.get(rawPid)?.isSupported ??
          false
        );
      },
      get supported() {
        return this.isSupported ? "Yes" : "No";
      },
      get badgeColor() {
        return this.isSupported ? "var(--normal-color)" : "var(--error-color)";
      },
    };
  }

  $effect(() => {
    if (piddef && piddef !== telemetryStore.lastPidDef) {
      telemetryStore.lastPidDef = piddef;

      const newDefMap = new Map();
      for (const def of piddef) {
        newDefMap.set(Number(def.pid), def);
      }

      const updatedPids = [];

      // 1. Process existing elements to retain their selected/loaded states
      for (const existing of telemetryStore.local_piddef) {
        const rawPid = parseInt(existing.pid, 16);
        const updatedDef = newDefMap.get(rawPid);

        if (updatedDef) {
          // PID is still in the CAN store. Keep states, update definitions.
          updatedPids.push(
            createRowObject(updatedDef, existing.loaded, existing.selected),
          );
          newDefMap.delete(rawPid);
        } else {
          // PID was removed from CAN store. Keep it, but set loaded to false.
          existing.loaded = false;
          updatedPids.push(existing);
        }
      }

      // 2. Add brand new elements
      for (const newDef of newDefMap.values()) {
        updatedPids.push(createRowObject(newDef, true, false));
      }

      telemetryStore.local_piddef = updatedPids;
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

  // `tableData` is no longer needed since `local_piddef` handles dynamic getters!

  $effect(() => {
    if (isValidHex(pidInput) && pidInput !== lastValidPidForLength) {
      lastValidPidForLength = pidInput;
      const pidVal = parseInt(pidInput.replace(/^0x/i, ""), 16);
      lengthInput = pidVal <= 0xFF ? 2 : 3;
    }
  });

  $effect(() => {
    if (modeInput && modeInput !== lastValidModeForId) {
      lastValidModeForId = modeInput;
      if (modeInput === "0x01" || modeInput === "0x45") {
        idInput = "0x7DF";
      }
    }
  });

  $effect(() => {
    console.log("Active index changed to:", activeIndex);
  });

  $effect(() => {
    if (canStore.connected) canStore.startLogging();
  });
  onDestroy(() => {
    canStore.stopLogging();
  });

  let activeTab = $state("editor");

  let selectedElements = $derived(
    telemetryStore.local_piddef.filter((item) => item.selected),
  );
  let selectedRow = $derived(
    activeIndex >= 0 && activeIndex < telemetryStore.local_piddef.length
      ? telemetryStore.local_piddef[activeIndex]
      : null,
  );

  let pidInput = $state("");
  let nameInput = $state("");
  let descInput = $state("");
  let unitInput = $state("");
  let minInput = $state<number | undefined>(undefined);
  let maxInput = $state<number | undefined>(undefined);
  let modeInput = $state("");
  let lengthInput = $state<number | undefined>(undefined);
  let lastValidPidForLength = $state<string>("");
  let idInput = $state("");
  let lastValidModeForId = $state<string>("");
  let formulaInput = $state("");
  let priorityInput = $state<number | undefined>(5);
  let colorInput = $state("#01AAFF");
  let colorUint32 = $derived(parseInt(colorInput.replace("#", ""), 16));
  let updateIntervalInput = $state<number | undefined>(512);
  let iconInput = $state("");

  const icons = [
    "chart", "home", "user", "gear", "gauge", "thermometer", "droplet", "engine",
    "alert-circle", "lock", "clock", "timeline-arrow", "magnifying-glass", "reload",
    "dial", "trash", "route", "circle", "square", "circle-solid", "stopwatch",
    "circle-xmark", "circle-check", "circle-exclamantion", "circle-info"
  ];

  const modes = [
    { label: "Current Data", value: "0x01" },
    { label: "Read Data By Identifier", value: "0x22" },
    { label: "Derived Data", value: "0x45" },
  ];

  const units = [
    "%",
    "kPa",
    "Pa",
    "rpm",
    "km/h",
    "° before TDC",
    "grams/sec",
    "seconds",
    "ratio",
    "count",
    "km",
    "V",
    "minutes",
    "g/s",
    "°",
    "°C",
    "L/h",
    "L",
  ];
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

    <label class="segment" class:active={activeTab === "editor"}>
      <input
        type="radio"
        name="framework"
        value="editor"
        bind:group={activeTab}
      />
      Editor
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
        <div class="aligned-title-row">
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
          <div class="status-subtitle">
            <span class="label">Selected:</span>
            <span class="value">{selectedElements.length}</span>
          </div>
        </div>
      </div>
    {:else if activeTab === "editor"}
      <div
        class="dashboard-card header-container"
        style="--module-accent: #01AAFF;"
      >
        <div class="title-row">
          <div class="header-title-row">
            <h2>{activeIndex === -1 ? "Editor" : "Editing"}</h2>
            <p class="status-subtitle">{selectedRow?.pid}</p>
          </div>
        </div>
        <div class="pid-editor-form">
          <input
            name="PID"
            placeholder="PID (e.g., 0x01)"
            aria-label="pid"
            aria-describedby="pid-helper"
            autocomplete="off"
            bind:value={pidInput}
            aria-invalid={!isValidHex(pidInput)}
          />
          <small id="pid-helper"> On-board diagnostics Parameter ID </small>
          <input
            name="Name"
            placeholder="Name"
            aria-label="name"
            aria-describedby="name-helper"
            autocomplete="off"
            bind:value={nameInput}
            aria-invalid={!isValidName(nameInput)}
          />
          <small id="name-helper"> Name of the parameter </small>
          <input
            name="Description"
            placeholder="Description (optional)"
            aria-label="description"
            aria-describedby="desc-helper"
            autocomplete="off"
            bind:value={descInput}
            aria-invalid={descInput.length > 0
              ? !isValidDescription(descInput)
              : undefined}
          />
          <small id="desc-helper">
            Detailed description of the parameter
          </small>

          <select
            bind:value={unitInput}
            name="Unit"
            aria-label="unit"
            aria-describedby="unit-helper"
          >
            <option value="" disabled selected>Select a unit...</option>
            {#each units as unit}
              <option value={unit}>{unit}</option>
            {/each}
          </select>
          <small id="unit-helper"> Unit of measurement </small>

          <div class="grid" style="margin-top: 1rem;">
            <div>
              <input
                type="number"
                name="MinValue"
                placeholder="Min Value"
                aria-label="Min value"
                bind:value={minInput}
                aria-invalid={minInput !== undefined &&
                maxInput !== undefined &&
                minInput > maxInput
                  ? "true"
                  : undefined}
              />
              <small>Minimum expected value</small>
            </div>
            <div>
              <input
                type="number"
                name="MaxValue"
                placeholder="Max Value"
                aria-label="Max value"
                bind:value={maxInput}
                aria-invalid={minInput !== undefined &&
                maxInput !== undefined &&
                minInput > maxInput
                  ? "true"
                  : undefined}
              />
              <small>Maximum expected value</small>
            </div>
          </div>

          <select
            bind:value={modeInput}
            name="Mode"
            aria-label="mode"
            aria-describedby="mode-helper"
            aria-invalid={!isModeValidForPid(pidInput, modeInput)}
            style="margin-top: 1rem;"
          >
            <option value="" disabled selected>Select a mode...</option>
            {#each modes as mode}
              <option value={mode.value}>{mode.label}</option>
            {/each}
          </select>
          <small id="mode-helper"> OBD2 Service Mode </small>

          <input
            name="Formula"
            placeholder="Formula (e.g., A * 100 / 255)"
            aria-label="formula"
            aria-describedby="formula-helper"
            autocomplete="off"
            bind:value={formulaInput}
            aria-invalid={!isValidFormula(modeInput, formulaInput)}
            style="margin-top: 1rem;"
          />
          <small id="formula-helper"> Expression to compute the final value </small>

          <input
            type="number"
            name="Length"
            placeholder="Length (bytes)"
            aria-label="length"
            aria-describedby="length-helper"
            bind:value={lengthInput}
            aria-invalid={!isLengthValidForPid(pidInput, lengthInput)}
            style="margin-top: 1rem;"
          />
          <small id="length-helper"> Length of the expected response in bytes </small>

          <input
            name="CAN ID"
            placeholder="CAN ID (e.g., 0x7DF)"
            aria-label="can id"
            aria-describedby="id-helper"
            autocomplete="off"
            bind:value={idInput}
            aria-invalid={!isIdValidForMode(modeInput, idInput)}
            style="margin-top: 1rem;"
          />
          <small id="id-helper"> ECU CAN Identifier </small>

          <input
            type="number"
            name="Priority"
            placeholder="Priority (1-255)"
            aria-label="priority"
            aria-describedby="priority-helper"
            min="1"
            max="255"
            bind:value={priorityInput}
            aria-invalid={priorityInput !== undefined && (priorityInput < 1 || priorityInput > 255) ? "true" : undefined}
            style="margin-top: 1rem;"
          />
          <small id="priority-helper"> Polling priority </small>

          <input
            type="color"
            name="Color"
            aria-label="color"
            aria-describedby="color-helper"
            bind:value={colorInput}
            style="margin-top: 1rem; height: 3rem; padding: 0.25rem;"
          />
          <small id="color-helper"> Display color ({colorInput.toUpperCase()}) </small>

          <input
            type="number"
            name="Update Interval"
            placeholder="Update Interval (ms)"
            aria-label="update interval"
            aria-describedby="interval-helper"
            min="0"
            max="4294967295"
            bind:value={updateIntervalInput}
            aria-invalid={updateIntervalInput !== undefined && updateIntervalInput !== 0 && (updateIntervalInput < 16 || updateIntervalInput > 4294967295) ? "true" : undefined}
            style="margin-top: 1rem;"
          />
          <small id="interval-helper">
            {updateIntervalInput === 0 ? "Polling Disabled" : "Update interval in milliseconds"}
          </small>

          <div style="margin-top: 1rem; display: flex; align-items: flex-start; gap: 1rem;">
            <div style="flex-grow: 1;">
              <select 
                bind:value={iconInput} 
                name="Icon"
                aria-label="icon"
                aria-describedby="icon-helper"
                aria-invalid={iconInput ? false : undefined}
              >
                <option value="" disabled selected>Select an icon...</option>
                {#each icons as iconName}
                  <option value={iconName}>
                    {iconName.split("-").map(word => word.charAt(0).toUpperCase() + word.slice(1)).join(" ")}
                  </option>
                {/each}
              </select>
              <small id="icon-helper"> Display icon </small>
            </div>
            {#if iconInput}
              <div style="width: calc(1rem * var(--pico-line-height) + var(--pico-form-element-spacing-vertical) * 2 + var(--pico-border-width) * 2); height: calc(1rem * var(--pico-line-height) + var(--pico-form-element-spacing-vertical) * 2 + var(--pico-border-width) * 2); display: flex; align-items: center; justify-content: center; background: var(--pico-form-element-background-color); border: var(--pico-border-width) solid var(--pico-form-element-border-color); border-radius: var(--pico-border-radius); color: {colorInput};">
                <Icon name={iconInput} size={24} />
              </div>
            {/if}
          </div>

        </div>
      </div>
    {:else if activeTab === "vue"}
      <article class="panel fade-in">
        <h4>Vue Content</h4>
        <p>This is the reactive template and composition API area.</p>
      </article>
    {/if}
  </div>
</div>

<DataTable
  {columns}
  data={telemetryStore.local_piddef}
  bind:selectedRowIndex={activeIndex}
/>

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

  .aligned-title-row {
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

  .header-title-row {
    display: flex;
    align-items: baseline;
    gap: 1.5rem;
    flex-wrap: wrap;
  }

  .header-title-row h2 {
    margin: 0;
    font-size: 1.5rem;
  }

  .status-subtitle {
    margin: 0;
    color: var(--module-accent, #a1a1aa);
    text-transform: uppercase;
    font-size: 0.85rem;
    letter-spacing: 0.05em;
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
