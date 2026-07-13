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
  import { onDestroy, onMount, untrack } from "svelte";
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

  $effect(() => {
    if (
      isValidHex(telemetryStore.pidInput) &&
      telemetryStore.pidInput !== telemetryStore.lastValidPidForLength
    ) {
      telemetryStore.lastValidPidForLength = telemetryStore.pidInput;
      const pidVal = parseInt(telemetryStore.pidInput.replace(/^0x/i, ""), 16);
      telemetryStore.lengthInput = pidVal <= 0xff ? 2 : 3;
    }
  });

  $effect(() => {
    if (
      telemetryStore.modeInput &&
      telemetryStore.modeInput !== telemetryStore.lastValidModeForId
    ) {
      telemetryStore.lastValidModeForId = telemetryStore.modeInput;
      if (
        telemetryStore.modeInput === "0x01" ||
        telemetryStore.modeInput === "0x45"
      ) {
        telemetryStore.idInput = "0x7DF";
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

  $effect(() => {
    // Only subscribe to activeIndex changes
    const idx = activeIndex;

    untrack(() => {
      if (idx >= 0 && idx < telemetryStore.local_piddef.length) {
        const row = telemetryStore.local_piddef[idx];
        const rawPid = parseInt(row.pid, 16);
        const def = telemetryStore.lastPidDef?.find(
          (d: any) => Number(d.pid) === rawPid,
        );

        if (def) {
          telemetryStore.pidInput =
            "0x" + Number(def.pid).toString(16).toUpperCase().padStart(2, "0");
          telemetryStore.lastValidPidForLength = telemetryStore.pidInput; // Prevent auto-fill from instantly overwriting loaded length

          telemetryStore.nameInput = def.name || "";
          telemetryStore.descInput = def.description || "";
          telemetryStore.unitInput = def.unit || "";
          telemetryStore.minInput = def.minValue;
          telemetryStore.maxInput = def.maxValue;

          telemetryStore.modeInput =
            "0x" + Number(def.mode).toString(16).padStart(2, "0");
          telemetryStore.lastValidModeForId = telemetryStore.modeInput; // Prevent auto-fill from instantly overwriting loaded ID

          telemetryStore.formulaInput = def.formula || "";
          telemetryStore.lengthInput = def.length;
          telemetryStore.idInput = def.id
            ? "0x" + Number(def.id).toString(16).toUpperCase()
            : "";
          telemetryStore.priorityInput = def.priority;
          telemetryStore.updateIntervalInput = def.update_interval_ms;
          telemetryStore.colorInput =
            "#" + Number(def.color).toString(16).padStart(6, "0");
          telemetryStore.iconInput = def.icon || "";
        }
      } else {
        // Clear form when no row is selected
        telemetryStore.clearForm();
      }
    });
  });

  const icons = [
    "chart",
    "home",
    "user",
    "gear",
    "gauge",
    "thermometer",
    "droplet",
    "engine",
    "alert-circle",
    "lock",
    "clock",
    "timeline-arrow",
    "magnifying-glass",
    "reload",
    "dial",
    "trash",
    "route",
    "circle",
    "square",
    "circle-solid",
    "stopwatch",
    "circle-xmark",
    "circle-check",
    "circle-exclamantion",
    "circle-info",
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
        style="--module-accent: {telemetryStore.colorInput};"
      >
        <div class="title-row">
          <div class="header-title-row">
            <h2>{activeIndex === -1 ? "Editor" : "Editing"}</h2>
            <p class="status-subtitle">{selectedRow?.pid}</p>
          </div>
        </div>
        <div class="editor-wrapper">
          {#if telemetryStore.validationErrors.length > 0}
            <div class="validation-errors-container">
              <div class="validation-errors-header">
                <Icon name="circle-exclamantion" size={16} />
                Please fix the following errors:
              </div>
              <div class="validation-errors-list">
                {#each telemetryStore.validationErrors as error}
                  <div>• {error}</div>
                {/each}
              </div>
            </div>
          {/if}

          <!-- Row 1: ID, MODE, PID, LENGTH -->
          <div class="grid">
            <div>
              <input
                name="CAN ID"
                placeholder="CAN ID (e.g., 0x7DF)"
                aria-label="can id"
                aria-describedby="id-helper"
                autocomplete="off"
                bind:value={telemetryStore.idInput}
                aria-invalid={!isIdValidForMode(
                  telemetryStore.modeInput,
                  telemetryStore.idInput,
                )}
              />
              <small id="id-helper"> ECU CAN Identifier </small>
            </div>
            <div>
              <select
                bind:value={telemetryStore.modeInput}
                name="Mode"
                aria-label="mode"
                aria-describedby="mode-helper"
                aria-invalid={!isModeValidForPid(
                  telemetryStore.pidInput,
                  telemetryStore.modeInput,
                )}
              >
                <option value="" disabled selected>Select a mode...</option>
                {#each modes as mode}
                  <option value={mode.value}>{mode.label}</option>
                {/each}
              </select>
              <small id="mode-helper"> OBD2 Service Mode </small>
            </div>
            <div>
              <input
                name="PID"
                placeholder="PID (e.g., 0x0C)"
                aria-label="pid"
                aria-describedby="pid-helper"
                autocomplete="off"
                bind:value={telemetryStore.pidInput}
                aria-invalid={!isValidHex(telemetryStore.pidInput)}
              />
              <small id="pid-helper"> Parameter ID </small>
            </div>
            <div>
              <input
                type="number"
                name="Length"
                placeholder="Length (bytes)"
                aria-label="length"
                aria-describedby="length-helper"
                bind:value={telemetryStore.lengthInput}
                aria-invalid={!isLengthValidForPid(
                  telemetryStore.pidInput,
                  telemetryStore.lengthInput,
                )}
              />
              <small id="length-helper"> Expected response bytes </small>
            </div>
          </div>

          <!-- Row 2: NAME, DESCRIPTION -->
          <div class="grid margin-top">
            <div>
              <input
                name="Name"
                placeholder="Name"
                aria-label="name"
                aria-describedby="name-helper"
                autocomplete="off"
                bind:value={telemetryStore.nameInput}
                aria-invalid={!isValidName(telemetryStore.nameInput)}
              />
              <small id="name-helper"> Name of the parameter </small>
            </div>
            <div>
              <input
                name="Description"
                placeholder="Description (optional)"
                aria-label="description"
                aria-describedby="desc-helper"
                autocomplete="off"
                bind:value={telemetryStore.descInput}
                aria-invalid={telemetryStore.descInput.length > 0
                  ? !isValidDescription(telemetryStore.descInput)
                  : undefined}
              />
              <small id="desc-helper"> Detailed description </small>
            </div>
          </div>

          <!-- Row 3: UNIT, MIN, MAX -->
          <div class="grid margin-top">
            <div>
              <select
                bind:value={telemetryStore.unitInput}
                name="Unit"
                aria-label="unit"
                aria-describedby="unit-helper"
                aria-invalid={telemetryStore.unitInput ? false : undefined}
              >
                <option value="" disabled selected>Select a unit...</option>
                {#each units as unit}
                  <option value={unit}>{unit}</option>
                {/each}
              </select>
              <small id="unit-helper"> Unit of measurement </small>
            </div>
            <div>
              <input
                type="number"
                name="MinValue"
                placeholder="Min Value"
                aria-label="Min value"
                bind:value={telemetryStore.minInput}
                aria-invalid={telemetryStore.minInput !== undefined &&
                telemetryStore.maxInput !== undefined &&
                telemetryStore.minInput > telemetryStore.maxInput
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
                bind:value={telemetryStore.maxInput}
                aria-invalid={telemetryStore.minInput !== undefined &&
                telemetryStore.maxInput !== undefined &&
                telemetryStore.minInput > telemetryStore.maxInput
                  ? "true"
                  : undefined}
              />
              <small>Maximum expected value</small>
            </div>
          </div>

          <!-- Row 4: FORMULA -->
          <div class="grid margin-top">
            <div>
              <input
                name="Formula"
                placeholder="Formula (e.g., A * 100 / 255)"
                aria-label="formula"
                aria-describedby="formula-helper"
                autocomplete="off"
                bind:value={telemetryStore.formulaInput}
                aria-invalid={!isValidFormula(
                  telemetryStore.modeInput,
                  telemetryStore.formulaInput,
                )}
              />
              <small id="formula-helper">
                Expression to compute the final value
              </small>
            </div>
          </div>

          <!-- Row 5: PRIORITY, INTERVAL, COLOR, ICON -->
          <div class="grid margin-top">
            <div>
              <input
                type="number"
                name="Priority"
                placeholder="Priority (1-255)"
                aria-label="priority"
                aria-describedby="priority-helper"
                min="1"
                max="255"
                bind:value={telemetryStore.priorityInput}
                aria-invalid={telemetryStore.priorityInput !== undefined &&
                (telemetryStore.priorityInput < 1 ||
                  telemetryStore.priorityInput > 255)
                  ? "true"
                  : undefined}
              />
              <small id="priority-helper"> Polling priority </small>
            </div>
            <div>
              <input
                type="number"
                name="Update Interval"
                placeholder="Interval (ms)"
                aria-label="update interval"
                aria-describedby="interval-helper"
                min="0"
                max="4294967295"
                bind:value={telemetryStore.updateIntervalInput}
                aria-invalid={telemetryStore.updateIntervalInput !==
                  undefined &&
                telemetryStore.updateIntervalInput !== 0 &&
                (telemetryStore.updateIntervalInput < 16 ||
                  telemetryStore.updateIntervalInput > 4294967295)
                  ? "true"
                  : undefined}
              />
              <small id="interval-helper"
                >{telemetryStore.updateIntervalInput === 0
                  ? "Polling Disabled"
                  : "Interval in ms"}</small
              >
            </div>
            <div>
              <input
                class="color-picker"
                type="color"
                name="Color"
                aria-label="color"
                aria-describedby="color-helper"
                bind:value={telemetryStore.colorInput}
              />
              <small id="color-helper">
                Color ({telemetryStore.colorInput.toUpperCase()})
              </small>
            </div>
            <div>
              <div class="icon-selector-wrapper">
                <div class="icon-select-container">
                  <select
                    bind:value={telemetryStore.iconInput}
                    name="Icon"
                    aria-label="icon"
                    aria-describedby="icon-helper"
                  >
                    <option value="" disabled selected>Select an icon...</option
                    >
                    {#each icons as iconName}
                      <option value={iconName}>
                        {iconName
                          .split("-")
                          .map(
                            (word) =>
                              word.charAt(0).toUpperCase() + word.slice(1),
                          )
                          .join(" ")}
                      </option>
                    {/each}
                  </select>
                  <small id="icon-helper"> Display icon </small>
                </div>
                {#if telemetryStore.iconInput}
                  <div
                    class="icon-preview-box"
                    style="color: {telemetryStore.colorInput};"
                  >
                    <Icon name={telemetryStore.iconInput} size={24} />
                  </div>
                {/if}
              </div>
            </div>
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

  .editor-wrapper {
    width: 100%;
    padding-top: 0.5rem;
    /* Remove default PicoCSS orange focus outline, but preserve red validation outlines */
    --pico-form-element-focus-color: transparent;
    --pico-form-element-active-border-color: var(
      --pico-form-element-border-color
    );

    /* Condense the inputs for a sleeker, tighter design */
    --pico-form-element-spacing-vertical: 0.4rem;
    --pico-form-element-spacing-horizontal: 0.75rem;
    --pico-spacing: 0.25rem;
  }

  .margin-top {
    margin-top: 0.5rem;
  }

  .color-picker {
    height: calc(
      1rem * var(--pico-line-height) + var(--pico-form-element-spacing-vertical) *
        2 + var(--pico-border-width) * 2
    );
    padding: 0.25rem;
    width: 100%;
  }

  .icon-selector-wrapper {
    display: flex;
    align-items: flex-start;
    gap: 0.5rem;
    width: 100%;
  }

  .icon-select-container {
    flex-grow: 1;
  }

  .icon-preview-box {
    display: flex;
    align-items: center;
    justify-content: center;
    /* Height matches the input minus padding/border */
    height: calc(
      1rem * var(--pico-line-height) + var(--pico-form-element-spacing-vertical) *
        2 + var(--pico-border-width) * 2
    );
    width: 3rem;
    border: var(--pico-border-width) solid var(--pico-form-element-border-color);
    border-radius: var(--pico-border-radius);
    background-color: var(--pico-form-element-background-color);
  }

  .validation-errors-container {
    margin-bottom: 1.5rem;
    color: var(--pico-form-element-invalid-active-border-color);
  }

  .validation-errors-header {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    font-size: 0.9rem;
    font-weight: 600;
    margin-bottom: 0.25rem;
  }

  .validation-errors-list {
    font-size: 0.85rem;
    padding-left: 1.5rem;
    display: flex;
    flex-direction: column;
    gap: 0.25rem;
  }
</style>
