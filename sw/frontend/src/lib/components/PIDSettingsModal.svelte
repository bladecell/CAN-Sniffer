<script lang="ts">
import { vehicleStore } from "$lib/vehicleStore.svelte";

  import TelemetryStepper from "./TelemetryStepper.svelte";
  import SelectMenu from "./SelectMenu.svelte";
  import Modal from "./Modal.svelte";
  import type {
    DashboardItem,
    PidGridItem,
    SpecialGridItem,
    OverviewGridItem,
    BatteryGridItem,
    BarStyle,
    GaugeStyle,
  } from "$lib/types";
  import { MODULE_CONFIGS, getModuleBounds } from "$lib/types";
  import { untrack } from "svelte";
  import type { SelectMenuOption } from "$lib/selectMenu";

  interface Props {
    isOpen: boolean;
    item: DashboardItem | null;
    isNewCard?: boolean;
    previewItem: DashboardItem | null;
    onSave: (updatedItem: DashboardItem) => void;
    onClose: () => void;
  }

  let {
    isOpen,
    item,
    isNewCard = false,
    previewItem = $bindable(),
    onSave,
    onClose,
  }: Props = $props();

  let localItem = $state<DashboardItem>({
    id: "",
    cardType: "pid",
    pid: 0,
    displayMode: "card",
    w: 10,
    h: 7,
  } as DashboardItem);

  let internalSync = $state(false);

  const gaugeStyleOptions: SelectMenuOption<GaugeStyle | undefined>[] = [
    { value: undefined, label: "Arc (Default 240°)" },
    { value: "arc", label: "Arc (240°)" },
    { value: "half", label: "Half (180°)" },
    { value: "donut", label: "Donut (360°)" },
    { value: "speedometer", label: "Speedometer (Detailed)" },
    { value: "gradient", label: "Gradient Progress" },
  ];

  const pidBarStyleOptions: SelectMenuOption<BarStyle>[] = [
    { value: "solid", label: "Solid (Default)" },
    { value: "zones", label: "Zones (equal thirds)" },
    { value: "segmented", label: "Segmented" },
    { value: "gradient", label: "Gradient" },
    { value: "marker", label: "Marker" },
  ];

  const batteryBarStyleOptions: SelectMenuOption<BarStyle>[] = [
    { value: "zones", label: "Zones (Default)" },
    { value: "solid", label: "Solid" },
    { value: "segmented", label: "Segmented" },
    { value: "gradient", label: "Gradient" },
    { value: "marker", label: "Marker" },
  ];

  const pidOptions: SelectMenuOption<number>[] = $derived(
    vehicleStore.pidDefinitions.length > 0
      ? vehicleStore.pidDefinitions.map((d) => ({
          value: d.pid,
          label: `0x${d.pid.toString(16).toUpperCase()} — ${d.name}`,
        }))
      : [{ value: 0, label: "No PID definitions available", disabled: true }],
  );

  const pidNameOptions: SelectMenuOption<number>[] = $derived(
    vehicleStore.pidDefinitions.map((d) => ({ value: d.pid, label: d.name })),
  );

  $effect(() => {
    if (item) {
      untrack(() => {
        internalSync = true;
        localItem = JSON.parse(JSON.stringify(item));

        // POLYFILL: Ensure Overview cards have required properties
        if (localItem.cardType === "overview") {
          const overItem = localItem as OverviewGridItem;
          if (!overItem.pids || overItem.pids.length !== 3) {
            overItem.pids = [0x0c, 0x2206, 0x04];
          }
          if (!overItem.color) {
            overItem.color = "#3b82f6";
          }
        }

        previewItem = JSON.parse(JSON.stringify(localItem));
        setTimeout(() => (internalSync = false), 50);
      });
    }
  });

  $effect(() => {
    // Sync local changes to the global previewItem for live dashboard updates
    if (!isOpen || !item || internalSync) return;

    // Track EVERYTHING we want to see in the live preview
    const { w, h, cardType } = localItem;

    // Read properties dynamically based on type for tracking
    let pidFields: Pick<PidGridItem, "pid" | "displayMode" | "gaugeStyle"> | undefined;
    let overFields: Pick<OverviewGridItem, "displayMode" | "color"> & { pids: string } | undefined;
    if (localItem.cardType === "pid") {
      const { pid, displayMode, gaugeStyle } = localItem as PidGridItem;
      pidFields = { pid, displayMode, gaugeStyle };
    } else if (localItem.cardType === "overview") {
      const { displayMode, color, pids } = localItem as OverviewGridItem;
      overFields = { displayMode, color, pids: JSON.stringify(pids) }; // Track deep array changes
    }

    untrack(() => {
      // Re-create object to trigger reference-based reactivity in parent
      const nextPreview = { ...localItem, w, h, cardType };

      if (pidFields) {
        Object.assign(nextPreview, pidFields);
      } else if (overFields) {
        const { pids, ...rest } = overFields;
        Object.assign(nextPreview, rest, { pids: JSON.parse(pids) });
      }

      previewItem = nextPreview as DashboardItem;
    });
  });

  const bounds = $derived(getModuleBounds(localItem));

  function handleCardTypeChange() {
    // Resets the entire item structure when the classification changes
    if (localItem.cardType === "pid") {
      const pidItem = localItem as PidGridItem;
      pidItem.pid = vehicleStore.pidDefinitions[0]?.pid || 0;
      pidItem.displayMode = "card";
    } else if (localItem.cardType === "overview") {
      const overItem = localItem as OverviewGridItem;
      overItem.displayMode = "default";
      overItem.pids = [0x0c, 0x2206, 0x04];
      overItem.color = "#3b82f6";
    } else if (localItem.cardType === "battery") {
      const batteryItem = localItem as BatteryGridItem;
      batteryItem.displayMode = "glyph";
      delete batteryItem.gaugeStyle;
      delete batteryItem.barStyle;
    } else {
      const specialItem = localItem as SpecialGridItem;
      specialItem.displayMode = "default";
      const stray = specialItem as Partial<Pick<PidGridItem, "pid"> & Pick<OverviewGridItem, "pids" | "color">>;
      delete stray.pid;
      delete stray.pids;
      delete stray.color;
    }
    updateDimensionsToMin();
  }

  function handleVisualizationChange() {
    // Only updates the grid bounds based on the new visualization mode
    updateDimensionsToMin();
  }

  function updateDimensionsToMin() {
    const newBounds = getModuleBounds(localItem);
    localItem.w = newBounds.min.w;
    localItem.h = newBounds.min.h;
  }

  function handleCancel() {
    previewItem = null;
    onClose();
  }

  function handleSubmit(e: SubmitEvent) {
    e.preventDefault();
    previewItem = null;
    onSave(localItem);
  }
</script>

<Modal
  open={isOpen}
  title={isNewCard ? "Create System Component" : "Configure Component Parameters"}
  size="lg"
  onClose={handleCancel}
>
  <form id="pid-settings-form" onsubmit={handleSubmit}>
        <label for="card-type-select" class="field-heading"
          >Component Classification</label
        >
        <div class="orange-select-field">
          <SelectMenu
            id="card-type-select"
            bind:value={localItem.cardType}
            onChange={handleCardTypeChange}
            disabled={!isNewCard}
            options={[
              { value: "pid", label: "OBD-II Parameter" },
              { value: "battery", label: "Car Battery (12 V)" },
              { value: "dtcs", label: "Diagnostic Trouble Codes (DTC)" },
              { value: "overview", label: "Performance Overview" },
            ]}
          />
        </div>

        <div class="steppers-inline-grid">
          <div class="stepper-field-wrapper">
            <span class="field-heading">Grid Width Columns</span>
            <TelemetryStepper
              bind:value={localItem.w}
              min={bounds.min.w}
              max={bounds.max.w}
            />
          </div>

          <div class="stepper-field-wrapper">
            <span class="field-heading">Grid Height Rows</span>
            <TelemetryStepper
              bind:value={localItem.h}
              min={bounds.min.h}
              max={bounds.max.h}
            />
          </div>
        </div>

        <hr class="form-section-divider" />

        {#if localItem.cardType === "pid"}
          {@const pidItem = localItem as PidGridItem}
          <div class="conditional-settings-panel animate-fade-in">
            <div class="panel-section-title">PID Sensor Parameters</div>

            <label for="pid-sensor-select" class="field-heading"
              >Target Vehicle Sensor Stream</label
            >
            <div class="orange-select-field">
              <SelectMenu
                id="pid-sensor-select"
                bind:value={pidItem.pid}
                options={pidOptions}
              />
            </div>

            <label for="pid-visualization-select" class="field-heading"
              >Widget Visualization Mode</label
            >
            <div class="orange-select-field">
              <SelectMenu
                id="pid-visualization-select"
                bind:value={pidItem.displayMode}
                onChange={handleVisualizationChange}
                options={[
                  { value: "card", label: "Digit Badge" },
                  { value: "chart", label: "Live History Chart" },
                  { value: "gauge", label: "Radial Gauge" },
                  { value: "bar", label: "Horizon Bar" },
                ]}
              />
            </div>

            {#if pidItem.displayMode === "gauge"}
              <label for="pid-gauge-style-select" class="field-heading"
                >Gauge Style</label
              >
              <div class="orange-select-field">
                <SelectMenu
                  id="pid-gauge-style-select"
                  bind:value={pidItem.gaugeStyle}
                  options={gaugeStyleOptions}
                />
              </div>
            {/if}

            {#if pidItem.displayMode === "bar"}
              <label for="pid-bar-style-select" class="field-heading"
                >Bar Style</label
              >
              <div class="orange-select-field">
                <SelectMenu
                  id="pid-bar-style-select"
                  value={pidItem.barStyle ?? "solid"}
                  onChange={(v) => (pidItem.barStyle = v)}
                  options={pidBarStyleOptions}
                />
              </div>
            {/if}
          </div>
        {:else if localItem.cardType === "overview"}
          {@const overItem = localItem as OverviewGridItem}
          <div class="conditional-settings-panel animate-fade-in">
            <div class="panel-section-title">
              Performance Overview Configuration
            </div>

            <div class="color-picker-row">
              <label for="over-color" class="field-heading"
                >Card Accent Color</label
              >
              <input
                type="color"
                id="over-color"
                bind:value={overItem.color}
                class="pico-color-input"
              />
            </div>

            <div class="pids-selection-grid">
              {#each [0, 1, 2] as idx (idx)}
                <div class="pid-select-field">
                  <label for="pid-select-{idx}" class="field-heading"
                    >Footer Metric #{idx + 1}</label
                  >
                  <div class="orange-select-field compact">
                    <SelectMenu
                      id="pid-select-{idx}"
                      bind:value={overItem.pids[idx]}
                      options={pidNameOptions}
                    />
                  </div>
                </div>
              {/each}
            </div>
          </div>
        {:else if localItem.cardType === "battery"}
          {@const batteryItem = localItem as BatteryGridItem}
          <div class="conditional-settings-panel animate-fade-in">
            <div class="panel-section-title">Car Battery Parameters</div>

            <label for="battery-visualization-select" class="field-heading"
              >Widget Visualization Mode</label
            >
            <div class="orange-select-field">
              <SelectMenu
                id="battery-visualization-select"
                bind:value={batteryItem.displayMode}
                onChange={handleVisualizationChange}
                options={[
                  { value: "glyph", label: "Battery" },
                  { value: "bar", label: "Horizontal Bar" },
                  { value: "gauge", label: "Radial Gauge" },
                  { value: "chart", label: "Live History Chart" },
                ]}
              />
            </div>

            {#if batteryItem.displayMode === "bar"}
              <label for="battery-bar-style-select" class="field-heading"
                >Bar Style</label
              >
              <div class="orange-select-field">
                <SelectMenu
                  id="battery-bar-style-select"
                  value={batteryItem.barStyle ?? "zones"}
                  onChange={(v) => (batteryItem.barStyle = v)}
                  options={batteryBarStyleOptions}
                />
              </div>
            {/if}

            {#if batteryItem.displayMode === "gauge"}
              <label for="battery-gauge-style-select" class="field-heading"
                >Gauge Style</label
              >
              <div class="orange-select-field">
                <SelectMenu
                  id="battery-gauge-style-select"
                  bind:value={batteryItem.gaugeStyle}
                  options={gaugeStyleOptions}
                />
              </div>
            {/if}
          </div>
        {:else}
          <div class="static-presets-notice animate-fade-in">
            <small>
              The selected <strong>{localItem.cardType.toUpperCase()}</strong> module
              uses a specialized layout. No extra properties are needed.
            </small>
          </div>
        {/if}

  </form>
  {#snippet footer()}
    <button type="submit" form="pid-settings-form" class="btn pri">Apply</button>
  {/snippet}
</Modal>

<style>
  .field-heading {
    display: block;
    margin-bottom: 6px;
    color: var(--muted);
    font-size: 0.85rem;
    font-weight: 700;
  }
  .orange-select-field {
    margin-bottom: 1rem;
  }
  .orange-select-field.compact {
    margin-bottom: 0.5rem;
  }
  .orange-select-field :global(.select-menu-trigger) {
    background: var(--ctl);
    border: 0;
    border-radius: var(--r-ctl);
    color: var(--ctl-text);
    box-shadow: none;
  }
  .orange-select-field.compact :global(.select-menu-trigger) {
    min-height: 40px;
    font-size: 0.85rem;
  }
  .steppers-inline-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 20px;
    margin: 1.25rem 0;
  }
  .form-section-divider {
    height: 1px;
    margin: 1.25rem 0;
    border: 0;
    background: var(--ctl);
  }
  .conditional-settings-panel {
    background: var(--card);
    padding: 1rem;
    border-radius: var(--r-ctl);
    margin-bottom: 1.5rem;
  }
  .panel-section-title {
    font-size: 0.7rem;
    text-transform: uppercase;
    letter-spacing: 0.06em;
    font-weight: 800;
    margin-bottom: 0.85rem;
  }
  .color-picker-row {
    display: flex;
    align-items: center;
    gap: 16px;
    margin-bottom: 1.5rem;
  }
  .pico-color-input {
    width: 60px;
    height: 40px;
    padding: 3px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    cursor: pointer;
  }
  .pids-selection-grid {
    display: flex;
    flex-direction: column;
    gap: 12px;
  }
  .static-presets-notice {
    padding: 1rem;
    border-radius: var(--r-ctl);
    background: var(--card);
    text-align: center;
    color: var(--muted);
    margin-bottom: 1.5rem;
  }
  .animate-fade-in {
    animation: modalBoxFadeIn 0.18s ease-out;
  }
  @keyframes modalBoxFadeIn {
    from {
      opacity: 0;
      transform: translateY(3px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }
</style>
