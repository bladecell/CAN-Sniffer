<script lang="ts">
  import { canStore } from "$lib/canStore.svelte";
  import TelemetryStepper from "./TelemetryStepper.svelte";
  import type {
    DashboardItem,
    PidGridItem,
    SpecialGridItem,
    OverviewGridItem,
  } from "$lib/types";
  import { MODULE_CONFIGS, getModuleBounds } from "$lib/types";
  import { untrack } from "svelte";

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
    let p: any, m: any, g: any, c: any, ps: any;
    if (localItem.cardType === "pid") {
      p = (localItem as PidGridItem).pid;
      m = (localItem as PidGridItem).displayMode;
      g = (localItem as PidGridItem).gaugeStyle;
    } else if (localItem.cardType === "overview") {
      const over = localItem as OverviewGridItem;
      m = over.displayMode;
      c = over.color;
      ps = JSON.stringify(over.pids); // Track deep array changes
    }

    untrack(() => {
      // Re-create object to trigger reference-based reactivity in parent
      const nextPreview = { ...localItem, w, h, cardType };

      if (localItem.cardType === "pid") {
        (nextPreview as PidGridItem).pid = p;
        (nextPreview as PidGridItem).displayMode = m;
        (nextPreview as PidGridItem).gaugeStyle = g;
      } else if (localItem.cardType === "overview") {
        (nextPreview as OverviewGridItem).displayMode = m;
        (nextPreview as OverviewGridItem).pids = JSON.parse(ps);
        (nextPreview as OverviewGridItem).color = c;
      }

      previewItem = nextPreview as DashboardItem;
    });
  });

  const bounds = $derived(getModuleBounds(localItem));

  function handleCardTypeChange() {
    // Resets the entire item structure when the classification changes
    if (localItem.cardType === "pid") {
      const pidItem = localItem as PidGridItem;
      pidItem.pid = canStore.pidDefinitions[0]?.pid || 0;
      pidItem.displayMode = "card";
    } else if (localItem.cardType === "overview") {
      const overItem = localItem as OverviewGridItem;
      overItem.displayMode = "default";
      overItem.pids = [0x0c, 0x2206, 0x04];
      overItem.color = "#3b82f6";
    } else {
      const specialItem = localItem as SpecialGridItem;
      specialItem.displayMode = "default";
      delete (specialItem as any).pid;
      delete (specialItem as any).pids;
      delete (specialItem as any).color;
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

{#if isOpen}
  <!-- svelte-ignore a11y_click_events_have_key_events -->
  <!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <div class="custom-modal-backdrop" onclick={handleCancel}>
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <article
      onclick={(e) => e.stopPropagation()}
      class="pico-orange-glass-modal blur-background"
    >
      <header class="modal-header-hull">
        <h5 class="modal-title-heading">
          {isNewCard ? "Create System Module" : "Configure Module Parameters"}
        </h5>
        <button aria-label="Close" class="close-modal-x" onclick={handleCancel}
          >✕</button
        >
      </header>

      <form onsubmit={handleSubmit} class="modal-form-body">
        <label for="card-type-select" class="field-heading"
          >Module Classification</label
        >
        <select
          id="card-type-select"
          bind:value={localItem.cardType}
          onchange={handleCardTypeChange}
          disabled={!isNewCard}
          class="orange-select-field"
        >
          <option value="pid">OBD-II Parameter</option>
          <option value="battery">System Battery Monitor</option>
          <option value="dtcs">Diagnostic Trouble Codes (DTC)</option>
          <option value="overview">Performance Overview</option>
        </select>

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
            <select
              id="pid-sensor-select"
              bind:value={pidItem.pid}
              class="orange-select-field"
            >
              {#each canStore.pidDefinitions as definition}
                <option value={definition.pid}>
                  0x{definition.pid.toString(16).toUpperCase()} — {definition.name}
                </option>
              {:else}
                <option value={0} disabled>No PID definitions available</option>
              {/each}
            </select>

            <label for="pid-visualization-select" class="field-heading"
              >Widget Visualization Mode</label
            >
            <select
              id="pid-visualization-select"
              bind:value={pidItem.displayMode}
              onchange={handleVisualizationChange}
              class="orange-select-field"
            >
              <option value="card">Digit Badge</option>
              <option value="chart">Live History Chart</option>
              <option value="gauge">Radial Gauge</option>
              <option value="bar">Horizon Bar</option>
            </select>

            {#if pidItem.displayMode === "gauge"}
              <label for="pid-gauge-style-select" class="field-heading"
                >Gauge Style</label
              >
              <select
                id="pid-gauge-style-select"
                bind:value={pidItem.gaugeStyle}
                class="orange-select-field"
              >
                <option value={undefined}>Arc (Default 240°)</option>
                <option value="arc">Arc (240°)</option>
                <option value="half">Half (180°)</option>
                <option value="donut">Donut (360°)</option>
                <option value="speedometer">Speedometer (Detailed)</option>
                <option value="gradient">Gradient Progress</option>
              </select>
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
              {#each [0, 1, 2] as idx}
                <div class="pid-select-field">
                  <label for="pid-select-{idx}" class="field-heading"
                    >Footer Metric #{idx + 1}</label
                  >
                  <select
                    id="pid-select-{idx}"
                    bind:value={overItem.pids[idx]}
                    class="orange-select-field compact"
                  >
                    {#each canStore.pidDefinitions as definition}
                      <option value={definition.pid}>
                        {definition.name}
                      </option>
                    {/each}
                  </select>
                </div>
              {/each}
            </div>
          </div>
        {:else}
          <div class="static-presets-notice animate-fade-in">
            <small>
              The selected <strong>{localItem.cardType.toUpperCase()}</strong> module
              uses a specialized layout. No extra properties are needed.
            </small>
          </div>
        {/if}

        <footer class="modal-footer-actions">
          <button type="submit" class="apply-btn">Apply</button>
        </footer>
      </form>
    </article>
  </div>
{/if}

<style>
  .custom-modal-backdrop {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
    background: rgba(0, 0, 0, 0.4);
    display: flex;
    align-items: center;
    justify-content: center;
    z-index: 12000;
    padding: 16px;
    border-radius: 12px;
  }
  .pico-orange-glass-modal {
    max-width: 500px;
    width: 100%;
    margin: 0;
    padding: 0;
    overflow: hidden;
  }
  .modal-header-hull {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 16px 24px;
    border-bottom: 1px solid var(--pico-border-color);
    background: transparent;
    margin: 0;
  }
  .modal-title-heading {
    margin: 0;
    font-size: 1.1rem;
    font-weight: 600;
  }
  .close-modal-x {
    background: transparent !important;
    border: none !important;
    box-shadow: none !important;
    color: var(--pico-muted-color) !important;
    cursor: pointer;
    font-size: 1.1rem;
    padding: 4px !important;
  }
  .modal-form-body {
    padding: 24px;
  }
  .field-heading {
    font-size: 0.75rem !important;
    text-transform: uppercase !important;
    letter-spacing: 0.05em !important;
    font-weight: 700 !important;
    color: var(--pico-muted-color) !important;
    margin-bottom: 0.35rem !important;
    display: block;
  }
  .orange-select-field {
    margin-bottom: 1rem !important;
    background-color: rgb(
      from var(--pico-form-element-background-color) r g b / 0.6
    );
    transition:
      border-color 0.2s ease,
      box-shadow 0.2s ease;
    outline: none !important;
  }
  .orange-select-field:active,
  .orange-select-field:focus,
  .orange-select-field:hover {
    border: var(--pico-border-width) solid var(--pico-form-element-color);
    outline: none !important;
    box-shadow: none !important;
  }
  .orange-select-field.compact {
    margin-bottom: 0.5rem !important;
    font-size: 0.85rem;
    padding: 6px 8px;
    outline: none !important;
    box-shadow: none !important;
  }
  .steppers-inline-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 20px;
    margin: 1.25rem 0;
  }
  .form-section-divider {
    border-color: var(--pico-muted-color);
    margin: 1.25rem 0;
  }
  .conditional-settings-panel {
    background-color: var(--pico-form-element-disabled-background-color);
    border: 1px solid var(--pico-border-color);
    padding: 1rem;
    border-radius: var(--pico-border-radius);
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
    height: 38px;
    padding: 2px;
    border-radius: 4px;
    background: rgba(255, 255, 255, 0.05);
    border: 1px solid var(--pico-border-color);
    cursor: pointer;
  }
  .pids-selection-grid {
    display: flex;
    flex-direction: column;
    gap: 12px;
  }
  .static-presets-notice {
    border: 1px dashed var(--pico-border-color);
    padding: 1rem;
    border-radius: var(--pico-border-radius);
    text-align: center;
    color: var(--pico-muted-color);
    margin-bottom: 1.5rem;
  }
  .modal-footer-actions {
    display: flex;
    justify-content: flex-end;
    gap: 12px;
  }
  .modal-footer-actions button {
    width: auto;
    padding: 8px 22px;
    font-size: 0.85rem;
    font-weight: 600;
  }
  .apply-btn {
    border: var(--pico-border-width) solid var(--pico-form-element-border-color);
    background-color: rgb(
      from var(--pico-form-element-background-color) r g b / 0.6
    );
  }
  .apply-btn:focus,
  .apply-btn:active,
  .apply-btn:hover {
    border: var(--pico-border-width) solid var(--pico-form-element-color);
    outline: none !important;
    box-shadow: none !important;
  }
  .apply-btn:active {
    background-color: rgba(255, 255, 255, 0.1) !important;
    transition: background-color 0s !important;
  }
  .animate-fade-in {
    animation: modalBoxFadeIn 0.18s ease-out forwards;
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
