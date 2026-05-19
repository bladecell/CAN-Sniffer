<script lang="ts">
  import { canStore } from "$lib/canStore.svelte.js";
  import TelemetryStepper from "./TelemetryStepper.svelte";
  import type { PidGridItem } from "$lib/types";
  import { untrack } from "svelte";

  interface Props {
    isOpen: boolean;
    item: PidGridItem | null;
    isNewCard?: boolean;
    previewItem: any;
    onSave: (updatedItem: any) => void;
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

  let localItem = $state<any>({
    id: "",
    cardType: "pid",
    pid: 0,
    displayMode: "card",
    w: 10,
    h: 7,
  });

  // Structural safety gate to ignore cascade updates during modal lifecycle changes
  let internalSync = $state(false);

  // Synchronize incoming data structures safely
  $effect(() => {
    // If the modal isn't open, completely lock down the reactive sync operations
    if (!isOpen) return;

    if (item) {
      internalSync = true;
      localItem = JSON.parse(JSON.stringify(item));
      previewItem = JSON.parse(JSON.stringify(item));
      setTimeout(() => {
        internalSync = false;
      }, 5);
    } else if (isNewCard) {
      internalSync = true;
      const immediateId = crypto.randomUUID();

      localItem = {
        id: immediateId,
        cardType: "pid",
        pid: canStore.pidDefinitions[0]?.pid || 0,
        displayMode: "card",
        w: 10,
        h: 7,
      };

      previewItem = { ...localItem };

      setTimeout(() => {
        internalSync = false;
      }, 5);
    }
  });

  // Handle active modification live previews
  $effect(() => {
    const currentW = localItem.w;
    const currentH = localItem.h;
    const currentPid = localItem.pid;
    const currentDisplayMode = localItem.displayMode;
    const currentCardType = localItem.cardType;

    untrack(() => {
      // Direct escape hatch: do not update anything if sync is active or modal is closing
      if (!isOpen || isNewCard || !item || internalSync) return;

      previewItem = {
        ...localItem,
        w: currentW,
        h: currentH,
        pid: currentPid,
        displayMode: currentDisplayMode,
        cardType: currentCardType,
      };
    });
  });

  function handleCardTypeChange() {
    if (localItem.cardType === "pid") {
      localItem.w = 10;
      localItem.h = 7;
      if (!localItem.displayMode) localItem.displayMode = "card";
      if (!localItem.pid) localItem.pid = canStore.pidDefinitions[0]?.pid || 0;
    } else if (localItem.cardType === "battery") {
      localItem.w = 6;
      localItem.h = 6;
    } else if (localItem.cardType === "dtcs") {
      localItem.w = 10;
      localItem.h = 8;
    } else if (localItem.cardType === "overview") {
      localItem.w = 10;
      localItem.h = 6;
    }
  }

  function handleCancel() {
    internalSync = true; // Lock all reactive effects immediately
    previewItem = null; // Safely clear the preview pipeline
    onClose(); // Close modal synchronously
  }

  function handleSubmit(e: SubmitEvent) {
    e.preventDefault();
    internalSync = true; // Lock down all reactive effects immediately

    // Extract a deep copy of the verified local parameters
    const finalData = JSON.parse(JSON.stringify(localItem));

    // Clear the preview assignment right before submitting to the dashboard
    previewItem = null;

    // Fire the save sequence synchronously to prevent reactive loop timing issues
    onSave(finalData);
  }
</script>

{#if isOpen}
  <div class="custom-modal-backdrop" onclick={handleCancel}>
    <article
      onclick={(e) => e.stopPropagation()}
      class="pico-orange-glass-modal"
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
          <option value="battery">System Battery Monitor Block</option>
          <option value="dtcs">Diagnostic Trouble Codes (DTC)</option>
          <option value="overview">Overview</option>
        </select>

        <div class="steppers-inline-grid">
          <div class="stepper-field-wrapper">
            <span class="field-heading">Grid Width Columns</span>
            <TelemetryStepper bind:value={localItem.w} min={4} max={60} />
          </div>

          <div class="stepper-field-wrapper">
            <span class="field-heading">Grid Height Rows</span>
            <TelemetryStepper bind:value={localItem.h} min={2} max={50} />
          </div>
        </div>

        <hr class="form-section-divider" />

        {#if localItem.cardType === "pid"}
          <div class="conditional-settings-panel animate-fade-in">
            <div class="panel-section-title">PID Sensor Parameters</div>

            <label for="pid-sensor-select" class="field-heading"
              >Target Vehicle Sensor Stream</label
            >
            <select
              id="pid-sensor-select"
              bind:value={localItem.pid}
              class="orange-select-field"
            >
              {#each canStore.pidDefinitions as definition}
                <option value={definition.pid}>
                  0x{definition.pid.toString(16).toUpperCase()} — {definition.name}
                </option>
              {/each}
            </select>

            <label for="pid-visualization-select" class="field-heading"
              >Widget Visualization Mode</label
            >
            <select
              id="pid-visualization-select"
              bind:value={localItem.displayMode}
              class="orange-select-field"
            >
              <option value="card">Digit Badge</option>
              <option value="chart">Chart</option>
              <option value="gauge">Gauge Display</option>
              <option value="bar">Horizon Bar</option>
            </select>
          </div>
        {:else}
          <div class="static-presets-notice animate-fade-in">
            <small>
              The selected <strong>{localItem.cardType.toUpperCase()}</strong> interface
              component runs on global canvas states. No extra properties are needed.
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
  .orange-select-field {
    margin-bottom: 1rem !important;
    color: var(--pico-color) !important;
    border: var(--pico-border-width) solid var(--pico-form-element-border-color);

    /* Mirror the visual transparency depth used on your master dialog frame */
    background-color: rgba(30, 30, 35, 0.65) !important;
    backdrop-filter: blur(10px) saturate(1.2) !important;
    -webkit-backdrop-filter: blur(10px) saturate(1.2) !important;
  }

  .orange-select-field:focus,
  .orange-select-field:hover {
    border-color: var(--pico-form-element-color) !important;
    background-color: rgba(35, 35, 40, 0.8) !important;
    box-shadow: none;
  }

  /* Target the individual selection option slots inside the picker pane drop down tree.
     Since the OS prevents transparency here, we match the solid dark tone of your dashboard base. */
  .orange-select-field option {
    background-color: #1c1c21 !important; /* Solid dark theme matching your glass tint */
    color: var(--pico-color) !important;
    padding: 8px;
  }

  /* Optional clean up for your nested conditional panel area */
  .conditional-settings-panel {
    background-color: rgba(255, 255, 255, 0.02);
    border: 1px solid rgba(255, 255, 255, 0.05);
    padding: 1rem;
    border-radius: var(--pico-border-radius);
    margin-bottom: 1.5rem;
  }

  /* All remaining dashboard/modal styles remain completely unchanged */
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
  }
  .pico-orange-glass-modal {
    backdrop-filter: var(
      --backdrop-filter,
      blur(10px) saturate(1.2)
    ) !important;
    -webkit-backdrop-filter: var(
      --backdrop-filter,
      blur(10px) saturate(1.2)
    ) !important;
    background-color: rgba(
      24,
      24,
      28,
      0.75
    ) !important; /* Stabilized glass layout backdrop base tint */
    border: var(--pico-border-width) solid var(--pico-form-element-border-color);
    max-width: 500px;
    width: 100%;
    margin: 0;
    padding: 0;
    box-shadow: var(--pico-box-shadow) !important;
    border-radius: var(--pico-border-radius);
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
    color: var(--pico-h1-color);
  }
  .close-modal-x {
    background: transparent !important;
    border: none !important;
    box-shadow: none !important;
    color: var(--pico-muted-color) !important;
    cursor: pointer;
    font-size: 1.1rem;
    padding: 4px !important;
    margin: 0 !important;
    width: auto !important;
  }
  .close-modal-x:hover {
    color: var(--pico-color) !important;
  }
  .modal-form-body {
    padding: 24px;
    margin: 0;
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
  .steppers-inline-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 20px;
    margin: 1.25rem 0;
  }
  .stepper-field-wrapper {
    display: flex;
    flex-direction: column;
  }
  .form-section-divider {
    border-color: var(--pico-muted-color);
    margin: 1.25rem 0;
  }
  .panel-section-title {
    font-size: 0.7rem;
    text-transform: uppercase;
    letter-spacing: 0.06em;
    font-weight: 800;
    color: var(--pico-h2-color);
    margin-bottom: 0.85rem;
  }
  .static-presets-notice {
    background: transparent;
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
    margin: 0;
    padding: 0;
    background: transparent;
    border: none;
  }
  .modal-footer-actions button {
    margin: 0;
    width: auto;
    padding: 8px 22px;
    font-size: 0.85rem;
    font-weight: 600;
  }
  .animate-fade-in {
    animation: modalBoxFadeIn 0.18s ease-out forwards;
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
