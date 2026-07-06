<script lang="ts">
  import { canStore } from "$lib/canStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import ToggleSwitch from "$lib/components/ToggleSwitch.svelte";

  let statusColor = "oklch(1 0 0)";
  let isRecording = $derived(canStore.isRecording);
</script>

<div
  class="dashboard-card recorder-header"
  style="--module-accent: {statusColor};"
>
  <div class="recorder-header-title-row">
    <h2>Recorder</h2>
    <p class="status-subtitle">{isRecording ? "Recording" : "Not Recording"}</p>
  </div>

  <div class="recorder-header-actions">
    <button
      type="button"
      class="btn btn-record"
      onclick={() => (canStore.isRecording = !canStore.isRecording)}
    >
      <Icon name={isRecording ? "square" : "circle-solid"} size={16} />
      {isRecording ? "Stop Recording" : "Record"}
    </button>

    <button
      type="button"
      class="btn btn-performance"
      // onclick={}
    >
      <Icon name="stopwatch" size={16} />
      Performance test – 0-100 km/h
    </button>

    <div class="autorecord-wrapper">
      <ToggleSwitch
        label="Auto Record"
        subLabel="Start recording when car is started"
        disabled={!canStore.wsCanStatus?.canConnected}
      />
    </div>

    <div class="autorecord-wrapper">
      <ToggleSwitch
        label="Log Data"
        subLabel="Start polling and logging data from the car"
        checked={canStore.obd2Status?.continuous_running}
        onchange={(newState: boolean) => {
          canStore.setContinuousPolling(newState);
        }}
        disabled={!canStore.wsCanStatus?.canConnected}
      />
    </div>
  </div>
</div>

<style>
  /* --- 1. HEADER CONTAINER --- */
  .recorder-header {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 1.5rem;
    width: 100%;
    padding: 1.5rem;
    margin-bottom: 2rem;
    box-sizing: border-box;
  }

  .dashboard-card.recorder-header {
    border: none !important;
  }

  .dashboard-card.recorder-header:hover {
    box-shadow: none;
    border-color: var(--pico-muted-border-color) !important;
  }

  /* --- 2. TITLE ROW --- */
  .recorder-header-title-row {
    display: flex;
    align-items: baseline;
    gap: 1.5rem;
    flex-wrap: wrap;
  }

  .recorder-header-title-row h2 {
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

  /* --- 3. ACTIONS ROW (Flexbox Responsive Magic) --- */
  .recorder-header-actions {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 1rem; /* Space between rows and columns */
    width: 100%;
  }

  /* The buttons share a row if there is room (desktop), but wrap on mobile */
  .recorder-header-actions > .btn-record,
  .recorder-header-actions > .btn-performance {
    flex: 1 1 250px; /* Grow: 1, Shrink: 1, Base width: 250px */
  }

  /* The toggle wrapper forces a line break by demanding 100% of the width */
  .autorecord-wrapper {
    flex: 1 1 100%;
    margin-top: 0.5rem; /* A little extra breathing room above the toggle */
  }

  /* --- 4. GENERIC BUTTONS --- */
  .btn {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    border: var(--pico-border-width) solid var(--pico-form-element-border-color);
    background-color: rgb(
      from var(--pico-form-element-background-color) r g b / 0.6
    );
    margin: 0;
    padding: 0 1rem;
    cursor: pointer;
  }

  .btn :global(svg) {
    margin: 0 !important;
  }

  /* --- 5. SPECIFIC BUTTON STYLES --- */
  .btn-record {
    --color: 57, 241, 166;
    background-color: rgba(var(--color), 0.1);
    border: none;
    color: rgb(var(--color));
    height: 54px;
    border-radius: 8px;
    font-weight: 500;
  }

  .btn:hover {
    background-color: rgba(var(--color), 0.2);
  }

  .btn:active {
    transform: scale(0.98);
  }

  .btn-performance {
    --color: 149, 144, 130;
    background-color: rgba(var(--color), 0.1);
    border: none;
    color: rgb(var(--color));
    height: 54px;
    border-radius: 8px;
    font-weight: 500;
  }

  /* --- 6. EXTERNAL CONTAINERS --- */
  .record-container {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
    gap: 1rem;
    width: 100%;
  }

  /* --- 7. MOBILE POLISH --- */
  @media (max-width: 500px) {
    .recorder-header {
      padding: 1rem;
    }
  }
</style>
