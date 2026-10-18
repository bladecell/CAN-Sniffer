<script lang="ts">
import { connectionStore } from "$lib/connectionStore.svelte";
import { vehicleStore } from "$lib/vehicleStore.svelte";
import { recordingStore } from "$lib/recordingStore.svelte";


  import Icon from "$lib/Icon.svelte";
  import Switch from "$lib/components/Switch.svelte";

  let statusColor = "oklch(1 0 0)";
  let isRecording = $derived(recordingStore.isRecording);
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
      onclick={() => (recordingStore.isRecording = !recordingStore.isRecording)}
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
      <Switch
        label="Auto Record"
        subLabel="Start recording when car is started"
        disabled={connectionStore.wsCanStatus?.state != "CAN Connected"}
      />
    </div>

    <div class="autorecord-wrapper">
      <Switch
        label="Auto poll"
        subLabel="Keep polling the car and resume after ignition off/on"
        checked={vehicleStore.obd2Status?.auto_poll}
        onchange={(newState: boolean) => {
          vehicleStore.setAutoPoll(newState);
        }}
        disabled={connectionStore.wsCanStatus?.state != "CAN Connected"}
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

  .btn-record {
    --color: 57, 241, 166;
    height: 54px;
    border-radius: 8px;
    font-weight: 500;
  }

  .btn-performance {
    --color: 149, 144, 130;
    height: 54px;
    border-radius: 8px;
    font-weight: 500;
  }

  /* --- 7. MOBILE POLISH --- */
  @media (max-width: 500px) {
    .recorder-header {
      padding: 1rem;
    }
  }
</style>
