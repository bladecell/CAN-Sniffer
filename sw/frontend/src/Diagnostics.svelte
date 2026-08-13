<script lang="ts">
  import type { DtcModeData, DTCFaultProps, singleDtc } from "$lib/types";
  import { canStore } from "./lib/canStore.svelte";
  import DTCFault from "./lib/components/DTCFault.svelte";
  import Icon from "$lib/Icon.svelte";

  const confirmedCodes = $derived(
    canStore?.dtc?.confirmed || {},
  ) as DtcModeData;

  const pendingCodes = $derived(canStore?.dtc?.pending || {}) as DtcModeData;

  const activeFaults = $derived.by((): DTCFaultProps[] => {
    const confirmed = confirmedCodes.dtc.map((code: singleDtc) => ({
      code: code.dtc,
      description: code.description || "No description provided",
      mode: 3,
    }));

    const pending = pendingCodes.dtc.map((code: singleDtc) => ({
      code: code.dtc,
      description: code.description || "No description provided",
      mode: 7,
    }));

    return [...confirmed, ...pending];
  });

  const status = $derived.by((): "healthy" | "warn" | "malfunction" => {
    if (confirmedCodes.dtc_count > 0) return "malfunction";
    if (pendingCodes.dtc_count > 0) return "warn";
    return "healthy";
  });

  const statusColor = $derived(
    status === "healthy"
      ? "var(--normal-color)"
      : status === "warn"
        ? "var(--warning-color)"
        : "var(--error-color)",
  );

  const statusDescription = $derived(
    status === "healthy"
      ? "POWERTRAIN SYSTEMS HEALTHY"
      : status === "warn"
        ? `${pendingCodes.dtc_count} PENDING FAULT${pendingCodes.dtc_count > 1 ? "S" : ""}`
        : `${confirmedCodes.dtc_count} ACTIVE FAULT${confirmedCodes.dtc_count > 1 ? "S" : ""}`,
  );
</script>

//TODO: add vizual indication that can bus is not connected and card that no dtc
is detected

<div class="dashboard-card dtc-header" style="--module-accent: {statusColor};">
  <div class="dtc-header-title-row">
    <h2>Diagnostic Trouble Codes</h2>
    <p class="status-subtitle">{statusDescription}</p>
  </div>

  <div class="dtc-header-actions">
    <button
      type="button"
      class="btn btn-scan"
      onclick={() => canStore.requestDTC()}
    >
      <Icon name="magnifying-glass" size={16} />
      Scan Vehicle
    </button>
    <button type="button" class="btn btn-clear" onclick={canStore.clearDTCs}>
      <Icon name="trash" size={16} />
      Clear Codes
    </button>
  </div>
</div>

<div class="dtc-container">
  {#each activeFaults as fault}
    <DTCFault
      code={fault.code}
      description={fault.description}
      mode={fault.mode}
    />
  {/each}
</div>

<style>
  .dtc-header {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 1.5rem; /* Space between text row and buttons */
    width: 100%;
    height: 150px;
    padding: 1.5rem; /* Adjust to match your card padding */
    margin-bottom: 2rem;
    box-sizing: border-box;
  }

  .dashboard-card.dtc-header {
    border: none !important;
  }

  .dashboard-card.dtc-header:hover {
    box-shadow: none;
    border-color: var(--pico-muted-border-color) !important;
  }

  .dtc-header-title-row {
    display: flex;
    align-items: baseline;
    gap: 1.5rem;
    flex-wrap: wrap;
  }

  .dtc-header-title-row h2 {
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

  .dtc-header-actions {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1rem;
    width: 100%;
    max-width: 400px;
  }

  .dtc-header-actions .btn {
    width: 100%;
  }

  @media (max-width: 500px) {
    .dtc-header-actions {
      grid-template-columns: 1fr;
    }
    .dtc-header {
      height: 250px;
      padding: 1rem;
    }
  }

  .dtc-container {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
    gap: 1rem;
    width: 100%;
  }

  .btn-scan {
    --color: 57, 241, 166;
  }

  .btn-clear {
    --color: 231, 75, 26;
  }
  /* .btn:focus,
  .btn:active,
  .btn:hover {
    border: var(--pico-border-width) solid var(--pico-form-element-color);
    outline: none !important;
    box-shadow: none !important;
  }
  .btn:active {
    background-color: rgba(255, 255, 255, 0.1) !important;
    transition: background-color 0s !important;
  } */
</style>
