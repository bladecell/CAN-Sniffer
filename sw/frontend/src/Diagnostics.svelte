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

<div class="card bg-base-200/50 backdrop-blur-md border border-base-300 mb-8 overflow-hidden shadow-sm hover:border-base-content/20 transition-colors" style="--module-accent: {statusColor};">
  <div class="card-body flex flex-col md:flex-row justify-between items-start md:items-center gap-6 p-4 md:p-6 relative">
    <!-- Optional: Adding a subtle colored left border hint based on status -->
    <div class="absolute left-0 top-0 bottom-0 w-1" style="background-color: var(--module-accent)"></div>
    
    <div class="flex flex-col md:flex-row items-baseline gap-2 md:gap-6 w-full">
      <h2 class="text-2xl font-bold m-0">Diagnostic Trouble Codes</h2>
      <p class="text-xs md:text-sm font-bold uppercase tracking-wider m-0" style="color: var(--module-accent)">{statusDescription}</p>
    </div>

    <div class="flex flex-col sm:flex-row w-full md:w-auto gap-3 min-w-fit">
      <button
        type="button"
        class="btn btn-outline btn-success flex-1 sm:flex-none gap-2 hover:bg-success/20 hover:text-success hover:border-success/50"
        onclick={() => canStore.requestDTC()}
      >
        <Icon name="magnifying-glass" size={16} />
        Scan Vehicle
      </button>
      <button 
        type="button" 
        class="btn btn-outline btn-error flex-1 sm:flex-none gap-2 hover:bg-error/20 hover:text-error hover:border-error/50" 
        onclick={canStore.clearDTCs}
      >
        <Icon name="trash" size={16} />
        Clear Codes
      </button>
    </div>
  </div>
</div>

<div class="grid grid-cols-1 lg:grid-cols-2 xl:grid-cols-3 gap-4 w-full">
  {#each activeFaults as fault}
    <DTCFault
      code={fault.code}
      description={fault.description}
      mode={fault.mode}
    />
  {/each}
</div>
