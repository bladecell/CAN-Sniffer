<script>
  import { canStore } from "$lib/canStore.svelte.js";
  import { onMount } from "svelte";
  import { on } from "svelte/events";
  import { CanStore } from "../canStore.svelte";

  let {
    pids = [0x0c, 0x2206, 0x04],
    color = "#3b82f6",
    moveStart = null,
    ...rest
  } = $props();

  onMount(async () => {
    await canStore.requestVin();
    await canStore.requestDTC();
  });

  const system = $derived(canStore.system || {});
  const vin = $derived(canStore.vin || "--- UNKNOWN ---");
  const battery = $derived(
    canStore.wsCanStatus?.battery_voltage.toFixed(1) || "0.0",
  );
  const isOnline = $derived(
    canStore.wsCanStatus?.state === "Connected" ? true : false,
  );
  const state = $derived(canStore.wsCanStatus?.state);
  const utilization = $derived(canStore.wsCanStatus?.utilization ?? 0);
  const dtcCount = $derived(canStore.totalDTCs ?? 0);

  const statusColor = $derived(
    isOnline ? "var(--pico-ins-color)" : "var(--pico-del-color)",
  );

  function getPidData(id) {
    const def = canStore.pidDefinitions.find((d) => d.pid === id);
    const data = canStore.pids.get(id);
    return {
      label: def?.name || id,
      value: data ? data.value.toFixed(0) : "---",
      unit: def?.unit || "",
      color: `#${def?.color.toString(16).padStart(6, "0")}`,
    };
  }

  let longPressTimer;

  function handlePointerDown(e, moveStart) {
    longPressTimer = setTimeout(() => moveStart(e), 300);
  }

  function handlePointerUp() {
    clearTimeout(longPressTimer);
  }
</script>

<article class="overview-card" style="--brand-color: {color}" {...rest}>
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <header
    class="card-header"
    onpointerdown={moveStart ? (e) => handlePointerDown(e, moveStart) : null}
    onpointerup={handlePointerUp}
    onpointermove={handlePointerUp}
    style="cursor: {moveStart ? 'grab' : 'default'}; touch-action: pan-y;"
  >
    <div class="titles">
      <div class="label">System Overview</div>
      <div class="vin-text">{vin}</div>
    </div>

    <div class="header-badges">
      <!-- Utilization Badge -->
      <div class="badge util-badge">
        <span class="badge-label">Util</span>
        <span class="badge-value">{utilization}%</span>
      </div>
      <!-- Connection Status Badge -->
      <div class="badge status-badge" style:--status-color={statusColor}>
        <div class="status-dot"></div>
        <span class="badge-value">{state}</span>
      </div>
    </div>
  </header>

  <div class="card-body">
    <!-- Main Stats: Battery and DTC -->
    <div class="stats-row">
      <div class="stat-item">
        <span class="label">Battery</span>
        <div class="value">
          <span class="number">{battery}</span>
          <span class="unit">V</span>
        </div>
      </div>
      <div class="stat-item">
        <span class="label">Diagnostics</span>
        <div class="value">
          <span class="number" class:error={dtcCount > 0}>{dtcCount}</span>
          <span class="unit">DTCs</span>
        </div>
      </div>
    </div>

    <!-- PIDs Footer Row -->
    <div class="pid-footer">
      {#each pids as pidId}
        {@const pid = getPidData(pidId)}
        <div
          class="pid-badge"
          style="background: color-mix(in srgb, {pid.color} 5%, transparent);"
        >
          <span class="pid-label">{pid.label}</span>
          <span class="pid-value">{pid.value}<small>{pid.unit}</small></span>
        </div>
      {/each}
    </div>
  </div>
</article>

<style>
  .overview-card {
    width: 100%;
    height: 100%;
    box-sizing: border-box;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    padding: 16px;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 12px;
    background: color-mix(in srgb, var(--brand-color) 5%, transparent);
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  /* HEADER & BADGES */
  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 16px;
    background: none;
    border: none;
  }

  .header-badges {
    display: flex;
    gap: 8px;
    align-items: center;
  }

  .badge {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 4px 8px;
    background: color-mix(
      in srgb,
      var(--pico-muted-border-color) 20%,
      transparent
    );
    border-radius: 6px;
    border: 1px solid var(--pico-muted-border-color);
  }

  .badge-label {
    font-size: 0.55rem;
    font-weight: 700;
    text-transform: uppercase;
    color: var(--pico-muted-color);
  }

  .badge-value {
    font-family: var(--pico-font-family-monospace);
    font-size: 0.7rem;
    font-weight: 800;
    color: var(--pico-contrast);
  }

  /* STATUS DOT ANIMATION */
  .status-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: var(--status-color);
    box-shadow: 0 0 8px var(--status-color);
    animation: pulse-glow 2s infinite alternate;
  }

  /* TITLES */
  .titles {
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  .label {
    font-size: 0.7rem;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: var(--pico-muted-color);
    font-weight: 500;
  }

  .vin-text {
    font-family: var(--pico-font-family-monospace);
    font-size: 1.2rem;
    font-weight: 700;
    color: var(--pico-contrast);
  }

  /* BODY STATS */
  .card-body {
    display: flex;
    flex-direction: column;
    gap: 20px;
  }

  .stats-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 16px;
  }

  .stat-item {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .value {
    display: flex;
    align-items: baseline;
    gap: 4px;
  }

  .number {
    font-size: 2rem;
    font-weight: 700;
    line-height: 1;
    font-family: var(--pico-font-family-monospace);
  }

  .number.error {
    color: var(--pico-del-color);
  }

  .unit {
    font-size: 0.9rem;
    color: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }

  /* PID FOOTER */
  .pid-footer {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 8px;
  }

  .pid-badge {
    padding: 8px;
    border-radius: 8px;
    text-align: center;
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  .pid-label {
    font-size: 0.55rem;
    text-transform: uppercase;
    color: var(--pico-muted-color);
    font-weight: 600;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .pid-value {
    font-family: var(--pico-font-family-monospace);
    font-size: 1rem;
    font-weight: 700;
  }

  .pid-value small {
    font-size: 0.6rem;
    margin-left: 2px;
    color: var(--pico-muted-color);
  }

  @keyframes pulse-glow {
    0% {
      opacity: 1;
      box-shadow: 0 0 1px var(--status-color);
    }
    50% {
      opacity: 0.85;
      box-shadow: 0 0 6px var(--status-color);
    }
    100% {
      opacity: 1;
      box-shadow: 0 0 8px var(--status-color);
    }
  }

  .overview-card:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  }
</style>
