<script lang="ts">
  import { canStore } from "$lib/canStore.svelte";
  import type { SpecialGridItem, DtcModeData } from "$lib/types";
  import Icon from "../Icon.svelte";

  interface Props {
    item: SpecialGridItem;
    [key: string]: any;
  }

  let { item, ...rest }: Props = $props();

  // --- RELIABLE NESTED EXTRACTION RIG ---
  const confirmedCodes = $derived(
    canStore?.dtc?.confirmed || {},
  ) as DtcModeData;

  const pendingCodes = $derived(canStore?.dtc?.pending || {}) as DtcModeData;

  const permanentCodes = $derived(
    canStore?.dtc?.permanent || {},
  ) as DtcModeData;

  // --- SEVERITY, ICONS & STATUS RUNES ---
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

<article
  class="dashboard-card dtc-card"
  class:status-warn={status === "warn"}
  class:status-error={status === "malfunction"}
  style="--module-accent: {statusColor};"
  {...rest}
>
  <header class="dashboard-card-header">
    <div class="dashboard-card-icon">
      <Icon name={"engine"} size={32} />
    </div>
    <div class="dashboard-card-titles">
      <div class="dashboard-card-label">Diagnostic Troube Codes</div>
      <div class="dashboard-card-subtitle">{statusDescription}</div>
    </div>
  </header>

  <div class="dashboard-card-body dtc-content-body">
    {#if confirmedCodes.dtc_count === 0 && permanentCodes.dtc_count === 0 && pendingCodes.dtc_count === 0}
      <div class="empty-log-state">
        <span>No diagnostic trouble codes logged in ECU memory bank.</span>
      </div>
    {:else}
      <div class="codes-list-container">
        <div
          class="code-column"
          class:empty-column={confirmedCodes.dtc_count === 0}
        >
          <div class="group-header error-text">
            <Icon name="alert-circle" size={14} />
            {confirmedCodes.dtc_count} Confirmed
          </div>
          <div class="column-viewport">
            <div
              class="chips-flex"
              class:marquee-active={confirmedCodes.dtc_count > 3}
              style="--item-count: {confirmedCodes.dtc_count};"
            >
              {#each confirmedCodes.dtc as code}
                <span class="dtc-chip confirmed-chip">{code.dtc}</span>
              {:else}
                <span class="column-placeholder">—</span>
              {/each}

              {#if confirmedCodes.dtc_count > 3}
                {#each confirmedCodes.dtc as code}
                  <span
                    class="dtc-chip confirmed-chip duplicate-tag"
                    aria-hidden="true">{code.dtc}</span
                  >
                {/each}
              {/if}
            </div>
          </div>
        </div>

        <!-- <div
          class="code-column"
          class:empty-column={permanentCodes.length === 0}
        >
          <div class="group-header permanent-text">
            <Icon name="lock" size={14} />
            {permanentCodes.length} Permanent
          </div>
          <div class="column-viewport">
            <div
              class="chips-flex"
              class:marquee-active={permanentCodes.length > 3}
              style="--item-count: {permanentCodes.length};"
            >
              {#each permanentCodes as code}
                <span class="dtc-chip permanent-chip">{code}</span>
              {:else}
                <span class="column-placeholder">—</span>
              {/each}

              {#if permanentCodes.length > 3}
                {#each permanentCodes as code}
                  <span
                    class="dtc-chip permanent-chip duplicate-tag"
                    aria-hidden="true">{code}</span
                  >
                {/each}
              {/if}
            </div>
          </div>
        </div> -->

        <div
          class="code-column"
          class:empty-column={pendingCodes.dtc_count === 0}
        >
          <div class="group-header warn-text">
            <Icon name="clock" size={14} />
            {pendingCodes.dtc_count} Pending
          </div>
          <div class="column-viewport">
            <div
              class="chips-flex"
              class:marquee-active={pendingCodes.dtc_count > 3}
              style="--item-count: {pendingCodes.dtc_count};"
            >
              {#each pendingCodes.dtc as code}
                <span class="dtc-chip pending-chip">{code.dtc}</span>
              {:else}
                <span class="column-placeholder">—</span>
              {/each}

              {#if pendingCodes.dtc_count > 3}
                {#each pendingCodes.dtc as code}
                  <span
                    class="dtc-chip pending-chip duplicate-tag"
                    aria-hidden="true">{code.dtc}</span
                  >
                {/each}
              {/if}
            </div>
          </div>
        </div>
      </div>
    {/if}
  </div>
</article>

<style>
  /* --- CONFIGURABLE CONSTANTS LAYER --- */
  :root {
    --seconds-per-item: 5s;
    --scroll-delay: 5s;
  }

  .dtc-card {
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .status-warn {
    background: color-mix(in srgb, #f59e0b 5%, rgba(25, 25, 30, 0.55));
  }

  .status-error {
    background: color-mix(in srgb, #ef4444 5%, rgba(25, 25, 30, 0.55));
  }

  .empty-log-state {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 100%;
    min-height: 60px;
    text-align: center;
    font-size: 0.75rem;
    color: var(--pico-muted-color);
    font-style: italic;
  }

  .codes-list-container {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
    width: 100%;
    height: 100%;
  }

  .code-column {
    display: flex;
    flex-direction: column;
    gap: 10px;
    background: rgba(0, 0, 0, 0.15);
    padding: 10px;
    border-radius: 8px;
    border: 1px solid rgba(255, 255, 255, 0.02);
    min-height: 0;
    height: 100%;
  }

  .empty-column {
    opacity: 0.5;
  }

  .group-header {
    display: grid;
    grid-template-columns: 1fr 4fr;
    align-items: center;
    align-self: flex-start;
    width: 100%;
    gap: 6px;
    font-size: 0.62rem;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    padding-bottom: 6px;
    flex-shrink: 0;
  }

  .error-text {
    color: #ef4444;
  }
  .permanent-text {
    color: #a1a1aa;
  }
  .warn-text {
    color: #f59e0b;
  }

  .column-viewport {
    width: 100%;
    flex: 1 1 auto;
    overflow: hidden;
    position: relative;
    min-height: 0;
  }

  .chips-flex {
    display: flex;
    flex-direction: column;
    gap: 6px;
    overflow: visible !important;
    position: relative;
    top: 0;
  }

  .marquee-active {
    animation: verticalMarquee calc(var(--item-count) * var(--seconds-per-item))
      linear var(--scroll-delay) infinite;
  }

  .marquee-active:hover {
    animation-play-state: paused;
  }

  @keyframes verticalMarquee {
    0%,
    10% {
      transform: translateY(0);
    }
    90%,
    100% {
      transform: translateY(
        calc(-1 * ((var(--item-count) * 29px) + (var(--item-count) * 6px)))
      );
    }
  }

  .dtc-chip {
    font-family: var(--pico-font-family-monospace);
    font-size: 0.75rem;
    font-weight: 700;
    padding: 5px 8px;
    border-radius: 4px;
    letter-spacing: 0.05em;
    text-align: center;
    width: 100%;
    box-sizing: border-box;
    height: 29px;
    flex-shrink: 0;
  }

  .column-placeholder {
    font-size: 0.75rem;
    color: var(--pico-muted-color);
    text-align: center;
    padding: 4px 0;
  }

  .confirmed-chip {
    background: rgba(239, 68, 68, 0.16);
    color: #f87171;
    border: 1px solid rgba(239, 68, 68, 0.3);
  }
  .permanent-chip {
    background: rgba(161, 161, 170, 0.1);
    color: #d4d4d8;
    border: 1px solid rgba(161, 161, 170, 0.2);
  }
  .clip-chip,
  .pending-chip {
    background: rgba(245, 158, 11, 0.12);
    color: #fbbf24;
    border: 1px solid rgba(245, 158, 11, 0.2);
  }

  @media (max-width: 500px) {
    .codes-list-container {
      grid-template-columns: 1fr;
      gap: 16px;
    }
    .column-viewport {
      height: auto;
      overflow: visible;
    }
    .marquee-active {
      animation: none;
    }
  }
</style>
