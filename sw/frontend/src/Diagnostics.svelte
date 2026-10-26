<script lang="ts">
  import { onDestroy } from "svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { dtcRows, dtcSummary, type DtcGroup, type DtcRow } from "$lib/diagnosticsView";
  import Icon from "$lib/Icon.svelte";
  import ViewHeader from "$lib/components/listDetail/ViewHeader.svelte";
  import StatusLine from "$lib/components/listDetail/StatusLine.svelte";
  import StateNote from "$lib/components/listDetail/StateNote.svelte";
  import MasterDetail from "$lib/components/listDetail/MasterDetail.svelte";

  const GROUP_ICON: Record<DtcGroup, string> = { confirmed: "triangle-exclamation", pending: "clock", permanent: "lock" };
  const GROUP_TONE: Record<DtcGroup, "crit" | "warn" | "perm"> = { confirmed: "crit", pending: "warn", permanent: "perm" };
  const GROUP_CHIP: Record<DtcGroup, string> = { confirmed: "c-crit", pending: "c-warn", permanent: "c-perm" };

  const rows = $derived(dtcRows(vehicleStore.dtc));
  const summary = $derived(dtcSummary(vehicleStore.dtc));
  const vin = $derived(vehicleStore.vin);
  const canConnected = $derived(connectionStore.wsCanStatus?.state === "CAN Connected");
  const armed = $derived(vehicleStore.clearState.kind === "armed");

  let scanning = $state(false);
  let selected = $state<string | null>(null);

  async function scan() {
    if (scanning) return;
    scanning = true;
    try {
      await vehicleStore.requestDTC();
    } finally {
      scanning = false;
    }
  }

  async function copyVin() {
    try {
      await navigator.clipboard.writeText(vin);
      alertStore.add("VIN copied", "success");
    } catch {
      alertStore.add("Could not copy the VIN", "error");
    }
  }

  const lookupUrl = (code: string) => `https://www.dtclookup.com/obd-ii/${encodeURIComponent(code.toLowerCase())}/`;

  onDestroy(vehicleStore.cancelClearDTCs);
</script>

<section class="rv-wrap">
  <ViewHeader title="Diagnostic Trouble Codes">
    {#snippet status()}
      {#if scanning}
        <StatusLine tone="muted" text="Scanning…" />
      {:else if !canConnected}
        <StatusLine tone="warn" icon="alert-circle" text="Vehicle not reachable" rest={rows.length ? "Codes are from the last scan." : "CAN bus not connected."} />
      {:else}
        <StatusLine
          tone={summary.tone}
          icon={summary.tone === "ok" ? "circle-check" : "alert-circle"}
          text={summary.text}
          rest={summary.rest}
        />
      {/if}
    {/snippet}

    {#snippet meta()}
      <p class="rv-meta">
        <span>VIN <span class="rv-vin">{vin || "not read yet"}</span></span>
        {#if vin}
          <button type="button" class="rv-btn sm" onclick={copyVin}>Copy VIN</button>
        {/if}
      </p>
    {/snippet}

    {#snippet actions()}
      <button type="button" class="rv-btn pri" aria-disabled={scanning} onclick={scan}>
        <Icon name="magnifying-glass" size={16} />
        Scan vehicle
      </button>
      <button
        type="button"
        class="rv-btn danger"
        class:fill-crit={armed}
        onclick={vehicleStore.pressClearDTCs}
        onblur={vehicleStore.cancelClearDTCs}
        onkeydown={(e) => {
          if (e.key === "Escape") vehicleStore.cancelClearDTCs();
          // A held Enter would fire repeated clicks: swallow them.
          else if (e.repeat) e.preventDefault();
        }}
        aria-disabled={vehicleStore.isClearing}
      >
        <Icon name="trash" size={16} />
        {armed ? "Confirm clear" : "Clear codes"}
      </button>
      <span class="sr-only" role="status" aria-live="polite">
        {armed ? "Press Confirm clear to clear trouble codes on the vehicle. Cancels in a few seconds, on Escape or when focus leaves." : ""}
      </span>
    {/snippet}
  </ViewHeader>

  {#if scanning && rows.length === 0}
    <StateNote title="Asking the ECU for codes" spinning>
      <p class="rv-muted">Reading modes 03, 07 and 10 one after another. This can take several seconds.</p>
    </StateNote>
  {:else if rows.length === 0 && !canConnected}
    <StateNote title="No answer from the vehicle" icon="alert-circle" tone="warn" role="alert">
      <p class="rv-muted">The CAN bus is not connected or the ignition is off. Check that the sniffer is plugged into the OBD-II port and the key is in the ON position, then scan again.</p>
      <div class="rv-acts">
        <button type="button" class="rv-btn pri" onclick={scan}>
          <Icon name="magnifying-glass" size={16} />
          Try again
        </button>
      </div>
    </StateNote>
  {:else if rows.length === 0}
    <StateNote title="No fault codes stored" icon="circle-check" tone="ok" role="status">
      <p class="rv-muted">The ECU reported zero codes in modes 03, 07 and 10. Scan again after a drive if the check-engine light was on.</p>
      <div class="rv-acts">
        <button type="button" class="rv-btn pri" onclick={scan}>
          <Icon name="magnifying-glass" size={16} />
          Scan vehicle
        </button>
      </div>
    </StateNote>
  {:else}
    <MasterDetail
      items={rows}
      keyOf={(r) => r.key}
      label={(r) => r.code}
      bind:selected
      listTitle="Codes"
      listHint="Most severe first"
      detailLabel="Code details"
    >
      {#snippet row(r: DtcRow)}
        <Icon name={GROUP_ICON[r.group]} size={22} class="rv-row-lead rv-t-{GROUP_TONE[r.group]}" />
        <span class="rv-row-main">
          <span class="rv-row-title">
            <span class="rv-row-code">{r.code}</span>
            <span class="rv-row-state rv-t-{GROUP_TONE[r.group]}">{r.groupLabel}</span>
          </span>
          <span class="rv-row-sub">{r.description || "No description in the DTC file"}</span>
        </span>
        <span class="rv-row-trail">{r.priority}</span>
      {/snippet}

      {#snippet detail(r: DtcRow, full: boolean)}
        {#if full}<h2>{r.code}</h2>{/if}
        <div class="rv-chips">
          <span class="rv-chip {GROUP_CHIP[r.group]}">
            <Icon name={GROUP_ICON[r.group]} size={14} />
            {r.groupLabel}
          </span>
          <span class="rv-chip c-plain">{r.priority}</span>
        </div>
        {#if r.description}
          <p class="rv-lede">{r.description}</p>
        {:else}
          <p class="rv-lede rv-muted">No description in the DTC file.</p>
        {/if}
        <dl class="rv-facts">
          <dt>Reported by</dt>
          <dd>Mode {r.mode}, {r.groupLabel.toLowerCase()}</dd>
        </dl>
        <div class="rv-acts">
          <a class="rv-btn" href={lookupUrl(r.code)} target="_blank" rel="noreferrer">Look up {r.code}</a>
        </div>
      {/snippet}
    </MasterDetail>
  {/if}
</section>

<style>
  .sr-only {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip-path: inset(50%);
    white-space: nowrap;
  }
</style>
