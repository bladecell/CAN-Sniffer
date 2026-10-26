<script lang="ts">
  import { onMount } from "svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { recordingStore } from "$lib/recordingStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { formatBytes, sidecarName, tripRows, type TripRow } from "$lib/recorderView";
  import Icon from "$lib/Icon.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import Dialog from "$lib/components/Dialog.svelte";
  import ViewHeader from "$lib/components/listDetail/ViewHeader.svelte";
  import StatusLine from "$lib/components/listDetail/StatusLine.svelte";
  import StateNote from "$lib/components/listDetail/StateNote.svelte";
  import MasterDetail from "$lib/components/listDetail/MasterDetail.svelte";

  const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

  const rec = $derived(recordingStore.status);
  const recState = $derived(rec?.state ?? "idle");
  const recording = $derived(recState === "recording" || recState === "stopping");
  const trips = $derived(tripRows(recordingStore.trips, rec ? { state: recState, trip: rec.trip, fileBytes: rec.file_bytes } : null));
  const canConnected = $derived(connectionStore.wsCanStatus?.state === "CAN Connected");
  const sdMissing = $derived(recordingStore.tripsError === "no-sd");

  let selected = $state<string | null>(null);
  let pendingDelete = $state<TripRow | null>(null);

  const line = $derived.by((): { tone: "crit" | "warn" | "muted"; icon?: string; text: string; rest: string } => {
    if (!rec) return { tone: "muted", text: "Checking recorder…", rest: "" };
    switch (recState) {
      case "recording":
        return {
          tone: "crit",
          icon: "circle-solid",
          text: "Recording",
          rest: `${rec.rows.toLocaleString("en")} rows, ${rec.dropped.toLocaleString("en")} dropped`,
        };
      case "stopping":
        return { tone: "muted", text: "Stopping…", rest: "Writing the last rows" };
      case "error":
        return { tone: "warn", icon: "alert-circle", text: "Recorder error", rest: rec.last_error ?? "" };
      default:
        return {
          tone: "muted",
          icon: "circle",
          text: "Not recording",
          rest: rec.trip > 0 ? `Last trip ${rec.trip}` : "No trips recorded yet",
        };
    }
  });

  const primaryDisabled = $derived(recording ? recordingStore.busy || recState === "stopping" : recordingStore.busy || recordingStore.coolingDown);
  const primaryLabel = $derived(
    recording ? "Stop recording" : recordingStore.coolingDown ? `Start in ${recordingStore.cooldownLeft} s` : "Start recording",
  );

  function togglePrimary() {
    if (primaryDisabled) return;
    if (recording) void recordingStore.stop();
    else void recordingStore.start();
  }

  function retry() {
    void recordingStore.refresh();
    void recordingStore.loadTrips();
  }

  async function download(t: TripRow) {
    try {
      await recordingStore.download(t.name);
    } catch (e) {
      alertStore.add(`Failed to download ${t.name}: ${errorText(e)}`, "error");
    }
  }

  async function confirmDelete() {
    const t = pendingDelete;
    if (!t) return;
    pendingDelete = null;
    try {
      await recordingStore.deleteTrip(t.name, t.sidecar);
      alertStore.add(`Deleted ${t.name}`, "success");
    } catch (e) {
      alertStore.add(`Failed to delete ${t.name}: ${errorText(e)}`, "error");
    }
  }

  onMount(() => {
    void recordingStore.refresh();
    void recordingStore.loadTrips();
  });

  // Countdown ticks only while a start is refused.
  $effect(() => {
    if (!recordingStore.coolingDown) return;
    const id = setInterval(() => (recordingStore.clock = Date.now()), 250);
    return () => clearInterval(id);
  });

  // The WebSocket frame has no file size, so re-read GET /recorder while a trip is being written.
  $effect(() => {
    if (!recording) return;
    const id = setInterval(() => void recordingStore.refresh(), 5_000);
    return () => clearInterval(id);
  });
</script>

<section class="rv-wrap">
  <ViewHeader title="Recorder">
    {#snippet status()}
      <StatusLine tone={line.tone} icon={line.icon} text={line.text} rest={line.rest} />
    {/snippet}

    {#snippet meta()}
      <div class="rv-meta">
        <Switch
          label="Auto poll"
          subLabel="Keep polling the car and resume after ignition off/on"
          checked={vehicleStore.obd2Status?.auto_poll}
          onchange={(on: boolean) => vehicleStore.setAutoPoll(on)}
          disabled={!canConnected}
        />
      </div>
    {/snippet}

    {#snippet actions()}
      <button type="button" class="rv-btn pri" aria-disabled={primaryDisabled} onclick={togglePrimary}>
        <Icon name={recording ? "square" : "circle-solid"} size={16} />
        {primaryLabel}
      </button>
    {/snippet}
  </ViewHeader>

  {#if sdMissing}
    <StateNote title="SD card not available" icon="alert-circle" tone="warn" role="alert">
      <p class="rv-muted">Trips are saved to the SD card, so recording is off and the trip list can't be read. Insert a card, or wait if it is busy, then check again.</p>
      <div class="rv-acts">
        <button type="button" class="rv-btn pri" onclick={retry}>Check again</button>
      </div>
    </StateNote>
  {:else if recordingStore.tripsError === "failed" && trips.length === 0}
    <StateNote title="Trip list failed to load" icon="alert-circle" tone="warn" role="alert">
      <div class="rv-acts">
        <button type="button" class="rv-btn pri" onclick={retry}>Try again</button>
      </div>
    </StateNote>
  {:else if trips.length === 0 && recordingStore.tripsLoading}
    <StateNote title="Loading trips" spinning role="status" />
  {:else if trips.length === 0}
    <StateNote title="No trips yet" icon="file-doc" role="status">
      <p class="rv-muted">Press Start recording. Each recording saves one CSV file to the SD card, and it shows up here when you stop.</p>
    </StateNote>
  {:else}
    <MasterDetail
      items={trips}
      keyOf={(t) => t.key}
      label={(t) => t.name}
      bind:selected
      listTitle="Trips"
      listHint={recordingStore.tripsTruncated ? "Newest 100" : "Newest first"}
      detailLabel="Trip details"
    >
      {#snippet row(t: TripRow)}
        <Icon name={t.live ? "circle-solid" : t.sidecar ? "file-doc" : "alert-circle"} size={22} class="rv-row-lead rv-t-{t.live ? 'crit' : t.sidecar ? 'ok' : 'warn'}" />
        <span class="rv-row-main">
          <span class="rv-row-title">
            <span class="rv-row-code">{t.name}</span>
            <span class="rv-row-state rv-t-{t.live ? 'crit' : t.sidecar ? 'ok' : 'warn'}">
              {t.live ? "Recording" : t.sidecar ? "Saved" : "No summary"}
            </span>
          </span>
          <span class="rv-row-sub">{t.live ? "Writing to the SD card" : t.sidecar ? "Summary file saved" : "Summary file missing"}</span>
        </span>
        <span class="rv-row-trail">{t.live ? "live" : formatBytes(t.size)}</span>
      {/snippet}

      {#snippet detail(t: TripRow, full: boolean)}
        {#if full}<h2>{t.name}</h2>{/if}
        <div class="rv-chips">
          {#if t.live}
            <span class="rv-chip c-crit"><Icon name="circle-solid" size={12} />Recording</span>
          {:else if t.sidecar}
            <span class="rv-chip c-ok"><Icon name="circle-check" size={14} />Summary saved</span>
          {:else}
            <span class="rv-chip c-warn"><Icon name="alert-circle" size={14} />No summary file</span>
          {/if}
        </div>
        <dl class="rv-facts">
          <dt>CSV size</dt>
          <dd class="rv-num">{t.live ? "Writing" : formatBytes(t.size)}</dd>
          <dt>Summary</dt>
          <dd>{t.sidecar ? sidecarName(t.name) : "Missing. Power was cut or the card was removed mid-trip."}</dd>
        </dl>
        {#if t.live}
          <p class="rv-muted">Still writing. Stop the recording to download or delete.</p>
        {:else}
          <div class="rv-acts">
            <button type="button" class="rv-btn" onclick={() => download(t)}>
              <Icon name="save" size={16} />
              Download
            </button>
            <button type="button" class="rv-btn danger" onclick={() => (pendingDelete = t)}>
              <Icon name="trash" size={16} />
              Delete
            </button>
          </div>
        {/if}
      {/snippet}
    </MasterDetail>
  {/if}
</section>

<Dialog
  open={pendingDelete !== null}
  title={pendingDelete ? `Delete ${pendingDelete.name}?` : ""}
  message={pendingDelete?.sidecar ? "Removes the CSV and its summary file from the SD card for good." : "Removes the CSV from the SD card for good."}
  confirmLabel="Delete trip"
  danger
  onConfirm={confirmDelete}
  onCancel={() => (pendingDelete = null)}
/>
