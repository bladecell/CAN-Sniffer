<script lang="ts">
  import { onMount } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { recordingStore } from "$lib/recordingStore.svelte";
  import { tripViewer } from "$lib/tripViewer.svelte";
  import { formatBytes, replayRows } from "$lib/recorderView";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  interface Props {
    phone?: boolean;
    onPick: (name: string) => void;
  }

  let { onPick, phone = false }: Props = $props();

  const rec = $derived(recordingStore.status);
  const rows = $derived(replayRows(recordingStore.trips, rec ? { state: rec.state, trip: rec.trip, fileBytes: rec.file_bytes } : null));
  const open = $derived(tripViewer.trip?.name ?? null);
  const counts = $derived(
    recordingStore.tripsError === "no-sd"
      ? "No SD card"
      : rows.length === 0
        ? "No recordings"
        : `${rows.length} ${rows.length === 1 ? "recording" : "recordings"}${recordingStore.tripsTruncated ? ", newest 100" : ""}`,
  );

  onMount(() => {
    void recordingStore.refreshIfStale();
    void recordingStore.loadTrips();
    return recordingStore.watchTrips();
  });
</script>

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={SIDEBAR_HINTS.replays} />
    <h2>Replays</h2>
    <p class="sb-counts num">{counts}</p>
  {/snippet}

  {#if recordingStore.tripsError === "no-sd"}
    <div class="sb-empty" role="alert"><b>SD card not available</b>Trips are saved on the SD card. Insert one, or wait if it is busy, then open this tab again.</div>
  {:else if recordingStore.tripsError === "failed" && rows.length === 0}
    <div class="sb-empty" role="alert">
      <b>The trip list failed to load</b>
      <button type="button" class="btn" onclick={() => void recordingStore.loadTrips()}>Try again</button>
    </div>
  {:else if rows.length === 0 && recordingStore.tripsLoading}
    <div class="sb-empty" role="status"><b>Loading trips</b></div>
  {:else if rows.length === 0}
    <div class="sb-empty"><b>No recordings yet</b>Record a trip on the Recorder page. It shows up here once it is stopped.</div>
  {:else}
    <ul class="rp" aria-label="Recorded trips">
      {#each rows as t (t.key)}
        <li>
          <button type="button" class="rp-row" class:phone aria-pressed={open === t.name} disabled={tripViewer.loading} onclick={() => onPick(t.name)}>
            <Icon name={t.sidecar ? "file-doc" : "alert-circle"} size={22} />
            <span class="rp-nm">
              <b class="mono">{t.name}</b>
              <span>{t.sidecar ? "Summary saved" : "No summary file"}</span>
            </span>
            <span class="rp-sz num">{formatBytes(t.size)}</span>
          </button>
        </li>
      {/each}
    </ul>
  {/if}
</SidebarPanel>

<style>
  .rp {
    display: grid;
    gap: 8px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .rp li {
    margin: 0;
    padding: 0;
  }
  .rp-row {
    all: unset;
    box-sizing: border-box;
    position: relative;
    display: grid;
    grid-template-columns: auto minmax(0, 1fr) auto;
    align-items: center;
    gap: 12px;
    width: 100%;
    min-height: 64px;
    padding: 8px 14px;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-sub);
    cursor: pointer;
  }
  .rp-row[aria-pressed='true'] {
    color: var(--ctl-sel);
    font-weight: 800;
  }
  .rp-row[aria-pressed='true']::after {
    content: '';
    position: absolute;
    left: 14px;
    right: 14px;
    bottom: 4px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .rp-row:disabled {
    opacity: 0.6;
    cursor: progress;
  }
  .rp-row:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
  }
  @media (hover: hover) {
    .rp-row:hover:not(:disabled) {
      background: var(--hov);
    }
  }
  .rp-nm {
    display: grid;
    min-width: 0;
  }
  .rp-nm b {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .rp-nm span {
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 600;
  }
  .rp-sz {
    color: var(--muted);
    font-weight: 700;
    white-space: nowrap;
  }
</style>
