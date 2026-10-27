<script lang="ts">
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { pendingKind, type PendingKind } from "$lib/telemetryView";
  import Icon from "$lib/Icon.svelte";
  import Dialog from "$lib/components/Dialog.svelte";

  interface Props {
    onImport: () => void;
    onEdit: (index: number) => void;
  }

  let { onImport, onEdit }: Props = $props();

  const pidColor = (row: PidRow) => safeHexColor("#" + Number(row.def.color).toString(16).padStart(6, "0"));

  const KIND_LABEL: Record<PendingKind, string> = { added: "Added", modified: "Modified", removed: "Removed" };
  const KIND_CLASS: Record<PendingKind, string> = { added: "k-ok", modified: "k-warn", removed: "k-crit" };
  const KIND_ORDER: PendingKind[] = ["added", "modified", "removed"];

  function kindOf(row: PidRow): PendingKind {
    return pendingKind(row.loaded, vehicleStore.pidDefinitions.some((d) => Number(d.pid) === parseInt(row.pid, 16)));
  }

  const changes = $derived(
    telemetryStore.local_piddef
      .filter((r) => r.pendingChanges === "Yes")
      .map((row) => ({ row, index: telemetryStore.local_piddef.indexOf(row), kind: kindOf(row) }))
      .sort((a, b) => KIND_ORDER.indexOf(a.kind) - KIND_ORDER.indexOf(b.kind)),
  );

  let selectedRows = $derived(telemetryStore.local_piddef.filter((r) => r.selected));
  let selectedCount = $derived(selectedRows.length);
  let selectedPendingCount = $derived(selectedRows.filter((r) => r.pendingChanges === "Yes").length);
  let pendingCount = $derived(changes.length);

  // Ticked PIDs are the scope of Update and Remove. With none ticked, Update covers every pending change.
  let updateText = $derived(selectedCount > 0 ? `Update ${selectedCount} selected` : `Update ${pendingCount} pending`);
  let updateDisabled = $derived(selectedCount > 0 ? selectedPendingCount !== selectedCount : pendingCount === 0);

  let trayOpen = $state(false);
  let confirmOpen = $state(false);
  let barEl = $state<HTMLDivElement>();
  let trayBtn = $state<HTMLButtonElement>();

  // The tray is a floating layer over the list: Esc and a press outside it close it.
  $effect(() => {
    if (!trayOpen) return;
    const onPointer = (e: PointerEvent) => {
      if (!barEl?.contains(e.target as Node)) trayOpen = false;
    };
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== "Escape") return;
      trayOpen = false;
      trayBtn?.focus();
    };
    document.addEventListener("pointerdown", onPointer);
    document.addEventListener("keydown", onKey);
    return () => {
      document.removeEventListener("pointerdown", onPointer);
      document.removeEventListener("keydown", onKey);
    };
  });

  async function update() {
    if (updateDisabled) return;
    await vehicleStore.updatePids(telemetryStore.local_piddef);
  }

  function exportDefinitions() {
    const blob = new Blob([JSON.stringify(vehicleStore.pidDefinitions, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "can_sniffer_pid_definitions.json";
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  async function confirmRemove() {
    const removed = new Set(selectedRows.map((r) => parseInt(r.pid, 16)));
    const desired = telemetryStore.local_piddef.filter((row) => row.loaded && !removed.has(parseInt(row.pid, 16)));
    confirmOpen = false;
    await vehicleStore.replacePids(desired);
  }

  function requestRemove() {
    if (selectedCount === 0) return;
    confirmOpen = true;
  }
</script>

<div class="ch">
  <header class="ch-head">
    <h2>Changes</h2>
    <p class="num">{pendingCount === 0 ? "Matches the device" : `${pendingCount} not on the device`}</p>
  </header>

  <div class="ch-bar" bind:this={barEl}>
  <div class="ch-actions">
    <button type="button" class="tel-btn pri" aria-disabled={updateDisabled} onclick={update}>
      <Icon name="reload" size={16} />{updateText}
    </button>
    <button
      type="button"
      class="tel-btn"
      onclick={() => vehicleStore.savePids(telemetryStore.local_piddef)}
      title="Save the PID map to the SD card"
    >
      <Icon name="save" size={16} />Save to SD card
    </button>
    <button type="button" class="tel-btn danger" aria-disabled={selectedCount === 0} onclick={requestRemove}>
      <Icon name="trash" size={16} />Remove{selectedCount > 0 ? ` ${selectedCount}` : ""}
    </button>
    <button
      bind:this={trayBtn}
      type="button"
      class="tel-btn"
      aria-expanded={trayOpen}
      aria-controls="ch-tray"
      onclick={() => (trayOpen = !trayOpen)}
    >
      <Icon name="import" size={16} />Import / Export
    </button>
  </div>

  {#if trayOpen}
    <div class="tray tel-float" id="ch-tray" role="group" aria-label="Import and export PIDs">
      <button type="button" class="tcard" onclick={onImport}>
        <Icon name="import" size={18} />
        <b>Import PIDs</b>
        <small>Add PIDs from a JSON file</small>
      </button>
      <button type="button" class="tcard" onclick={exportDefinitions}>
        <Icon name="export" size={18} />
        <b>Export PIDs</b>
        <small>Download the current set as JSON</small>
      </button>
      <p class="thint">Changes stay on this page until you update the device.</p>
    </div>
  {/if}
  </div>

  <ul class="ch-list">
    {#each changes as change (change.row)}
      <li class="crow">
        <button type="button" class="cmain" onclick={() => onEdit(change.index)} aria-label="Edit {change.row.name}, {KIND_LABEL[change.kind]}">
          <span class="nm"><i style="--sw: {pidColor(change.row)}"></i><b>{change.row.name}</b></span>
          <span class="sub num">{change.row.pid}</span>
        </button>
        <span class="tel-chip {KIND_CLASS[change.kind]}">{KIND_LABEL[change.kind]}</span>
      </li>
    {:else}
      <li class="empty">No changes. The device matches these definitions.</li>
    {/each}
  </ul>
</div>

<Dialog
  open={confirmOpen}
  title="Remove {selectedCount} PID{selectedCount === 1 ? '' : 's'}?"
  message="Removes the definitions from the device. The SD card file changes when you press Save to SD card."
  confirmLabel="Remove {selectedCount}"
  danger
  onConfirm={confirmRemove}
  onCancel={() => (confirmOpen = false)}
/>

<style>
  .ch {
    display: flex;
    flex-direction: column;
    gap: 10px;
    min-height: 0;
    flex: 1 1 auto;
  }
  .ch-head {
    display: flex;
    flex-wrap: wrap;
    align-items: baseline;
    justify-content: space-between;
    gap: 4px 12px;
    padding: 6px 8px 0 10px;
  }
  .ch-head h2 {
    font-size: 1.15rem;
    margin: 0;
  }
  .ch-head p {
    margin: 0;
    font-size: 0.85rem;
    color: var(--muted);
  }
  .ch-actions {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 6px;
    padding: 0 6px;
  }
  .ch-actions .tel-btn {
    width: 100%;
    padding: 0 8px;
    font-size: 0.88rem;
  }
  .ch-actions .pri {
    grid-column: 1 / -1;
  }
  .ch-bar {
    position: relative;
    display: flex;
    flex-direction: column;
    flex: none;
  }
  .tray {
    position: absolute;
    top: calc(100% + 6px);
    left: 6px;
    right: 6px;
    z-index: 10;
    display: grid;
    gap: 8px;
    padding: 10px;
  }
  .tcard {
    all: unset;
    box-sizing: border-box;
    display: grid;
    gap: 2px;
    justify-items: start;
    padding: 10px 12px;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    cursor: pointer;
  }
  .tcard:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .tcard small {
    color: var(--ctl-sub);
  }
  .thint {
    margin: 0;
    font-size: 0.8rem;
    color: var(--muted);
  }
  .ch-list {
    display: flex;
    flex-direction: column;
    gap: 6px;
    min-height: 120px;
    flex: 1 1 auto;
    overflow: auto;
    margin: 0;
    padding: 2px;
    list-style: none;
  }
  .crow {
    display: flex;
    align-items: center;
    gap: 8px;
    min-height: 56px;
    padding-left: 4px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  .cmain {
    all: unset;
    box-sizing: border-box;
    flex: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
    gap: 2px;
    padding: 6px 4px;
    cursor: pointer;
  }
  .cmain:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
    border-radius: var(--r-ctl);
  }
  .nm {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .nm b {
    font-weight: 800;
    overflow-wrap: anywhere;
  }
  .nm i {
    display: inline-block;
    width: 14px;
    height: 4px;
    border-radius: 2px;
    background: var(--sw);
    flex: none;
  }
  .sub {
    color: var(--muted);
    font-size: 0.8rem;
  }
  .crow .tel-chip {
    margin-right: 8px;
  }
  .empty {
    padding: 12px;
    color: var(--muted);
  }
</style>
