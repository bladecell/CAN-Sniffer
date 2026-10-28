<script lang="ts">
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  // The SD file list needs the firmware file API, which is not wired yet. Every action is disabled until it lands.
  const cardPresent = $derived(vehicleStore.sdInfo?.is_mounted === true);

  $effect(() => {
    void vehicleStore.requestSDInfo();
  });
</script>

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={SIDEBAR_HINTS.sd} />
    <h2>SD card</h2>
    <p class="sb-counts">{cardPresent ? "Card present" : "No SD card detected"}</p>
    <div class="sb-selrow"><span>No file selected</span></div>
  {/snippet}

  <div class="sb-list">
    <div class="sb-empty"><b>No .json files</b></div>
  </div>

  {#snippet footer()}
    <button type="button" class="tel-btn pri" aria-disabled="true">Open file</button>
    <button type="button" class="tel-btn" aria-disabled="true">Save selected PIDs</button>
    <button type="button" class="tel-btn" aria-disabled="true">New file</button>
    <button type="button" class="tel-btn danger" aria-disabled="true">Delete file</button>
  {/snippet}
</SidebarPanel>

<style>
  .sb-list {
    display: grid;
    margin: 0;
    padding: 0;
  }
  .tel-btn.danger {
    color: var(--ctl-crit);
  }
  .tel-btn[aria-disabled='true'] {
    opacity: 0.55;
    cursor: not-allowed;
  }
</style>
