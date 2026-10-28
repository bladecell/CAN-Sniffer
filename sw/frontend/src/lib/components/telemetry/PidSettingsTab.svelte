<script lang="ts">
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";
</script>

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={SIDEBAR_HINTS.settings} />
    <h2>Settings</h2>
    <p class="sb-counts">Where the SD tab looks</p>
  {/snippet}

<div class="set">
  <section class="fld">
    <label for="sd-folder">SD card folder</label>
    <div class="inrow">
      <input id="sd-folder" class="ctl mono" value="" placeholder="/sdcard/pids" disabled aria-describedby="sd-help" />
      <button type="button" class="btn" aria-disabled="true">Choose</button>
    </div>
    <p class="note" id="sd-help">The SD tab lists the .json files in this folder. Kept in this browser, not on the device. Not saved yet.</p>
  </section>

  <section class="fld">
    <h3>Stored on the device</h3>
    <output class="ro">{vehicleStore.pidDefinitions.length} PIDs</output>
    <p class="note">The device keeps its PIDs in its own memory and loads them at start. It does not need the SD card.</p>
  </section>
</div>
</SidebarPanel>

<style>
  .set {
    display: grid;
    gap: 22px;
    padding: 8px 2px 12px;
  }
  .fld {
    display: grid;
    gap: 6px;
  }
  .fld h3,
  .fld > label {
    margin: 0;
    font-size: 0.95rem;
    font-weight: 800;
  }
  .inrow {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 8px;
  }
  .ctl {
    height: var(--ctl-h);
    min-width: 0;
    width: 100%;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    padding: 0 14px;
    font: inherit;
  }
  .ctl.mono {
    font-family: var(--mono);
    font-size: 0.875rem;
  }
  .ctl:disabled {
    opacity: 0.55;
  }
  .btn {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    min-height: var(--ctl-h);
    padding: 0 16px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font-weight: 700;
    opacity: 0.55;
    cursor: not-allowed;
  }
  .ro {
    display: block;
    min-height: var(--ctl-h);
    padding: 12px 14px;
    border-radius: var(--r-ctl);
    background: var(--row);
    font-weight: 700;
    font-variant-numeric: tabular-nums;
  }
  .note {
    margin: 0;
    font-size: 0.82rem;
    font-weight: 600;
    line-height: 1.4;
    color: var(--muted);
  }
</style>
