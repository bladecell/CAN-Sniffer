<script lang="ts">
  import { onMount } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import Dialog from "$lib/components/Dialog.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { parseJsonResponse } from "$lib/api";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { GROUP, ROW, formatUptime, anyShown, rowShown, type RowMeta } from "$lib/settingsModel";
  import SettingRow from "./SettingRow.svelte";

  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.device, row);
  const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));
  const KB = 1024;

  // GET /system is open (no login), so the figures load without a session.
  onMount(() => void vehicleStore.requestSystem());
  const sys = $derived(vehicleStore.system);
  const loaded = $derived(sys.api_version > 0);

  let confirmRestart = $state(false);
  let restarting = $state(false);
  async function restart() {
    confirmRestart = false;
    try {
      await parseJsonResponse<unknown>(await firmwareApi.reboot());
      restarting = true;
      alertStore.add("Restarting. Settings are kept.", "success");
    } catch (e) {
      alertStore.add(`Restart failed: ${errorText(e)}`, "error");
    }
  }
</script>

{#if anyShown(q, GROUP.device, [ROW.firmware])}
<div class="st-sec">
  <h2>Firmware</h2>
  <div class="st-list">
    {#if show(ROW.firmware)}
      <SettingRow title={ROW.firmware.title} hint={loaded ? `API version ${sys.api_version}` : "Reading the device"} inline>
        <span class="st-val">{loaded && sys.app_version ? sys.app_version : "Unknown"}</span>
      </SettingRow>
    {/if}
  </div>
</div>
{/if}

{#if anyShown(q, GROUP.device, [ROW.uptime, ROW.lastReset, ROW.freeMemory, ROW.restart])}
<div class="st-sec">
  <h2>System</h2>
  <div class="st-list">
    {#if show(ROW.uptime)}
      <SettingRow title={ROW.uptime.title} inline><span class="st-val">{loaded ? formatUptime(sys.uptime_s) : "-"}</span></SettingRow>
    {/if}
    {#if show(ROW.lastReset)}
      <SettingRow title={ROW.lastReset.title} inline><span class="st-val">{loaded && sys.restart_reason ? sys.restart_reason : "-"}</span></SettingRow>
    {/if}
    {#if show(ROW.freeMemory)}
      <SettingRow title={ROW.freeMemory.title} hint="All heaps, PSRAM included." inline>
        <span class="st-val">{loaded ? `${Math.round(sys.heap_free / KB)} KB` : "-"}</span>
      </SettingRow>
    {/if}
    {#if show(ROW.restart)}
      <SettingRow title={ROW.restart.title} hint="Keeps all settings. The page reconnects when the device is back.">
        <button type="button" class="st-btn" onclick={() => (confirmRestart = true)} disabled={restarting}>
          <Icon name="reload" size={16} />Restart device
        </button>
      </SettingRow>
    {/if}
  </div>
</div>
{/if}

<Dialog
  open={confirmRestart}
  title="Restart the device?"
  message="The device restarts within a second. Logging and polling stop until it is back."
  confirmLabel="Restart"
  onConfirm={restart}
  onCancel={() => (confirmRestart = false)}
/>
