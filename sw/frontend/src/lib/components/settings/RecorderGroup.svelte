<script lang="ts">
  import Switch from "$lib/components/Switch.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { GROUP, ROW, rowShown, type RowMeta } from "$lib/settingsModel";
  import SettingRow from "./SettingRow.svelte";

  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.recorder, row);
  const canConnected = $derived(connectionStore.wsCanStatus?.state === "CAN Connected");
</script>

<div class="st-sec">
  <h2>Polling</h2>
  <div class="st-list">
    {#if show(ROW.autoPoll)}
      <SettingRow
        title={ROW.autoPoll.title}
        hint="Keep polling the car and resume after ignition off/on. Recording starts polling on its own."
        inline
        disabled={!canConnected}
      >
        <Switch
          checked={vehicleStore.obd2Status?.auto_poll ?? false}
          disabled={!canConnected}
          aria-label={ROW.autoPoll.title}
          onchange={(on: boolean) => vehicleStore.setAutoPoll(on)}
        />
      </SettingRow>
    {/if}
  </div>
</div>
