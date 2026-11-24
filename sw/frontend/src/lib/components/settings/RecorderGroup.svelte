<script lang="ts">
  import { onMount } from "svelte";
  import Switch from "$lib/components/Switch.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { recordingStore } from "$lib/recordingStore.svelte";
  import { GROUP, ROW, rowShown, type RowMeta } from "$lib/settingsModel";
  import type { RecorderTrigger } from "$lib/types";
  import SegControl from "./SegControl.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.recorder, row);
  const canConnected = $derived(connectionStore.wsCanStatus?.state === "CAN Connected");
  const rec = $derived(recordingStore.status);
  const TRIGGERS: readonly { value: RecorderTrigger; label: string }[] = [
    { value: "manual", label: "Manual" },
    { value: "auto", label: "Automatic" },
  ];

  onMount(() => void recordingStore.refreshIfStale());
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

{#if show(ROW.recTrigger) || show(ROW.recContinuous)}
  <div class="st-sec">
    <h2>Trips</h2>
    <div class="st-list">
      {#if show(ROW.recTrigger)}
        <SettingRow
          title={ROW.recTrigger.title}
          hint="Automatic starts a trip when the engine runs or the car moves, and stops it 60 s after both stop. Manual trips are never stopped automatically."
          stack
          disabled={!rec}
        >
          <SegControl
            label={ROW.recTrigger.title}
            options={TRIGGERS}
            value={rec?.trigger ?? "manual"}
            onChange={(v) => void recordingStore.saveSettings({ trigger: v })}
          />
        </SettingRow>
      {/if}
      {#if show(ROW.recContinuous)}
        <SettingRow
          title={ROW.recContinuous.title}
          hint="Keep polling while the bus is connected, so Automatic sees RPM and speed. Costs battery while parked."
          inline
          disabled={!rec}
        >
          <Switch
            checked={rec?.continuous_polling ?? false}
            disabled={!rec}
            aria-label={ROW.recContinuous.title}
            onchange={(on: boolean) => void recordingStore.saveSettings({ continuous_polling: on })}
          />
        </SettingRow>
      {/if}
    </div>
  </div>
{/if}
