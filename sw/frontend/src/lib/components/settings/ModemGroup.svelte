<script lang="ts">
  import { GROUP, ROW, anyShown, rowShown, type RowMeta } from "$lib/settingsModel";
  import SettingRow from "./SettingRow.svelte";

  // Placeholder for the HW v2 LTE modem: no API yet, so every row is disabled and tagged.
  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.modem, row);
</script>

{#if anyShown(q, GROUP.modem, [ROW.modem, ROW.apn, ROW.mqtt, ROW.gnss])}
<div class="st-sec">
  <h2>LTE modem</h2>
  <div class="st-list">
    {#if show(ROW.modem)}
      <SettingRow title={ROW.modem.title} hint="Needs the HW v2 board. These settings appear when a modem is detected." inline>
        <span class="st-val">Not fitted</span>
      </SettingRow>
    {/if}
    {#if show(ROW.apn)}
      <SettingRow title={ROW.apn.title} tag="HW v2" disabled>
        <input class="st-in" type="text" placeholder="Not available" disabled aria-label={ROW.apn.title} />
      </SettingRow>
    {/if}
    {#if show(ROW.mqtt)}
      <SettingRow title={ROW.mqtt.title} tag="HW v2" disabled>
        <input class="st-in mono" type="text" placeholder="Not available" disabled aria-label={ROW.mqtt.title} />
      </SettingRow>
    {/if}
    {#if show(ROW.gnss)}
      <SettingRow title={ROW.gnss.title} tag="HW v2" disabled inline />
    {/if}
  </div>
</div>
{/if}
