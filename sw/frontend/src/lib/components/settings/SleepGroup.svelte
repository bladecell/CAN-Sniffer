<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import Dialog from "$lib/components/Dialog.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { parseJsonResponse } from "$lib/api";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import RangeSlider from "$lib/components/RangeSlider.svelte";
  import { GROUP, ROW, VOLT_MAX, VOLT_MIN, anyShown, buildSleepPatch, parseVolts, rowShown, sleepDraft, type RowMeta, type SettingsDoc } from "$lib/settingsModel";
  import ApplyBar from "./ApplyBar.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q, doc }: { q: string; doc: SettingsDoc } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.sleep, row);

  let edits = $state<{ enabled?: boolean; awake?: string; low?: string }>({});
  const power = $derived(doc.system.power);
  const draft = $derived(power ? { ...sleepDraft(power), ...edits } : null);
  const built = $derived(power && draft ? buildSleepPatch(power, draft) : { patch: {}, errors: {} });
  const count = $derived(Object.keys(built.patch).length);
  const blocked = $derived(Object.keys(built.errors).length > 0);

  /** "13.2" or "13.2 V" to volts, or null. */
  const parseVoltsText = (t: string) => {
    const mv = parseVolts(t.trim().replace(/\s*v$/i, ""));
    return mv === null ? null : mv / 1000;
  };
  // Saved values are always in range, so the fallback only guards a hand-edited draft.
  const volts = (text: string) => (parseVolts(text) ?? VOLT_MIN * 1000) / 1000;

  let confirmSleep = $state(false);
  let sleeping = $state(false);
  async function sleepNow() {
    confirmSleep = false;
    try {
      await parseJsonResponse<unknown>(await firmwareApi.sleep());
      sleeping = true;
      alertStore.add("Going to sleep. The page reconnects after the device wakes and the CAN bus is live.", "success");
    } catch (e) {
      alertStore.add(`Sleep failed: ${e instanceof Error ? e.message : String(e)}`, "error");
    }
  }

  async function apply() {
    if (blocked || count === 0) return;
    if (await settingsStore.save("system", built.patch, false)) edits = {};
  }
</script>

{#if anyShown(q, GROUP.sleep, [ROW.sleepAuto, ROW.stayAwakeV, ROW.lowBatteryV, ROW.sleepNow])}
<div class="st-sec">
  <h2>Battery and idle</h2>
  <div class="st-list">
    {#if draft && show(ROW.sleepAuto)}
      <SettingRow inline title={ROW.sleepAuto.title} hint="Sleeps after 60 s without a CAN bus. Off keeps the device awake at any battery voltage and can drain the car battery. Applies at once.">
        <Switch aria-label={ROW.sleepAuto.title} checked={draft.enabled} onchange={(v) => (edits.enabled = v)} />
      </SettingRow>
    {/if}
    {#if draft && (show(ROW.lowBatteryV) || show(ROW.stayAwakeV))}
      <SettingRow title="Battery thresholds" hint="Above the right thumb the engine is charging, so the device stays awake. Below the left thumb the device sleeps 60 s per cycle instead of 10 s, to save the battery. 9.0 to 16.0 V." error={built.errors.stay_awake_voltage_mv ?? built.errors.low_battery_voltage_mv}>
        <span class="sr" id="sl-lo">{ROW.lowBatteryV.title}</span>
        <span class="sr" id="sl-aw">{ROW.stayAwakeV.title}</span>
        <RangeSlider
          labelIds={["sl-lo", "sl-aw"]}
          values={[volts(draft.low), volts(draft.awake)]}
          min={VOLT_MIN}
          max={VOLT_MAX}
          step={0.1}
          gap={0.7}
          fit
          parse={parseVoltsText}
          page={1}
          format={(v) => `${v.toFixed(1)} V`}
          ticks={[11.8, 14.2]}
          onChange={([l, h]) => {
            edits.low = l.toFixed(1);
            edits.awake = h.toFixed(1);
          }}
        />
      </SettingRow>
    {/if}
    {#if show(ROW.sleepNow)}
      <SettingRow inline title={ROW.sleepNow.title} hint="One sleep cycle, not power off. Wi-Fi drops; the device wakes every 10 s (60 s on a low battery) and comes back only when the CAN bus is live." boxed>
        <button type="button" class="btn" onclick={() => (confirmSleep = true)} disabled={sleeping}>
          <Icon name="zzz" size={16} />Sleep now
        </button>
      </SettingRow>
    {/if}
  </div>
</div>
{/if}

<Dialog
  open={confirmSleep}
  title="Put the device to sleep?"
  message="Wi-Fi and this page disconnect. Logging and polling stop until the device wakes with a live CAN bus."
  confirmLabel="Sleep"
  onConfirm={sleepNow}
  onCancel={() => (confirmSleep = false)}
/>

<ApplyBar {count} busy={settingsStore.saving} {blocked} onRevert={() => (edits = {})} onApply={apply} />
