<script lang="ts">
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import { BITRATES, GROUP, ROW, bitrateLabel, buildCanPatch, anyShown, rowShown, type RowMeta, type SettingsDoc } from "$lib/settingsModel";
  import ApplyBar from "./ApplyBar.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q, doc }: { q: string; doc: SettingsDoc } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.vehicle, row);
  const bitrateOptions = BITRATES.map((bps) => ({ value: bps, label: bitrateLabel(bps) }));

  // Queues are text in the draft so a half-typed number can be shown as an error, not clamped.
  let edits = $state<{ bitrate?: number; rx?: string; tx?: string }>({});
  const draft = $derived({
    bitrate: edits.bitrate ?? doc.can.bitrate,
    rx: edits.rx ?? String(doc.can.rx_queue_size),
    tx: edits.tx ?? String(doc.can.tx_queue_depth),
  });
  const built = $derived(buildCanPatch(doc.can, draft));
  const count = $derived(Object.keys(built.patch).length);
  const blocked = $derived(Object.keys(built.errors).length > 0);

  async function apply() {
    if (blocked || count === 0) return;
    if (await settingsStore.save("can", built.patch)) edits = {};
  }
</script>

{#if anyShown(q, GROUP.vehicle, [ROW.bitrate, ROW.rxQueue, ROW.txQueue])}
<div class="st-sec">
  <h2>Bus</h2>
  <div class="st-list">
    {#if show(ROW.bitrate)}
      <SettingRow title={ROW.bitrate.title} hint="Must match the car. 500 kbps is typical for OBD-II." error={built.errors.bitrate}>
        <SelectMenu options={bitrateOptions} value={draft.bitrate} ariaLabel={ROW.bitrate.title} onChange={(v) => (edits.bitrate = v)} />
      </SettingRow>
    {/if}
    {#if show(ROW.rxQueue)}
      <SettingRow title={ROW.rxQueue.title} hint="Frames buffered between the bus and the poller. 1 to 256." error={built.errors.rx_queue_size}>
        <input
          class="st-in num mono"
          type="text"
          inputmode="numeric"
          aria-label={ROW.rxQueue.title}
          value={draft.rx}
          aria-invalid={built.errors.rx_queue_size ? "true" : undefined}
          oninput={(e) => (edits.rx = e.currentTarget.value)}
        />
      </SettingRow>
    {/if}
    {#if show(ROW.txQueue)}
      <SettingRow title={ROW.txQueue.title} hint="Frames waiting to be sent. 1 to 64." error={built.errors.tx_queue_depth}>
        <input
          class="st-in num mono"
          type="text"
          inputmode="numeric"
          aria-label={ROW.txQueue.title}
          value={draft.tx}
          aria-invalid={built.errors.tx_queue_depth ? "true" : undefined}
          oninput={(e) => (edits.tx = e.currentTarget.value)}
        />
      </SettingRow>
    {/if}
  </div>
</div>
{/if}

<ApplyBar {count} busy={settingsStore.saving} {blocked} onRevert={() => (edits = {})} onApply={apply} />
