<script lang="ts">
  import SegControl from "./SegControl.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import { BITRATES, GROUP, ROW, bitrateLabel, buildCanPatch, buildHwFilterPatch, hwFilterDraft, hwFilterPreview, anyShown, rowShown, type HwFilterDraft, type RowMeta, type SettingsDoc } from "$lib/settingsModel";
  import ApplyBar from "./ApplyBar.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q, doc }: { q: string; doc: SettingsDoc } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.vehicle, row);
  const ID_LENGTHS = [
    { value: "std", label: "Standard, 11-bit" },
    { value: "ext", label: "Extended, 29-bit" },
  ] as const;
  const bitrateOptions = BITRATES.map((bps) => ({ value: bps, label: bitrateLabel(bps) }));

  // Queues are text in the draft so a half-typed number can be shown as an error, not clamped.
  let edits = $state<{ bitrate?: number; rx?: string; tx?: string }>({});
  let hwEdits = $state<Partial<HwFilterDraft>>({});
  const draft = $derived({
    bitrate: edits.bitrate ?? doc.can.bitrate,
    rx: edits.rx ?? String(doc.can.rx_queue_size),
    tx: edits.tx ?? String(doc.can.tx_queue_depth),
  });
  const hw = $derived(doc.can.hwFilter);
  const hwDraft = $derived<HwFilterDraft | null>(hw ? { ...hwFilterDraft(hw), ...hwEdits } : null);
  const preview = $derived(hwDraft ? hwFilterPreview(hwDraft.id, hwDraft.mask, hwDraft.ext) : null);
  const hwBuilt = $derived(hw && hwDraft ? buildHwFilterPatch(hw, hwDraft) : { patch: {}, errors: {} });
  const canBuilt = $derived(buildCanPatch(doc.can, draft));
  const built = $derived({ patch: { ...canBuilt.patch, ...hwBuilt.patch }, errors: { ...canBuilt.errors, ...hwBuilt.errors } });
  const count = $derived(Object.keys(built.patch).length);
  const blocked = $derived(Object.keys(built.errors).length > 0);

  async function apply() {
    if (blocked || count === 0) return;
    if (await settingsStore.save("can", built.patch)) {
      edits = {};
      hwEdits = {};
    }
  }
</script>

{#if anyShown(q, GROUP.vehicle, [ROW.bitrate, ROW.rxQueue, ROW.txQueue, ROW.hwFilter])}
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
          class="in num mono"
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
          class="in num mono"
          type="text"
          inputmode="numeric"
          aria-label={ROW.txQueue.title}
          value={draft.tx}
          aria-invalid={built.errors.tx_queue_depth ? "true" : undefined}
          oninput={(e) => (edits.tx = e.currentTarget.value)}
        />
      </SettingRow>
    {/if}
    {#if hwDraft && show(ROW.hwFilter)}
      <SettingRow title={ROW.hwFilter.title} hint="Takes effect after a restart. Frames the controller rejects never reach the CAN Bus page or the poller." error={built.errors.mfilter_cfg}>
        <Switch label="Filter on" checked={hwDraft.on} onchange={(v) => (hwEdits.on = v)} />
      </SettingRow>
      {#if hwDraft.on}
        <SettingRow title="Accepted IDs" hint="Hex. A mask bit set to 1 must match the same bit of the ID." stack error={built.errors.mfilter_cfg}>
          <div class="hw">
            <div class="hw-fields">
              <div class="hw-f">
                <span class="st-lab">Length</span>
                <SegControl label="ID length" options={ID_LENGTHS} value={hwDraft.ext ? "ext" : "std"} onChange={(v) => (hwEdits.ext = v === "ext")} />
              </div>
              <label class="hw-f">
                <span class="st-lab">ID</span>
                <input class="in num mono" type="text" spellcheck="false" autocomplete="off" value={hwDraft.id} aria-invalid={built.errors.mfilter_cfg ? "true" : undefined} oninput={(e) => (hwEdits.id = e.currentTarget.value)} />
              </label>
              <label class="hw-f">
                <span class="st-lab">Mask</span>
                <input class="in num mono" type="text" spellcheck="false" autocomplete="off" value={hwDraft.mask} aria-invalid={built.errors.mfilter_cfg ? "true" : undefined} oninput={(e) => (hwEdits.mask = e.currentTarget.value)} />
              </label>
            </div>
            <p class="hw-pre" aria-live="polite">
              {#if preview}
                <span class="hw-bits mono">{preview.pattern}</span>
                <span class="hw-count">{preview.count === 1 ? "1 ID" : preview.count === 2 ** (hwDraft.ext ? 29 : 11) ? "Every ID" : `${preview.count.toLocaleString("en")} IDs`}</span>
              {:else}
                <span class="muted">Enter hex values to see what passes.</span>
              {/if}
            </p>
          </div>
        </SettingRow>
      {/if}
    {/if}
  </div>
</div>
{/if}

<ApplyBar {count} busy={settingsStore.saving} {blocked} onRevert={() => {
  edits = {};
  hwEdits = {};
}} onApply={apply} />

<style>
  .hw {
    display: grid;
    gap: 12px;
    width: 100%;
    min-width: 0;
  }
  .hw-fields {
    display: flex;
    flex-wrap: wrap;
    align-items: flex-end;
    gap: 12px;
  }
  .hw-f {
    display: grid;
    gap: 4px;
    min-width: 0;
  }
  .hw .in {
    width: 8rem;
    max-width: 100%;
  }
  .hw-pre {
    display: flex;
    flex-wrap: wrap;
    align-items: baseline;
    gap: 6px 14px;
    margin: 0;
    padding: 10px 14px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  .hw-bits {
    font-weight: 800;
    letter-spacing: 0.06em;
    color: var(--accent-text);
  }
  .hw-count {
    color: var(--muted);
    font-weight: 700;
    font-size: 0.88rem;
  }
</style>
