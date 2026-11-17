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
              <div class="hw-f hw-len">
                <span class="st-lab">Length</span>
                <SegControl label="ID length" options={ID_LENGTHS} value={hwDraft.ext ? "ext" : "std"} onChange={(v) => (hwEdits.ext = v === "ext")} />
              </div>
              <label class="hw-f hw-id">
                <span class="st-lab">ID</span>
                <input class="in num mono" type="text" spellcheck="false" autocomplete="off" value={hwDraft.id} aria-invalid={built.errors.mfilter_cfg ? "true" : undefined} oninput={(e) => (hwEdits.id = e.currentTarget.value)} />
              </label>
              <label class="hw-f hw-id">
                <span class="st-lab">Mask</span>
                <input class="in num mono" type="text" spellcheck="false" autocomplete="off" value={hwDraft.mask} aria-invalid={built.errors.mfilter_cfg ? "true" : undefined} oninput={(e) => (hwEdits.mask = e.currentTarget.value)} />
              </label>
            </div>
            <div class="hw-pre" aria-live="polite">
              {#if preview}
                <div class="hw-bits" role="img" aria-label="Bit pattern {preview.pattern}">
                  {#each preview.pattern.split(" ") as group, g (g + g)}
                    <span class="hw-group">
                      {#each [...group] as bit, b (b)}
                        <span class="hw-bit" class:hw-one={bit === "1"} class:hw-any={bit === "x"}>{bit}</span>
                      {/each}
                    </span>
                  {/each}
                </div>
                <span class="hw-count">{preview.count === 1 ? "1 ID" : preview.count === 2 ** (hwDraft.ext ? 29 : 11) ? "Every ID" : `${preview.count.toLocaleString("en")} IDs`}</span>
                <span class="hw-legend">1 and 0 must match, x is open</span>
              {:else}
                <span class="muted">Enter hex values to see what passes.</span>
              {/if}
            </div>
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
  /* Length, ID and Mask share one top line: labels align, controls align at --ctl-h. Wraps to two rows when narrow. */
  .hw-fields {
    display: flex;
    flex-wrap: wrap;
    align-items: start;
    gap: 12px;
  }
  .hw-f {
    display: grid;
    gap: 6px;
    align-content: start;
    min-width: 0;
  }
  .hw-len {
    flex: 1 1 15rem;
  }
  .hw-id {
    flex: 1 1 7rem;
  }
  .hw-len :global(.seg) {
    display: flex;
    width: 100%;
    min-height: var(--ctl-h);
    box-sizing: border-box;
  }
  .hw-len :global(.seg > button) {
    flex: 1 1 0;
    min-height: 0;
    align-self: stretch;
    padding: 0 10px;
  }
  .hw .in {
    width: 100%;
    min-height: var(--ctl-h);
  }
  .hw-pre {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 10px 14px;
    margin: 0;
    padding: 10px 14px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  .hw-bits {
    display: flex;
    flex-wrap: wrap;
    gap: 6px 10px;
  }
  .hw-group {
    display: inline-flex;
    gap: 3px;
  }
  .hw-bit {
    display: grid;
    place-items: center;
    width: 1.3rem;
    height: 1.7rem;
    border-radius: 6px;
    background: var(--ctl);
    color: var(--ctl-sub);
    font-family: var(--mono);
    font-size: 0.85rem;
    font-weight: 800;
  }
  .hw-bit.hw-one {
    background: var(--accent);
    color: var(--on-accent);
  }
  .hw-bit.hw-any {
    background: transparent;
    box-shadow: inset 0 0 0 1.5px var(--muted);
    color: var(--muted);
  }
  .hw-count {
    color: var(--accent-text);
    font-weight: 800;
    font-size: 0.9rem;
    white-space: nowrap;
  }
  .hw-legend {
    color: var(--muted);
    font-size: 0.82rem;
    font-weight: 600;
  }
</style>
