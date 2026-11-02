<script lang="ts">
  import { onMount } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import Dialog from "$lib/components/Dialog.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { parseJsonResponse } from "$lib/api";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import {
    GROUP,
    ROW,
    anyShown,
    buildPidPathPatch,
    parseSdInfo,
    rowShown,
    sdView,
    type RowMeta,
    type SdInfoWire,
    type SettingsDoc,
  } from "$lib/settingsModel";
  import ApplyBar from "./ApplyBar.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q, doc }: { q: string; doc: SettingsDoc | null } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.storage, row);
  const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

  // GET /sd_card/info is open. 503 means the card is busy (the recorder or a format holds it): retry, not an error.
  type CardState = { kind: "loading" } | { kind: "busy" } | { kind: "error"; message: string } | { kind: "ok"; info: SdInfoWire };
  let card = $state<CardState>({ kind: "loading" });

  async function loadCard() {
    card = { kind: "loading" };
    try {
      const res = await firmwareApi.sdInfo();
      if (res.status === 503) {
        card = { kind: "busy" };
        return;
      }
      const info = parseSdInfo(await parseJsonResponse<unknown>(res));
      card = info === null ? { kind: "error", message: "Unexpected reply" } : { kind: "ok", info };
    } catch (e) {
      card = { kind: "error", message: errorText(e) };
    }
  }
  onMount(() => void loadCard());

  const view = $derived(card.kind === "ok" ? sdView(card.info) : null);
  const cardHint = $derived.by(() => {
    switch (card.kind) {
      case "loading":
        return "Reading the card";
      case "busy":
        return "The card is busy (recording or formatting). Try again in a moment.";
      case "error":
        return `Could not read the card: ${card.message}`;
      case "ok":
        if (view?.kind === "no-card") return "No card in the slot. Insert one, then check again.";
        if (view?.kind === "not-mounted") return "The card is present but not mounted. Check the card, then check again.";
        return `${view?.kind === "ok" ? view.usedGb : ""} of ${view?.kind === "ok" ? view.totalGb : ""} GB used`;
    }
  });

  let folder = $state<string | null>(null);
  const draft = $derived(folder ?? doc?.system.pid_def_path ?? "");
  const built = $derived(doc ? buildPidPathPatch(doc.system, draft) : { patch: {}, errors: {} });
  const count = $derived(Object.keys(built.patch).length);
  const blocked = $derived(Object.keys(built.errors).length > 0);

  async function apply() {
    if (blocked || count === 0) return;
    if (await settingsStore.save("system", built.patch)) folder = null;
  }

  let confirmFormat = $state(false);
  let formatting = $state(false);
  async function format() {
    confirmFormat = false;
    formatting = true;
    try {
      await parseJsonResponse<unknown>(await firmwareApi.formatSdCard());
      alertStore.add("SD card formatted", "success");
      await loadCard();
    } catch (e) {
      alertStore.add(`Format failed: ${errorText(e)}`, "error");
    } finally {
      formatting = false;
    }
  }
</script>

<div class="st-sec">
  <h2>Card</h2>
  {#if anyShown(q, GROUP.storage, [ROW.sdUsage])}
  <div class="st-list">
    {#if show(ROW.sdUsage)}
      <SettingRow title={ROW.sdUsage.title} hint={cardHint} stack>
        {#if view?.kind === "ok"}
          <div class="st-meter" role="meter" aria-label="Card used" aria-valuemin="0" aria-valuemax="100" aria-valuenow={view.percent}>
            <i style="width: {view.percent}%"></i>
          </div>
        {:else if card.kind === "busy" || card.kind === "error"}
          <button type="button" class="st-btn sm" onclick={loadCard}><Icon name="reload" size={16} />Check again</button>
        {/if}
      </SettingRow>
    {/if}
  </div>
  {/if}
</div>

<div class="st-sec">
  <h2>Files</h2>
  {#if anyShown(q, GROUP.storage, [ROW.pidFolder, ROW.sdFormat])}
  <div class="st-list">
    {#if doc && show(ROW.pidFolder)}
      <SettingRow title={ROW.pidFolder.title} hint="Where Open file and Save selected look for .json files." stack error={built.errors.pid_def_path}>
        <input
          class="st-in mono"
          type="text"
          aria-label={ROW.pidFolder.title}
          value={draft}
          aria-invalid={built.errors.pid_def_path ? "true" : undefined}
          oninput={(e) => (folder = e.currentTarget.value)}
        />
      </SettingRow>
    {/if}
    {#if show(ROW.sdFormat)}
      <SettingRow title={ROW.sdFormat.title} hint="Erases every file on the card. Trips and PID files are lost.">
        <button type="button" class="st-btn danger" onclick={() => (confirmFormat = true)} disabled={formatting}>
          <Icon name="trash" size={16} />Format…
        </button>
      </SettingRow>
    {/if}
  </div>
  {/if}
</div>

<ApplyBar {count} busy={settingsStore.saving} {blocked} onRevert={() => (folder = null)} onApply={apply} />

<Dialog
  open={confirmFormat}
  title="Format the SD card?"
  message="Every file on the card is deleted for good, including trips and PID files. This cannot be undone."
  confirmLabel="Format card"
  danger
  onConfirm={format}
  onCancel={() => (confirmFormat = false)}
/>
