<script lang="ts">
  import { untrack } from "svelte";
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { parseJsonResponse, parseJsonResponseAllowEmpty } from "$lib/api";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import { sdFolder } from "$lib/sdFolder.svelte";
  import { normalizePidDefinition } from "$lib/pidDefinition";
  import type { PidDefinition } from "$lib/types";
  import { classifyPidImport, pidImportCounts } from "$lib/pidImport";
  import {
    MAX_PID_FILE_BYTES,
    cardFilePath,
    parsePidFile,
    readFolder,
    serializePidFile,
    validFileName,
    type FolderListing,
    type SdFile,
  } from "$lib/sdPidFiles";
  import Dialog from "$lib/components/Dialog.svelte";
  import Sheet from "$lib/components/Sheet.svelte";
  import Icon from "$lib/Icon.svelte";
  import PidFileImportSheet from "./PidFileImportSheet.svelte";
  import { swipeRow } from "./swipeRow";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  interface Props {
    /** Phone: a swipe left opens a file and right deletes it. The Open and Delete buttons are hidden. */
    phone?: boolean;
    onSettings: () => void;
  }

  let { phone = false, onSettings }: Props = $props();

  type Listing = FolderListing | { kind: "loading" };
  let listing = $state<Listing>({ kind: "loading" });
  let loadToken = 0;
  let selected = $state<string | null>(null);
  let busy = $state(false);
  let openError = $state<{ name: string; reason: string } | null>(null);
  let confirm = $state<"overwrite" | "delete" | null>(null);
  let newOpen = $state(false);
  let newName = $state("");
  let newError = $state("");
  /** The file read by Open, listed in the import sheet until it is closed or imported. */
  let preview = $state<{ name: string; defs: PidDefinition[] } | null>(null);
  let importOpen = $state(false);

  const folder = $derived(sdFolder.path);
  const files = $derived<SdFile[]>(listing.kind === "ok" ? listing.files : []);
  const pidCount = $derived(telemetryStore.local_piddef.filter((row) => row.loaded && row.selected).length);
  const readable = $derived(listing.kind === "ok");
  const fileSelected = $derived(selected !== null && files.some((file) => file.name === selected));
  const canOpen = $derived(readable && fileSelected && !busy);
  const canSave = $derived(readable && fileSelected && pidCount > 0 && !busy);
  const canNew = $derived(readable && pidCount > 0 && !busy);
  const canDelete = $derived(readable && fileSelected && !busy);

  const plural = (n: number, word: string) => `${n} ${word}${n === 1 ? "" : "s"}`;

  const subline = $derived.by(() => {
    if (listing.kind === "nocard") return "No card detected";
    if (listing.kind === "ok") return `${folder} · ${plural(listing.files.length, "pid config file")}`;
    return folder;
  });

  const hint = $derived.by(() => {
    if (listing.kind === "loading") return "Reading the card.";
    if (!readable) return "Actions need a card and a valid folder.";
    if (!fileSelected) return "Select a file first.";
    if (pidCount === 0) return "No PIDs selected.";
    return `${plural(pidCount, "PID")} selected on the PIDs tab.`;
  });

  function formatSize(bytes: number): string {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / 1024 / 1024).toFixed(1)} MB`;
  }

  async function load(path: string) {
    const token = ++loadToken;
    listing = { kind: "loading" };
    const next = await readFolder(path, (treePath) => firmwareApi.sdTree(treePath, { depth: 0, limit: 512 }));
    if (token !== loadToken) return;
    listing = next;
    if (selected !== null && !(next.kind === "ok" && next.files.some((file) => file.name === selected))) selected = null;
  }

  $effect(() => {
    const path = sdFolder.path;
    untrack(() => void load(path));
  });

  function pick(name: string) {
    selected = selected === name ? null : name;
    openError = null;
  }

  /** The PIDs ticked on the PIDs tab, as the file holds them. */
  function selectedDefinitions(): PidDefinition[] {
    return telemetryStore.local_piddef.filter((row: PidRow) => row.loaded && row.selected).map((row) => normalizePidDefinition(row.def));
  }

  /** A file of the card as PID definitions. Any failure is a reason for the row, and nothing is added. */
  async function readPidFile(path: string): Promise<PidDefinition[]> {
    const response = await firmwareApi.downloadSdFile(path);
    if (!response.ok) throw new Error(response.status === 404 ? "The file is not on the card." : `HTTP ${response.status}.`);
    const declared = Number(response.headers.get("content-length"));
    if (declared > MAX_PID_FILE_BYTES) throw new Error("The file is larger than 64 KiB.");
    const defs = parsePidFile(await response.text());
    if (defs.length === 0) throw new Error("The file holds no PIDs.");
    return defs;
  }

  /** Writes a PID list to a card file. Returns the failure reason, or null. */
  async function writePidFile(path: string, defs: PidDefinition[]): Promise<string | null> {
    try {
      const text = serializePidFile(defs);
      if (new TextEncoder().encode(text).byteLength > MAX_PID_FILE_BYTES) return "The PIDs are larger than 64 KiB.";
      await parseJsonResponse(await firmwareApi.uploadSdFile(path, new Blob([text], { type: "application/json" })));
      return null;
    } catch (e) {
      return e instanceof Error ? e.message : "The save failed.";
    }
  }

  /** Open file: reads the file and lists its PIDs in a sheet. Nothing is added until the user imports them. */
  async function openFile(name = selected) {
    if (!name || !canOpen) return;
    busy = true;
    try {
      const defs = await readPidFile(cardFilePath(folder, name));
      openError = null;
      preview = { name, defs };
      importOpen = true;
    } catch (e) {
      openError = { name, reason: e instanceof Error ? e.message : "The file could not be read." };
    } finally {
      busy = false;
    }
  }

  /** The chosen PIDs go into the list as drafts, to review in Changes. The device is not touched. */
  function importChosen(chosen: PidDefinition[]) {
    if (!preview || chosen.length === 0) return;
    const { name } = preview;
    const { added, changed } = pidImportCounts(classifyPidImport(chosen, telemetryStore.local_piddef));
    telemetryStore.importPidDefinitions(chosen);
    importOpen = false;
    alertStore.add(
      added + changed === 0 ? `${name} matches your PIDs.` : `${name}: ${added} added, ${changed} changed. Review them in Changes.`,
      "success",
    );
  }

  async function saveSelected() {
    confirm = null;
    if (!selected || !canSave) return;
    const name = selected;
    busy = true;
    const count = pidCount;
    try {
      const reason = await writePidFile(cardFilePath(folder, name), selectedDefinitions());
      if (reason) alertStore.add(`Could not save ${name}: ${reason}`, "error");
      else alertStore.add(`Saved ${plural(count, "PID")} to ${name}`, "success");
    } catch (e) {
      alertStore.add(`Could not save ${name}: ${e instanceof Error ? e.message : "unknown error"}`, "error");
    } finally {
      busy = false;
    }
    await load(folder);
  }

  function closeNew() {
    newOpen = false;
    newName = "";
    newError = "";
  }

  async function createFile() {
    if (!canNew) return;
    const name = validFileName(newName);
    if (!name.ok) {
      newError = name.reason;
      return;
    }
    if (files.some((file) => file.name.toLowerCase() === name.name.toLowerCase())) {
      newError = "A file with this name exists.";
      return;
    }
    busy = true;
    let reason: string | null;
    try {
      reason = await writePidFile(cardFilePath(folder, name.name), selectedDefinitions());
    } catch (e) {
      reason = e instanceof Error ? e.message : "The save failed.";
    }
    busy = false;
    if (reason) {
      newError = reason;
      return;
    }
    closeNew();
    selected = name.name;
    alertStore.add(`Created ${name.name}`, "success");
    await load(folder);
  }

  async function deleteSelected() {
    confirm = null;
    if (!selected || !canDelete) return;
    const name = selected;
    busy = true;
    try {
      await parseJsonResponseAllowEmpty(await firmwareApi.deleteSdFile(cardFilePath(folder, name)));
      selected = null;
      alertStore.add(`Deleted ${name}`, "success");
    } catch (e) {
      alertStore.add(`Could not delete ${name}: ${e instanceof Error ? e.message : "unknown error"}`, "error");
    } finally {
      busy = false;
    }
    await load(folder);
  }

  // Phone swipes act on the card they were made on.
  function swipeOpen(name: string) {
    selected = name;
    void openFile(name);
  }
  function swipeDelete(name: string) {
    selected = name;
    confirm = "delete";
  }
</script>

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={SIDEBAR_HINTS.sd} />
    <h2>SD card</h2>
    <p class="sb-counts num">{subline}</p>
    <div class="sb-selrow"><span>{fileSelected ? "1 file selected" : "No file selected"}</span></div>
  {/snippet}

  {#if listing.kind === "loading"}
    <div class="sb-empty" role="status"><b>Reading the card</b></div>
  {:else if listing.kind === "nocard"}
    <div class="sb-empty">
      <b>No SD card</b>The slot is empty, or the card could not be read. Insert a FAT card and wait a few seconds. The PIDs on the device are not affected.
      <div class="sb-act"><button type="button" class="tel-btn" onclick={() => load(folder)}>Check again</button></div>
    </div>
  {:else if listing.kind === "missing"}
    <div class="sb-empty">
      <b>Folder not found</b><span class="tel-mono">{folder}</span> is not on the card. Choose another folder in Settings.
      <div class="sb-act"><button type="button" class="tel-btn" onclick={onSettings}><Icon name="sliders" size={18} />Open Settings</button></div>
    </div>
  {:else if listing.kind === "error"}
    <div class="sb-empty">
      <b>Could not read the folder</b>{listing.reason}
      <div class="sb-act"><button type="button" class="tel-btn" onclick={() => load(folder)}>Try again</button></div>
    </div>
  {:else if files.length === 0}
    <div class="sb-empty">
      <b>No .json files here</b><span class="tel-mono">{folder}</span> has none. Save the selected PIDs as a new file, or pick another folder.
      <div class="sb-act"><button type="button" class="tel-btn" onclick={onSettings}><Icon name="sliders" size={18} />Open Settings</button></div>
    </div>
  {:else}
    <ul class="sb-list" aria-label=".json files in {folder}">
      {#each files as file (file.name)}
        {@const on = selected === file.name}
        <li class="sb-row" class:sel={on} class:phone>
          {#if phone}
            <div class="sb-lay sb-lay-l sb-lay-del" aria-hidden="true">Delete</div>
            <div class="sb-lay sb-lay-r sb-lay-apply" aria-hidden="true">Open</div>
          {/if}
          <div class="sb-slide" {@attach swipeRow({ left: () => swipeOpen(file.name), right: () => swipeDelete(file.name) })}>
            <button type="button" class="sb-main file" aria-pressed={on} onclick={() => pick(file.name)}>
              <i class="sb-sw" class:off={!on} style:--sw={on ? 'var(--accent)' : 'transparent'}></i>
              <span class="sb-nm">
                <span class="sb-nl"><b class="fname">{file.name}</b></span>
                <span class="sb-hx">{formatSize(file.size)}</span>
                {#if openError?.name === file.name}
                  <span class="tel-msg bad" role="alert">{openError.reason} Nothing was added.</span>
                {/if}
              </span>
            </button>
          </div>
          {#if phone}
            <!-- Keyboard path to the swipe actions. Hidden until focused, then the card slides to show the action. -->
            <button type="button" class="sb-sra sb-sra-l" onclick={() => swipeDelete(file.name)}>Delete {file.name}</button>
            <button type="button" class="sb-sra sb-sra-r" onclick={() => swipeOpen(file.name)}>Open {file.name}</button>
          {/if}
        </li>
      {/each}
    </ul>
  {/if}

  {#snippet footer()}
    <p class="fhint float-layer" aria-live="polite">{hint}</p>
    {#if !phone}
      <button type="button" class="tel-btn pri" aria-disabled={!canOpen} onclick={() => openFile()}>Open file</button>
    {/if}
    <button type="button" class="tel-btn" class:pri={phone} aria-disabled={!canSave} onclick={() => canSave && (confirm = "overwrite")}>
      Save selected PIDs
    </button>
    <button type="button" class="tel-btn" aria-disabled={!canNew} onclick={() => canNew && (newOpen = true)}>New file</button>
    {#if !phone}
      <button type="button" class="tel-btn danger" aria-disabled={!canDelete} onclick={() => canDelete && (confirm = "delete")}>
        Delete file
      </button>
    {/if}
  {/snippet}
</SidebarPanel>

<Dialog
  open={confirm === "delete"}
  title="Delete {selected ?? 'this file'}?"
  message="This removes the file from the SD card. The PIDs on the PIDs tab and on the device stay."
  confirmLabel="Delete file"
  danger
  onConfirm={deleteSelected}
  onCancel={() => (confirm = null)}
/>
<Dialog
  open={confirm === "overwrite"}
  title="Overwrite {selected ?? 'this file'}?"
  message="The file will hold the {plural(pidCount, 'PID')} selected on the PIDs tab. What it holds now is replaced."
  confirmLabel="Overwrite file"
  danger
  onConfirm={saveSelected}
  onCancel={() => (confirm = null)}
/>

{#if preview}
  <PidFileImportSheet
    open={importOpen}
    fileName={preview.name}
    defs={preview.defs}
    current={telemetryStore.local_piddef}
    onImport={importChosen}
    onClose={() => (importOpen = false)}
  />
{/if}

<Sheet open={newOpen} title="New PID file" onClose={closeNew}>
  <form
    class="nf"
    onsubmit={(e) => {
      e.preventDefault();
      void createFile();
    }}
  >
    <label for="nf-name">File name</label>
    <div class="nf-row">
      <input
        id="nf-name"
        class="tel-in tel-mono"
        bind:value={newName}
        maxlength="40"
        autocomplete="off"
        autocapitalize="off"
        spellcheck="false"
        aria-invalid={newError === "" ? undefined : "true"}
        aria-describedby="nf-msg"
      />
      <span class="tel-mono">.json</span>
    </div>
    <p id="nf-msg" class="tel-msg" class:bad={newError !== ""} aria-live="polite">{newError || "Letters, digits, - and _."}</p>
    <p class="tel-note">Creates a file in <span class="tel-mono">{folder}</span> from the {plural(pidCount, "PID")} selected on the PIDs tab.</p>
  </form>
  {#snippet footer()}
    <button type="button" class="btn" onclick={closeNew}>Cancel</button>
    <button type="button" class="btn pri" aria-disabled={!canNew} onclick={createFile}>Create file</button>
  {/snippet}
</Sheet>

<style>
  .sb-act {
    margin-top: 14px;
  }
  .sb-empty .tel-btn {
    display: inline-flex;
  }
  .sb-main.file {
    grid-template-columns: 6px minmax(0, 1fr);
  }
  .fname {
    overflow-wrap: anywhere;
    white-space: normal;
  }
  .fhint {
    grid-column: 1 / -1;
    margin: 0;
    padding: 6px 12px;
    border-radius: 14px;
    font-size: 0.82rem;
    font-weight: 600;
    line-height: 1.35;
    color: var(--muted);
  }
  .nf {
    display: grid;
    gap: 10px;
  }
  .nf label {
    font-size: 0.95rem;
    font-weight: 800;
  }
  .nf-row {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: center;
    gap: 8px;
  }
  .nf p {
    margin: 0;
  }
</style>
