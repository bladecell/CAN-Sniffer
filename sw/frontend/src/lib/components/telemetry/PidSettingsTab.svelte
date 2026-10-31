<script lang="ts">
  import { untrack } from "svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import { parseJsonResponse } from "$lib/api";
  import { sdFolder } from "$lib/sdFolder.svelte";
  import { folderChoices, normalizeSdFolder, readFolder, type FolderChoice, type FolderListing } from "$lib/sdPidFiles";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  // The draft is what the input shows. It becomes the folder when committed (Enter, or leaving the field).
  let draft = $state(sdFolder.path);
  const typed = $derived(normalizeSdFolder(draft));
  const deviceCount = $derived(vehicleStore.pidDefinitions.length);

  function commit(input = draft) {
    const result = sdFolder.set(input);
    draft = result.ok ? result.folder : input;
  }

  // Status of the stored folder on the card, as the mockup's line shows it. Only the newest check may write.
  let found = $state<{ tone: "ok" | "warn" | "bad"; text: string }>({ tone: "ok", text: "" });
  let checkToken = 0;

  function describe(listing: FolderListing): { tone: "ok" | "warn" | "bad"; text: string } {
    if (listing.kind === "ok") {
      const n = listing.files.length;
      return n > 0
        ? { tone: "ok", text: `Found. ${n} .json file${n === 1 ? "" : "s"} in it.` }
        : { tone: "ok", text: "Found, but no .json files in it." };
    }
    if (listing.kind === "nocard") return { tone: "warn", text: "No SD card, so the folder cannot be checked." };
    if (listing.kind === "missing") return { tone: "bad", text: "No such folder on the card." };
    return { tone: "bad", text: listing.reason };
  }

  async function check(path: string) {
    const token = ++checkToken;
    const listing = await readFolder(path, (treePath) => firmwareApi.sdTree(treePath, { depth: 0, limit: 512 }));
    if (token === checkToken) found = describe(listing);
  }

  $effect(() => {
    const path = sdFolder.path;
    untrack(() => void check(path));
  });

  const status = $derived(typed.ok ? found : { tone: "bad" as const, text: typed.reason });

  type Browse = { kind: "closed" } | { kind: "loading" } | { kind: "ready"; folders: FolderChoice[] } | { kind: "error"; reason: string };
  let browse = $state<Browse>({ kind: "closed" });

  /** Lists the folders on the card, once per open. The folder picked is committed like a typed one. */
  async function toggleBrowse() {
    if (browse.kind !== "closed") {
      browse = { kind: "closed" };
      return;
    }
    browse = { kind: "loading" };
    try {
      const tree = await parseJsonResponse<unknown>(await firmwareApi.sdTree("", { depth: 5, limit: 512 }));
      browse = { kind: "ready", folders: folderChoices(tree) };
    } catch (e) {
      browse = { kind: "error", reason: e instanceof Error ? e.message : "The card could not be read." };
    }
  }
</script>

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={SIDEBAR_HINTS.settings} />
    <h2>Settings</h2>
    <p class="sb-counts">Where the SD tab looks</p>
  {/snippet}

  <div class="set">
    <section class="fld">
      <label for="sd-folder">SD card folder</label>
      <div class="inrow">
        <input
          id="sd-folder"
          class="tel-in tel-mono"
          bind:value={draft}
          onchange={() => commit()}
          onkeydown={(e) => e.key === "Enter" && commit()}
          spellcheck="false"
          autocomplete="off"
          autocapitalize="off"
          aria-invalid={typed.ok ? undefined : "true"}
          aria-describedby="sd-msg sd-help"
        />
        <button type="button" class="tel-btn choose" aria-expanded={browse.kind !== "closed"} aria-controls="sd-folders" onclick={toggleBrowse}>
          Choose
        </button>
      </div>
      {#if browse.kind !== "closed"}
        <div id="sd-folders" class="blist" role="group" aria-label="Folders on the card">
          {#if browse.kind === "loading"}
            <p class="tel-note bi-msg" role="status">Reading the card.</p>
          {:else if browse.kind === "error"}
            <p class="tel-msg bad bi-msg" role="alert">{browse.reason}</p>
          {:else if browse.folders.length === 0}
            <p class="tel-note bi-msg">No folders on the card.</p>
          {:else}
            {#each browse.folders as choice (choice.folder)}
              <button
                type="button"
                class="bi"
                aria-pressed={sdFolder.path === choice.folder}
                onclick={() => {
                  commit(choice.folder);
                }}
              >
                <span class="tel-mono">{choice.folder}</span>
                <span class="tel-note nowrap">{choice.jsonCount} .json</span>
              </button>
            {/each}
          {/if}
        </div>
      {/if}
      <p id="sd-msg" class="tel-msg {status.tone}" aria-live="polite">{status.text}</p>
      <p id="sd-help" class="tel-note">
        The SD tab lists the .json files in this folder. Kept in this browser, not on the device.
      </p>
    </section>

    <section class="fld">
      <h3 id="dev-h">Stored on the device</h3>
      <output class="ro" aria-labelledby="dev-h">{deviceCount} {deviceCount === 1 ? "PID" : "PIDs"}</output>
      <p class="tel-note">The device keeps its PIDs in its own memory and loads them at start. It does not need the SD card.</p>
    </section>
  </div>
</SidebarPanel>

<style>
  .set {
    display: grid;
    gap: 22px;
    padding: 8px 2px 12px;
  }
  .fld {
    display: grid;
    gap: 6px;
  }
  .fld h3,
  .fld > label {
    margin: 0;
    font-size: 0.95rem;
    font-weight: 800;
  }
  .inrow {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: stretch;
    gap: 8px;
  }
  /* Input and Choose share the control height, as in the mockup (.btn, .ctl: 48px). */
  .inrow .tel-in {
    height: var(--ctl-h);
    min-height: 0;
  }
  .choose {
    height: var(--ctl-h);
    min-height: 0;
    padding: 0 16px;
    margin: 0;
  }
  .ro {
    display: block;
    min-height: var(--ctl-h);
    padding: 12px 14px;
    border-radius: var(--r-ctl);
    background: var(--row);
    font-weight: 700;
    font-variant-numeric: tabular-nums;
  }
  /* Folder browser: rows on a control fill, the picked folder marked with an accent bar. */
  .blist {
    display: grid;
    gap: 4px;
    padding: 6px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  .bi {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    min-height: 48px;
    padding: 0 12px;
    border: 0;
    border-radius: 12px;
    background: none;
    color: var(--text);
    text-align: left;
    cursor: pointer;
  }
  .bi[aria-pressed='true'] {
    background: var(--ctl);
    box-shadow: inset 3px 0 0 var(--accent);
  }
  @media (hover: hover) {
    .bi:hover {
      background: var(--hov);
    }
  }
  .bi-msg {
    padding: 0 6px;
  }
  .nowrap {
    white-space: nowrap;
  }
</style>
