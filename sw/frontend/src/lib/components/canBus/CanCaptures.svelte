<script lang="ts">
  import { untrack } from "svelte";
  import SidebarHint from "$lib/components/telemetry/SidebarHint.svelte";
  import Icon from "$lib/Icon.svelte";
  import { auth } from "$lib/auth.svelte";
  import { captures } from "$lib/canBus/captureStore.svelte";
  import { captureEndedText, formatBytes } from "$lib/canBus/canCapture";
  import { alertStore } from "$lib/alertStore.svelte";

  let pendingDelete = $state<string | null>(null);

  // The list is gated: read it once the tab opens and the user is logged in.
  $effect(() => {
    if (auth.canUseGated) untrack(() => void captures.refreshList());
  });

  const last = $derived(captures.status ? captureEndedText(captures.status.ended, captures.status.error) : "");

  async function download(name: string) {
    const err = await captures.download(name);
    if (err) alertStore.add(`Download failed: ${err}`, "error");
  }

  async function remove(name: string) {
    pendingDelete = null;
    const err = await captures.remove(name);
    if (err) alertStore.add(`Not deleted: ${err}`, "error");
  }
</script>

<div class="cc">
  <div>
    <div class="h-hint"><h2>Captures</h2><SidebarHint inline tone="warn" label="Capture limits" text="A capture stops at 8 MB, when the stream ends, or when the card is full." /></div>
    <p class="muted">CSV files on the SD card. Start and stop with Save to SD in the toolbar.</p>
  </div>

  {#if captures.capturing}
    <p class="t-ok">Capturing {captures.status?.file ?? ""}: {captures.status?.rows ?? 0} rows, {formatBytes(captures.status?.bytes ?? 0)}</p>
  {:else if last}
    <p class="muted">Last capture: {last}</p>
  {/if}

  {#if !auth.canUseGated}
    <button class="btn pri" onclick={() => void auth.ensureLogin()}>Log in to see captures</button>
  {:else if captures.listError}
    <p class="t-crit" role="alert">{captures.listError}</p>
  {:else if captures.files.length === 0}
    <p class="muted">No capture files yet.</p>
  {:else}
    <ul class="cc-list">
      {#each captures.files as f (f.name)}
        <li>
          <span>
            <b class="mono">{f.name}</b>
            <small>{formatBytes(f.bytes)}</small>
          </span>
          <span class="cc-actions">
            <button class="btn sm" aria-label="Download {f.name}" onclick={() => download(f.name)}>
              <Icon name="download" size={16} />
            </button>
            {#if pendingDelete === f.name}
              <button class="btn sm fill-crit" onclick={() => remove(f.name)}>Delete</button>
              <button class="btn sm" onclick={() => (pendingDelete = null)}>Keep</button>
            {:else}
              <button class="btn sm" aria-label="Delete {f.name}" disabled={captures.capturing && captures.status?.file === f.name} onclick={() => (pendingDelete = f.name)}>
                <Icon name="trash" size={16} />
              </button>
            {/if}
          </span>
        </li>
      {/each}
    </ul>
    {#if captures.truncated}
      <p class="muted">Showing the newest 100 files.</p>
    {/if}
  {/if}
</div>

<style>
  .cc {
    display: grid;
    gap: 12px;
    min-width: 0;
  }
  .cc h2 {
    margin: 0;
  }
  .cc p {
    margin: 0;
  }
  .cc-list {
    display: grid;
    gap: 8px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .cc-list li {
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 8px;
    padding: 10px 12px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  .cc-list small {
    display: block;
    color: var(--muted);
    font-weight: 600;
  }
  .cc-actions {
    display: flex;
    gap: 6px;
  }
</style>
