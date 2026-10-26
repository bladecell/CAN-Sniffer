<script lang="ts">
import { vehicleStore } from "$lib/vehicleStore.svelte";

  import { alertStore } from "$lib/alertStore.svelte";
  import { telemetryStore } from "$lib/telemetryStore.svelte";
  import { parseJsonResponse } from "$lib/api";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import FileTree from "./FileTree.svelte";
  import Sheet from "./Sheet.svelte";
  import type { SdTreeNode } from "$lib/types";

  interface Props {
    isOpen: boolean;
    onClose: () => void;
  }

  let { isOpen, onClose }: Props = $props();

  let tree = $state<SdTreeNode | null>(null);
  let loadingTree = $state(false);
  let fetchingFile = $state(false);

  $effect(() => {
    if (isOpen && vehicleStore.sdInfo?.is_mounted) {
      loadTree();
    }
  });

  async function loadTree() {
    loadingTree = true;
    try {
      const response = await firmwareApi.sdTree();
      tree = await parseJsonResponse(response);
    } catch (e) {
      const reason = e instanceof Error ? `: ${e.message}` : "";
      alertStore.add(`Failed to fetch SD card tree${reason}`, "error");
    } finally {
      loadingTree = false;
    }
  }

  async function handleFileSelect(path: string) {
    if (!path.endsWith(".json")) {
      alertStore.add("Please select a JSON file.", "warning");
      return;
    }

    fetchingFile = true;
    try {
      const response = await firmwareApi.downloadSdFile(path);
      if (!response.ok) {
        console.error("HTTP error:", response.status, response.statusText);
        throw new Error("HTTP " + response.status);
      }
      const rawText = await response.text();
      console.log("File content preview:", rawText.substring(0, 100));
      const result = JSON.parse(rawText);
      const importedCount = telemetryStore.importPidDefinitions(result);
      alertStore.add(
        `Successfully imported ${importedCount} PID${importedCount === 1 ? "" : "s"}.`,
        "success",
      );
      onClose();
    } catch (e) {
      console.error("SD Card Import Exception:", e);
      alertStore.add(
        "Failed to load or parse JSON file from SD card.",
        "error",
      );
    } finally {
      fetchingFile = false;
    }
  }
</script>

<Sheet open={isOpen} title="Select PID Configuration File" onClose={onClose}>
  {#if !vehicleStore.sdInfo?.is_mounted}
    <div class="alert-box warning">
      SD Card is not mounted or present.
    </div>
  {:else if loadingTree}
    <div class="alert-box info">Scanning SD Card...</div>
  {:else if tree}
    <div class="tree">
      <FileTree
        node={tree}
        onFileSelect={handleFileSelect}
        allowedExtensions={[".json"]}
        onUploadSuccess={loadTree}
      />
    </div>
  {:else}
    <div class="alert-box error">Failed to load directory tree.</div>
  {/if}
</Sheet>

<style>
  .alert-box {
    padding: 1rem;
    border-radius: var(--r-ctl);
    background: var(--card);
    color: var(--muted);
    text-align: center;
    font-weight: 700;
  }
  /* Tree labels sit on control fills; full text colour keeps them above 4.5:1 (muted text does not). */
  .tree {
    --pico-muted-color: var(--text);
  }
  .alert-box.warning {
    color: var(--warning-color);
  }
  .alert-box.error {
    color: var(--error-color);
  }
</style>
