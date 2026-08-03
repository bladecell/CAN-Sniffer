<script lang="ts">
  import { canStore } from "$lib/canStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { telemetryStore } from "$lib/telemetryStore.svelte";

  interface Props {
    isOpen: boolean;
    onClose: () => void;
  }

  let { isOpen, onClose }: Props = $props();

  let tree = $state<any>(null);
  let loadingTree = $state(false);
  let fetchingFile = $state(false);

  $effect(() => {
    if (isOpen && canStore.sdInfo?.is_mounted) {
      loadTree();
    }
  });

  async function loadTree() {
    loadingTree = true;
    try {
      const response = await fetch("/api/v1/sd_card/tree");
      const result = await response.json();
      if (result.status === "success") {
        tree = result;
      }
    } catch (e) {
      alertStore.add("Failed to fetch SD card tree", "error");
    } finally {
      loadingTree = false;
    }
  }

  async function handleFileSelect(path: string) {
    if (!path.endsWith('.json')) {
      alertStore.add("Please select a JSON file.", "warning");
      return;
    }

    fetchingFile = true;
    try {
      // The API streams the file directly without needing mountpoint
      // path example: /CONFIG/dtcs.json -> /api/v1/sd_card/file/CONFIG/dtcs.json
      const endpoint = "/api/v1/sd_card/file" + path;
      const response = await fetch(endpoint);
      if (!response.ok) throw new Error("Fetch failed");
      const result = await response.json();
      telemetryStore.importPidDefinitions(result);
      alertStore.add("Successfully loaded PIDs from SD card. Review and click Update.", "success");
      onClose();
    } catch (e) {
      alertStore.add("Failed to load or parse JSON file from SD card.", "error");
    } finally {
      fetchingFile = false;
    }
  }
</script>

{#if isOpen}
  <!-- svelte-ignore a11y_click_events_have_key_events -->
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
  <div class="custom-modal-backdrop" onclick={onClose}>
    <article
      onclick={(e) => e.stopPropagation()}
      class="pico-orange-glass-modal blur-background"
    >
      <header class="modal-header-hull">
        <h5 class="modal-title-heading">Select PID Configuration File</h5>
        <button aria-label="Close" class="close-modal-x" onclick={onClose}>✕</button>
      </header>

      <div class="modal-form-body" style="max-height: 60vh; overflow-y: auto; padding: 24px;">
        {#if !canStore.sdInfo?.is_mounted}
          <div class="alert-box warning">SD Card is not mounted or present.</div>
        {:else if loadingTree}
          <div class="alert-box info">Scanning SD Card...</div>
        {:else if tree}
          <div class="tree-container">
            <!-- Recursive Tree Renderer -->
            {#snippet renderNode(node)}
              <div class="tree-node" style="margin-left: 1rem; margin-bottom: 0.25rem;">
                {#if node.children}
                  <div class="folder-name" style="font-weight: 600; color: #f97316;">
                    📁 {node.name || (node.path === '/' ? 'SD Root' : node.path.split('/').pop())}
                  </div>
                  {#each node.children as child}
                    {@render renderNode(child)}
                  {/each}
                {:else}
                  <!-- svelte-ignore a11y_click_events_have_key_events -->
                  <div 
                    class="file-item" 
                    class:clickable={node.name.endsWith('.json')}
                    onclick={() => node.name.endsWith('.json') && handleFileSelect(node.path)}
                  >
                    <span style="color: {node.name.endsWith('.json') ? '#84da89' : 'rgba(255,255,255,0.5)'}">
                      📄 {node.name}
                    </span>
                    <span class="file-size">{(node.size / 1024).toFixed(1)} KB</span>
                  </div>
                {/if}
              </div>
            {/snippet}

            {@render renderNode(tree)}
          </div>
        {:else}
          <div class="alert-box error">Failed to load directory tree.</div>
        {/if}
      </div>
    </article>
  </div>
{/if}


<style>
  /* --- CUSTOM MODAL STYLES (Matching PIDSettingsModal) --- */
  .custom-modal-backdrop {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
    background: rgba(0, 0, 0, 0.4);
    display: flex;
    align-items: center;
    justify-content: center;
    z-index: 12000;
    padding: 16px;
    border-radius: 12px;
  }
  .pico-orange-glass-modal {
    max-width: 500px;
    width: 100%;
    margin: 0;
    padding: 0;
    overflow: hidden;
  }
  .modal-header-hull {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 16px 24px;
    border-bottom: 1px solid var(--pico-border-color);
    background: transparent;
    margin: 0;
  }
  .modal-title-heading {
    margin: 0;
    font-size: 1.1rem;
    font-weight: 600;
  }
  .close-modal-x {
    background: transparent !important;
    border: none !important;
    box-shadow: none !important;
    color: var(--pico-muted-color) !important;
    cursor: pointer;
    font-size: 1.1rem;
    padding: 4px !important;
  }
  .modal-form-body {
    padding: 24px;
  }
  .file-item {
    display: flex;
    justify-content: space-between;
    padding: 0.5rem 0.75rem;
    border-radius: var(--pico-border-radius);
    font-size: 0.9rem;
    transition: background 0.2s;
    border: 1px solid transparent;
  }
  .file-item.clickable {
    cursor: pointer;
  }
  .file-item.clickable:hover {
    background: rgb(from var(--pico-form-element-background-color) r g b / 0.6);
    border-color: var(--pico-border-color);
  }
  .file-size {
    color: var(--pico-muted-color);
    font-size: 0.8rem;
  }
  .alert-box {
    padding: 1rem;
    border-radius: var(--pico-border-radius);
    text-align: center;
    font-weight: 600;
    background: var(--pico-form-element-disabled-background-color);
    border: 1px dashed var(--pico-border-color);
    color: var(--pico-muted-color);
  }
</style>

