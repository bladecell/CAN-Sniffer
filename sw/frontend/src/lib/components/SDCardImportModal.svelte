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

      <div class="modal-body-content" style="padding: 1.5rem; max-height: 60vh; overflow-y: auto;">
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
  .custom-modal-backdrop {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
    background: rgba(0, 0, 0, 0.6);
    backdrop-filter: blur(4px);
    display: flex;
    justify-content: center;
    align-items: center;
    z-index: 1000;
  }
  
  .pico-orange-glass-modal {
    background: rgba(30, 30, 30, 0.85);
    border: 1px solid rgba(249, 115, 22, 0.3);
    border-radius: 12px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5), inset 0 1px 0 rgba(255, 255, 255, 0.1);
    width: 90%;
    max-width: 600px;
    display: flex;
    flex-direction: column;
  }

  .modal-header-hull {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 1rem 1.5rem;
    border-bottom: 1px solid rgba(255, 255, 255, 0.1);
    background: linear-gradient(to right, rgba(249, 115, 22, 0.1), transparent);
    border-top-left-radius: 12px;
    border-top-right-radius: 12px;
  }

  .modal-title-heading {
    margin: 0;
    color: #f97316;
    font-size: 1.1rem;
    font-weight: 600;
  }

  .close-modal-x {
    background: none;
    border: none;
    color: rgba(255, 255, 255, 0.6);
    font-size: 1.2rem;
    cursor: pointer;
    transition: color 0.2s;
  }

  .close-modal-x:hover {
    color: #fff;
  }

  .file-item {
    display: flex;
    justify-content: space-between;
    padding: 0.4rem 0.5rem;
    border-radius: 6px;
    font-size: 0.9rem;
    transition: background 0.2s;
  }

  .file-item.clickable {
    cursor: pointer;
  }

  .file-item.clickable:hover {
    background: rgba(255, 255, 255, 0.1);
  }

  .file-size {
    color: rgba(255, 255, 255, 0.4);
    font-size: 0.8rem;
  }
</style>
