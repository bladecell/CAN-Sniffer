<script lang="ts">
  import FileTree from "./FileTree.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import Icon from "$lib/Icon.svelte";

  interface FileNode {
    name?: string;
    path: string;
    type?: string;
    size?: number;
    children?: FileNode[];
  }

  interface Props {
    node: FileNode;
    isRoot?: boolean;
    allowedExtensions?: string[];
    onFileSelect?: (path: string) => void;
    parentPath?: string;
    onUploadSuccess?: () => void;
  }

  let { node, isRoot = true, allowedExtensions = [".json"], onFileSelect, parentPath = "", onUploadSuccess }: Props = $props();

  let isUploading = $state(false);
  let fileInputForUpload = $state<HTMLInputElement | null>(null);
  let uploadTargetFolder = $state("");

  let confirmDeletePath = $state<string | null>(null);
  let confirmDeleteIsFolder = $state<boolean>(false);

  function triggerDelete(path: string, isFolder: boolean, e: Event) {
    e.preventDefault();
    e.stopPropagation();
    confirmDeletePath = path;
    confirmDeleteIsFolder = isFolder;
  }

  function cancelDelete(e: Event) {
    e.preventDefault();
    e.stopPropagation();
    confirmDeletePath = null;
  }

  async function executeDelete(e: Event) {
    e.preventDefault();
    e.stopPropagation();
    if (!confirmDeletePath) return;

    const path = confirmDeletePath;
    const isFolder = confirmDeleteIsFolder;
    confirmDeletePath = null;

    try {
      const endpoint = "/api/v1/sd_card/file" + (path.startsWith('/') ? path : '/' + path) + (isFolder ? '/' : '');
      const response = await fetch(endpoint, { method: "DELETE" });
      if (!response.ok) throw new Error("Delete failed: " + response.statusText);
      
      alertStore.add(`Successfully deleted ${path}`, "success");
      if (onUploadSuccess) onUploadSuccess();
    } catch (err: any) {
      console.error(err);
      alertStore.add("Failed to delete: " + err.message, "error");
    }
  }

  let isCreatingFolder = $state(false);
  let newFolderName = $state("");
  let newFolderInput = $state<HTMLInputElement | null>(null);

  $effect(() => {
    if (isCreatingFolder && newFolderInput) {
      newFolderInput.focus();
      newFolderInput.select();
    }
  });

  function triggerAddFolder(e: Event) {
    e.preventDefault();
    e.stopPropagation();
    isCreatingFolder = true;
    newFolderName = "New Folder";
    // Force folder to open
    const checkbox = document.getElementById(generateId(node.path || "root")) as HTMLInputElement;
    if (checkbox) checkbox.checked = true;
  }

  async function commitAddFolder() {
    if (!isCreatingFolder) return;
    isCreatingFolder = false;
    const name = newFolderName.trim();
    if (!name) return;

    try {
      let base = node.path || "/";
      if (!base.startsWith("/")) base = "/" + base;
      if (!base.endsWith("/")) base += "/";
      
      const endpoint = "/api/v1/sd_card/file" + base + name + "/";
      const response = await fetch(endpoint, { method: "POST", body: "" });
      
      if (!response.ok) throw new Error("Failed to create folder: " + response.statusText);
      
      alertStore.add(`Folder created successfully`, "success");
      if (onUploadSuccess) onUploadSuccess();
    } catch (err: any) {
      console.error(err);
      alertStore.add("Failed to create folder: " + err.message, "error");
    }
  }

  function handleNewFolderKeyDown(e: KeyboardEvent) {
    if (e.key === 'Enter') {
      commitAddFolder();
    } else if (e.key === 'Escape') {
      isCreatingFolder = false;
    }
  }

  

  function triggerUpload(folderPath: string, e: Event) {
    e.preventDefault();
    e.stopPropagation();
    let fp = folderPath || "/";
    if (!fp.startsWith("/")) fp = "/" + fp;
    uploadTargetFolder = fp;
    if (fileInputForUpload) {
      fileInputForUpload.click();
    }
  }

  async function handleFileSelected(e: Event) {
    const target = e.target as HTMLInputElement;
    const file = target.files?.[0];
    if (!file) return;

    isUploading = true;
    try {
      const folder = uploadTargetFolder.endsWith("/") ? uploadTargetFolder : uploadTargetFolder + "/";
      const fullPath = folder + file.name;
      const endpoint = "/api/v1/sd_card/file" + fullPath;
      
      const response = await fetch(endpoint, {
        method: "POST",
        body: file,
        headers: {
          "Content-Type": "application/octet-stream"
        }
      });
      
      if (!response.ok) throw new Error("Upload failed: " + response.statusText);
      
      alertStore.add("File uploaded successfully to " + fullPath, "success");
      if (onUploadSuccess) onUploadSuccess();
    } catch (err: any) {
      console.error(err);
      alertStore.add("Failed to upload file: " + err.message, "error");
    } finally {
      isUploading = false;
      target.value = ""; // reset input
    }
  }


  
  function getFileFullPath() {
    return node.path || ((parentPath.endsWith('/') ? parentPath : parentPath + '/') + (node.name || ''));
  }

  function generateId(path: string) {
    return 'folder-' + path.replace(/[^a-zA-Z0-9]/g, '-');
  }

  function isClickable(name: string | undefined) {
    if (!name) return false;
    if (allowedExtensions.length === 0) return true;
    return allowedExtensions.some(ext => name.endsWith(ext));
  }

  function handleFileClick(node: FileNode) {
    if (isClickable(node.name) && onFileSelect) {
      let fullPath = node.path;
      if (!fullPath && node.name) {
        const base = parentPath.endsWith('/') ? parentPath : parentPath + '/';
        fullPath = base + node.name;
      }
      onFileSelect(fullPath);
    }
  }

  // Safe name fallback
  let displayName = $derived(node.name || (node.path === '/' ? 'SD Root' : node.path.split('/').pop() || 'Folder'));
</script>

<input type="file" bind:this={fileInputForUpload} style="display: none;" onchange={handleFileSelected} />
{#if isRoot}
  <div class="tree-container">
    <ul class="tree-root-ul">
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <li class="tree-item">
        {#if node.children}
          {@const folderId = generateId(node.path || "root")}
          <input type="checkbox" id={folderId} class="tree-toggle" checked />
          <label for={folderId} class="tree-label" onmouseleave={cancelDelete}>
            <Icon name="folder-closed" class="icon folder-closed-icon" />
            <Icon name="folder-open" class="icon folder-open-icon" />
            <span class="folder-name-span">{displayName}</span>
            <div class="action-buttons">
              <button class="icon-action-btn" onclick={(e) => triggerAddFolder(e)} title="New Folder" aria-label="New Folder">
                <Icon name="folder-add" size={14} />
              </button>
              <button class="icon-action-btn" onclick={(e) => triggerUpload(node.path, e)} title="Upload File" aria-label="Upload File">
                <Icon name="upload" size={14} />
              </button>
            </div>
          </label>
          <div class="tree-children-wrapper">
            <ul class="tree-children">
              {#if isCreatingFolder}
                <li class="tree-item">
                  <div class="file-item is-creating">
                    <Icon name="folder-closed" class="icon folder-closed-icon inline-svg" />
                    <input 
                      type="text" 
                      class="new-folder-input" 
                      bind:value={newFolderName} 
                      bind:this={newFolderInput} 
                      onblur={commitAddFolder} 
                      onkeydown={handleNewFolderKeyDown} 
                    />
                  </div>
                </li>
              {/if}
              {#each node.children as child}
                <FileTree node={child} isRoot={false} {allowedExtensions} {onFileSelect} parentPath={node.path || parentPath} {onUploadSuccess} />
              {/each}
            </ul>
          </div>
        {:else}
          <!-- svelte-ignore a11y_click_events_have_key_events -->
          <!-- svelte-ignore a11y_no_static_element_interactions -->
          <div 
            class="file-item" 
            class:is-clickable={isClickable(node.name)}
            onclick={() => handleFileClick(node)}
            onmouseleave={cancelDelete}
          >
            <Icon name="file-doc" class="icon" />
            <span class="folder-name-span">{displayName}</span>
            <div class="action-buttons">
              {#if confirmDeletePath === getFileFullPath()}
                <button class="icon-action-btn confirm-yes-btn" onclick={executeDelete} title="Confirm Delete" aria-label="Confirm">
                  <Icon name="check" size={14} />
                </button>
                <button class="icon-action-btn" onclick={cancelDelete} title="Cancel" aria-label="Cancel">
                  <Icon name="x" size={14} />
                </button>
              {:else}
                <button class="icon-action-btn delete-btn" onclick={(e) => triggerDelete(getFileFullPath(), false, e)} title="Delete File" aria-label="Delete File">
                  <Icon name="trash" size={14} />
                </button>
              {/if}
            </div>
          </div>
        {/if}
      </li>
    </ul>
  </div>
{:else}
  <!-- Recursive call without the root container wrapper -->
  <li class="tree-item">
    {#if node.children}
      {@const folderId = generateId(node.path)}
      <input type="checkbox" id={folderId} class="tree-toggle" checked />
      <label for={folderId} class="tree-label" onmouseleave={cancelDelete}>
        <Icon name="folder-closed" class="icon folder-closed-icon" />
        <Icon name="folder-open" class="icon folder-open-icon" />
        <span class="folder-name-span">{displayName}</span>
        <div class="action-buttons">
          {#if confirmDeletePath === node.path}
            <button class="icon-action-btn confirm-yes-btn" onclick={executeDelete} title="Confirm Delete" aria-label="Confirm">
              <Icon name="check" size={14} />
            </button>
            <button class="icon-action-btn" onclick={cancelDelete} title="Cancel" aria-label="Cancel">
              <Icon name="x" size={14} />
            </button>
          {:else}
            <button class="icon-action-btn" onclick={(e) => triggerAddFolder(e)} title="New Folder" aria-label="New Folder">
              <Icon name="folder-add" size={14} />
            </button>
            <button class="icon-action-btn" onclick={(e) => triggerUpload(node.path, e)} title="Upload File" aria-label="Upload File">
              <Icon name="upload" size={14} />
            </button>
            {#if node.path !== '/'}
              <button class="icon-action-btn delete-btn" onclick={(e) => triggerDelete(node.path, true, e)} title="Delete Folder" aria-label="Delete Folder">
                <Icon name="trash" size={14} />
              </button>
            {/if}
          {/if}
        </div>
      </label>
      <div class="tree-children-wrapper">
        <ul class="tree-children">
          {#if isCreatingFolder}
            <li class="tree-item">
              <div class="file-item is-creating">
                <Icon name="folder-closed" class="icon folder-closed-icon inline-svg" />
                <input 
                  type="text" 
                  class="new-folder-input" 
                  bind:value={newFolderName} 
                  bind:this={newFolderInput} 
                  onblur={commitAddFolder} 
                  onkeydown={handleNewFolderKeyDown} 
                />
              </div>
            </li>
          {/if}
          {#each node.children as child}
            <FileTree node={child} isRoot={false} {allowedExtensions} {onFileSelect} parentPath={node.path || parentPath} {onUploadSuccess} />
          {/each}
        </ul>
      </div>
    {:else}
      <!-- svelte-ignore a11y_click_events_have_key_events -->
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div 
        class="file-item" 
        class:is-clickable={isClickable(node.name)}
        onclick={() => handleFileClick(node)}
        onmouseleave={cancelDelete}
      >
        <Icon name="file-doc" class="icon" />
        <span class="folder-name-span">{displayName}</span>
        <div class="action-buttons">
          {#if confirmDeletePath === getFileFullPath()}
            <button class="icon-action-btn confirm-yes-btn" onclick={executeDelete} title="Confirm Delete" aria-label="Confirm">
              <Icon name="check" size={14} />
            </button>
            <button class="icon-action-btn" onclick={cancelDelete} title="Cancel" aria-label="Cancel">
              <Icon name="x" size={14} />
            </button>
          {:else}
            <button class="icon-action-btn delete-btn" onclick={(e) => triggerDelete(getFileFullPath(), false, e)} title="Delete File" aria-label="Delete File">
              <Icon name="trash" size={14} />
            </button>
          {/if}
        </div>
      </div>
    {/if}
  </li>
{/if}
<style>
  /* Adapted from Uiverse.io snippet to match PicoCSS glassmorphism dark theme */
  .tree-container {
    width: 100%;
    border: 1px solid var(--pico-border-color);
    border-radius: var(--pico-border-radius);
    padding: 1rem;
    box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
  }

  .tree-root-ul,
  .tree-children {
    list-style: none;
    padding: 0;
    margin: 0;
  }

  .tree-children {
    margin-left: 12px;
    padding-left: 0;
  }

  .tree-item {
    position: relative;
    margin-top: 4px;
    padding-left: 14px;
  }

  .tree-children > :global(.tree-item)::before {
    content: "";
    position: absolute;
    left: 0;
    top: -4px;
    bottom: 0;
    width: 1px;
    background-color: var(--pico-muted-color);
    opacity: 0.3;
  }

  .tree-children > :global(.tree-item)::after {
    content: "";
    position: absolute;
    left: 0;
    top: 15px;
    width: 14px;
    height: 1px;
    background-color: var(--pico-muted-color);
    opacity: 0.3;
  }

  .tree-children > :global(.tree-item:last-child)::before {
    height: 20px;
    bottom: auto;
  }

  .tree-label,
  .file-item {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 4px 8px;
    border-radius: 4px;
    font-size: 0.9rem;
    color: var(--pico-color);
    transition: background-color 0.2s, color 0.2s;
    user-select: none;
    text-decoration: none;
    height: 32px;
  }

  .tree-label {
    cursor: pointer;
  }

  .file-item {
    opacity: 0.6; /* Dim non-clickable files by default */
  }

  .file-item.is-clickable {
    cursor: pointer;
    opacity: 1;
    color: #f97316; /* Highlight valid files in orange */
  }

  .tree-label:hover,
  .file-item.is-clickable:hover {
    background-color: rgb(from var(--pico-form-element-background-color) r g b / 0.3);
  }

  :global(.folder-open-icon) {
    display: none;
  }
  :global(.folder-closed-icon) {
    display: block;
  }

  .tree-toggle:checked ~ .tree-label :global(.folder-open-icon) {
    display: block;
    color: #f97316;
  }
  
  .tree-toggle:checked ~ .tree-label :global(.folder-closed-icon) {
    display: none;
  }

  .tree-toggle {
    display: none;
  }

  .tree-children-wrapper {
    display: grid;
    grid-template-rows: 0fr;
    transition: grid-template-rows 0.2s ease-in-out;
  }

  .tree-children {
    overflow: hidden;
    padding-top: 2px;
    padding-bottom: 2px;
  }

  /* State: Open */
  .tree-toggle:checked ~ .tree-children-wrapper {
    grid-template-rows: 1fr;
  }

  .tree-label {
    display: flex;
    justify-content: space-between; /* ensures button goes to the right if we want, or we can just use gap */
    width: 100%;
  }
  .folder-name-span {
    flex-grow: 1;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .action-buttons {
    display: flex;
    gap: 4px;
  }
  .icon-action-btn {
    background: transparent;
    border: none;
    padding: 2px 4px;
    margin: 0;
    color: var(--pico-muted-color);
    cursor: pointer;
    border-radius: 4px;
    display: flex;
    align-items: center;
    opacity: 0;
    transition: opacity 0.2s, background-color 0.2s, color 0.2s;
  }
  .tree-label:hover .icon-action-btn,
  .file-item:hover .icon-action-btn {
    opacity: 1;
  }
  .icon-action-btn:hover {
    background: rgb(from var(--pico-primary) r g b / 0.2) !important;
    color: var(--pico-primary) !important;
  }
  .icon-action-btn.delete-btn:hover {
    background: rgb(from var(--pico-del-color, #f95050) r g b / 0.2) !important;
    color: var(--pico-del-color, #f95050) !important;
  }
.new-folder-input {
    background: transparent;
    border: 1px solid var(--pico-primary);
    color: var(--pico-color);
    border-radius: 4px;
    padding: 2px 4px;
    font-size: 0.9rem;
    height: 24px;
    outline: none;
    width: 150px;
    margin-top: 2px;
    margin-bottom: 2px;
    box-shadow: 0 0 0 2px rgba(from var(--pico-primary) r g b / 0.2);
  }

  .is-creating {
    opacity: 1;
  }
.confirm-yes-btn {
    color: var(--pico-del-color, #f95050) !important;
  }
  .confirm-yes-btn:hover {
    background: rgb(from var(--pico-del-color, #f95050) r g b / 0.2) !important;
  }

  @media (max-width: 768px) {
    .icon-action-btn {
      opacity: 0.8;
      padding: 6px 8px;
    }
    .tree-label, .file-item {
      height: 40px;
    }
    .icon-action-btn:active {
      opacity: 1;
    }
  }
</style>
