<script lang="ts">
  import FileTree from "./FileTree.svelte";
  import { alertStore } from "$lib/alertStore.svelte";

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

{#if isRoot}
  <div class="tree-container">
    <input type="file" bind:this={fileInputForUpload} style="display: none;" onchange={handleFileSelected} />
    <ul class="tree-root-ul">
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <li class="tree-item">
        {#if node.children}
          {@const folderId = generateId(node.path || "root")}
          <input type="checkbox" id={folderId} class="tree-toggle" checked />
          <label for={folderId} class="tree-label" onmouseleave={cancelDelete}>
            <svg class="icon folder-closed-icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
              <path d="M4 20h16a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.93a2 2 0 0 1-1.66-.9l-.82-1.2A2 2 0 0 0 7.93 2H4a2 2 0 0 0-2 2v13c0 1.1.9 2 2 2Z"></path>
            </svg>
            <svg class="icon folder-open-icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
              <path d="M4 20h16a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.93a2 2 0 0 1-1.66-.9l-.82-1.2A2 2 0 0 0 7.93 2H4a2 2 0 0 0-2 2v13c0 1.1.9 2 2 2Z"></path>
              <path d="M2 10h20"></path>
            </svg>
            <span class="folder-name-span">{displayName}</span>
            <div class="action-buttons">
              <button class="icon-action-btn" onclick={(e) => triggerAddFolder(e)} title="New Folder" aria-label="New Folder">
                <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M20 20a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.9a2 2 0 0 1-1.69-.9L9.6 3.9A2 2 0 0 0 7.93 3H4a2 2 0 0 0-2 2v13a2 2 0 0 0 2 2Z"/><line x1="12" y1="10" x2="12" y2="16"/><line x1="9" y1="13" x2="15" y2="13"/></svg>
              </button>
              <button class="icon-action-btn" onclick={(e) => triggerUpload(node.path, e)} title="Upload File" aria-label="Upload File">
                <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg>
              </button>
            </div>
          </label>
          <div class="tree-children-wrapper">
            <ul class="tree-children">
              {#if isCreatingFolder}
                <li class="tree-item">
                  <div class="file-item is-creating">
                    <svg class="icon folder-closed-icon inline-svg" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                      <path d="M4 20h16a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.93a2 2 0 0 1-1.66-.9l-.82-1.2A2 2 0 0 0 7.93 2H4a2 2 0 0 0-2 2v13c0 1.1.9 2 2 2Z"></path>
                    </svg>
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
            <svg class="icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
              <path d="M14.5 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V7.5L14.5 2z"></path>
              <polyline points="14 2 14 8 20 8"></polyline>
            </svg>
            <span class="folder-name-span">{displayName}</span>
            <div class="action-buttons">
              {#if confirmDeletePath === getFileFullPath()}
                <button class="icon-action-btn confirm-yes-btn" onclick={executeDelete} title="Confirm Delete" aria-label="Confirm">
                  <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="20 6 9 17 4 12"></polyline></svg>
                </button>
                <button class="icon-action-btn" onclick={cancelDelete} title="Cancel" aria-label="Cancel">
                  <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg>
                </button>
              {:else}
                <button class="icon-action-btn delete-btn" onclick={(e) => triggerDelete(getFileFullPath(), false, e)} title="Delete File" aria-label="Delete File">
                  <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 6h18"/><path d="M19 6v14c0 1-1 2-2 2H7c-1 0-2-1-2-2V6"/><path d="M8 6V4c0-1 1-2 2-2h4c1 0 2 1 2 2v2"/><line x1="10" y1="11" x2="10" y2="17"/><line x1="14" y1="11" x2="14" y2="17"/></svg>
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
        <svg class="icon folder-closed-icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M4 20h16a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.93a2 2 0 0 1-1.66-.9l-.82-1.2A2 2 0 0 0 7.93 2H4a2 2 0 0 0-2 2v13c0 1.1.9 2 2 2Z"></path>
        </svg>
        <svg class="icon folder-open-icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M4 20h16a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.93a2 2 0 0 1-1.66-.9l-.82-1.2A2 2 0 0 0 7.93 2H4a2 2 0 0 0-2 2v13c0 1.1.9 2 2 2Z"></path>
          <path d="M2 10h20"></path>
        </svg>
        <span class="folder-name-span">{displayName}</span>
        <div class="action-buttons">
          {#if confirmDeletePath === node.path}
            <button class="icon-action-btn confirm-yes-btn" onclick={executeDelete} title="Confirm Delete" aria-label="Confirm">
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="20 6 9 17 4 12"></polyline></svg>
            </button>
            <button class="icon-action-btn" onclick={cancelDelete} title="Cancel" aria-label="Cancel">
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg>
            </button>
          {:else}
            <button class="icon-action-btn" onclick={(e) => triggerAddFolder(e)} title="New Folder" aria-label="New Folder">
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M20 20a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.9a2 2 0 0 1-1.69-.9L9.6 3.9A2 2 0 0 0 7.93 3H4a2 2 0 0 0-2 2v13a2 2 0 0 0 2 2Z"/><line x1="12" y1="10" x2="12" y2="16"/><line x1="9" y1="13" x2="15" y2="13"/></svg>
            </button>
            <button class="icon-action-btn" onclick={(e) => triggerUpload(node.path, e)} title="Upload File" aria-label="Upload File">
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg>
            </button>
            {#if node.path !== '/'}
              <button class="icon-action-btn delete-btn" onclick={(e) => triggerDelete(node.path, true, e)} title="Delete Folder" aria-label="Delete Folder">
                <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 6h18"/><path d="M19 6v14c0 1-1 2-2 2H7c-1 0-2-1-2-2V6"/><path d="M8 6V4c0-1 1-2 2-2h4c1 0 2 1 2 2v2"/><line x1="10" y1="11" x2="10" y2="17"/><line x1="14" y1="11" x2="14" y2="17"/></svg>
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
                <svg class="icon folder-closed-icon inline-svg" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                  <path d="M4 20h16a2 2 0 0 0 2-2V8a2 2 0 0 0-2-2h-7.93a2 2 0 0 1-1.66-.9l-.82-1.2A2 2 0 0 0 7.93 2H4a2 2 0 0 0-2 2v13c0 1.1.9 2 2 2Z"></path>
                </svg>
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
        <svg class="icon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M14.5 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V7.5L14.5 2z"></path>
          <polyline points="14 2 14 8 20 8"></polyline>
        </svg>
        <span class="folder-name-span">{displayName}</span>
        <div class="action-buttons">
          {#if confirmDeletePath === getFileFullPath()}
            <button class="icon-action-btn confirm-yes-btn" onclick={executeDelete} title="Confirm Delete" aria-label="Confirm">
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="20 6 9 17 4 12"></polyline></svg>
            </button>
            <button class="icon-action-btn" onclick={cancelDelete} title="Cancel" aria-label="Cancel">
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg>
            </button>
          {:else}
            <button class="icon-action-btn delete-btn" onclick={(e) => triggerDelete(getFileFullPath(), false, e)} title="Delete File" aria-label="Delete File">
              <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 6h18"/><path d="M19 6v14c0 1-1 2-2 2H7c-1 0-2-1-2-2V6"/><path d="M8 6V4c0-1 1-2 2-2h4c1 0 2 1 2 2v2"/><line x1="10" y1="11" x2="10" y2="17"/><line x1="14" y1="11" x2="14" y2="17"/></svg>
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

  .icon {
    width: 16px;
    height: 16px;
    color: var(--pico-muted-color);
    flex-shrink: 0;
  }
  
  .file-item.is-clickable .icon {
    color: #f97316;
  }

  .folder-open-icon {
    display: none;
  }
  .folder-closed-icon {
    display: block;
  }

  .tree-toggle:checked ~ .tree-label .folder-open-icon {
    display: block;
    color: #f97316;
  }
  
  .tree-toggle:checked ~ .tree-label .folder-closed-icon {
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
  .inline-svg {
    display: block !important;
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
