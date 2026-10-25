<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  
  interface Props {
    maxRows?: number;
    maxCols?: number;
    onSelect: (rows: number, cols: number) => void;
  }
  
  let { maxRows = 3, maxCols = 4, onSelect }: Props = $props();
  
  let open = $state(false);
  let hoverR = $state(0);
  let hoverC = $state(0);
  
  function handleSelect(r: number, c: number) {
    onSelect(r, c);
    open = false;
  }
</script>

<svelte:window onkeydown={(e) => e.key === "Escape" && open && (open = false)} />

<div class="grid-selector-container">
  <button class="btn btn-outline" onclick={() => (open = !open)}>
    <Icon name="grid" size={16} />
    Layout
  </button>
  
  {#if open}
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <!-- svelte-ignore a11y_no_static_element_interactions -->
    <div class="dropdown-overlay" role="presentation" onclick={() => (open = false)}></div>
    <div class="popup-menu blur-background">
      <div class="grid-label">
        {hoverR > 0 && hoverC > 0 ? `${hoverR} x ${hoverC}` : "Select Layout"}
      </div>
      <div class="squares">
        {#each Array(maxRows) as _, r}
          <div class="row">
            {#each Array(maxCols) as _, c}
              <button 
                class="square" 
                class:active={r < hoverR && c < hoverC}
                onmouseover={() => { hoverR = r + 1; hoverC = c + 1; }}
                onfocus={() => { hoverR = r + 1; hoverC = c + 1; }}
                onclick={() => handleSelect(r + 1, c + 1)}
                aria-label="{r+1}x{c+1} Grid"
              ></button>
            {/each}
          </div>
        {/each}
      </div>
    </div>
  {/if}
</div>

<style>
  .grid-selector-container {
    position: relative;
    display: inline-block;
  }
  
  .btn-outline {
    display: inline-flex;
    align-items: center;
    gap: 0.4rem;
    background: transparent;
    border: 1px solid var(--overlay-2);
    color: var(--pico-color);
    padding: 0.35rem 0.75rem;
    border-radius: 6px;
    font-size: 0.75rem;
    font-weight: 500;
    cursor: pointer;
    transition: background 0.2s var(--ease-out), color 0.2s var(--ease-out), border-color 0.2s var(--ease-out), opacity 0.2s var(--ease-out), transform 0.2s var(--ease-out);
  }
  
  .btn-outline:hover {
    background: var(--overlay-1);
    border-color: rgba(255, 255, 255, 0.2);
  }
  

  
  .popup-menu {
    position: absolute;
    top: 100%;
    right: 0;
    margin-top: 8px;
    padding: 12px;
    z-index: 100;
  }
  
  .grid-label {
    text-align: center;
    font-size: 0.85rem;
    font-weight: 600;
    margin-bottom: 8px;
    color: var(--pico-muted-color);
  }
  
  .squares {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }
  
  .row {
    display: flex;
    gap: 4px;
  }
  
  .square {
    width: 24px;
    height: 24px;
    border: 1px solid var(--overlay-2);
    background: transparent;
    border-radius: 4px;
    cursor: pointer;
    padding: 0;
    transition: background 0.1s, border-color 0.1s;
  }
  
  @media (pointer: coarse) {
    .square {
      width: 36px;
      height: 36px;
    }
  }

  .square.active {
    background: var(--pico-primary);
    border-color: var(--pico-primary);
  }
</style>
