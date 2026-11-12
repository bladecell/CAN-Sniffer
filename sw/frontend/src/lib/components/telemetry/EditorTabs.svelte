<script lang="ts">
  import { EDITOR_SECTIONS, editorNav } from "./editorNav.svelte";
</script>

<nav class="tabs" aria-label="Editor sections">
  {#each EDITOR_SECTIONS as s, i (s.id)}
    <button type="button" class="tab" aria-current={editorNav.active === i ? "true" : undefined} onclick={() => editorNav.goTo(i)}>
      {s.label}
    </button>
  {/each}
</nav>

<style>
  /* Section tabs: the same segmented switch as the PID panels (PidTabs bar): a tray, a raised selected tab, an accent underline. */
  .tabs {
    margin: 0;
    box-sizing: border-box;
    display: flex;
    gap: 4px;
    padding: 4px;
    border-radius: 18px;
    background: var(--ctl);
  }
  .tab {
    position: relative;
    flex: 1 1 0;
    min-width: 0;
    min-height: 44px;
    margin: 0;
    padding: 0 10px;
    border: 0;
    border-radius: 15px;
    background: none;
    color: var(--ctl-sub);
    font: 700 0.9rem/1.2 var(--font);
    white-space: nowrap;
    cursor: pointer;
  }
  @media (hover: hover) {
    .tab:hover {
      background: var(--hov);
    }
  }
  .tab[aria-current='true'] {
    background: var(--float-solid);
    color: var(--text);
    font-weight: 800;
  }
  .tab[aria-current='true']::after {
    content: '';
    position: absolute;
    left: 24%;
    right: 24%;
    bottom: 3px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  /* No ring on mouse or keyboard focus (Pico draws one with box-shadow). Keyboard focus is a background step. */
  .tab:focus,
  .tab:focus-visible {
    outline: none !important;
    box-shadow: none !important;
  }
  .tab:focus-visible {
    background: var(--hov);
  }
  .tab[aria-current='true']:focus-visible {
    background: var(--float-solid);
  }
</style>
