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
  /* Section tabs: a segmented control inside the sheet header's float layer. The selected tab is an accent fill. */
  .tabs {
    margin: 0;
    box-sizing: border-box;
    display: flex;
    gap: 4px;
    height: auto;
    padding: 4px;
    border-radius: 18px;
    /* A subtle tone under the sheet, darker than the sheet in both themes (mockup). */
    background: #ece2d4;
  }
  @media (prefers-color-scheme: dark) {
    .tabs {
      background: #34291f;
    }
  }
  :global(html[data-theme='dark']) .tabs {
    background: #34291f;
  }
  .tab {
    flex: 1 1 0;
    min-width: 0;
    min-height: 44px;
    margin: 0;
    padding: 0 10px;
    border: 0;
    border-radius: 14px;
    background: transparent;
    color: var(--ctl-sub);
    font: inherit;
    font-weight: 650;
    white-space: nowrap;
    cursor: pointer;
  }
  @media (hover: hover) {
    .tab:hover {
      background: var(--hov);
    }
  }
  .tab[aria-current='true'] {
    background: var(--accent);
    color: var(--on-accent);
    font-weight: 650;
    box-shadow: none;
    outline: none;
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
    background: var(--accent);
    filter: brightness(0.92);
  }
</style>
