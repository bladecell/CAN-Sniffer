<script lang="ts">
  import { EDITOR_SECTIONS, editorNav } from "./editorNav.svelte";
</script>

<nav class="seg eq" aria-label="Editor sections">
  {#each EDITOR_SECTIONS as s, i (s.id)}
    <button type="button" aria-current={editorNav.active === i ? "true" : undefined} onclick={() => editorNav.goTo(i)}>
      {s.label}
    </button>
  {/each}
</nav>
