<script lang="ts">
  import type { Snippet } from "svelte";
  import "./listDetail.css";

  interface Props {
    title: string;
    status?: Snippet;
    meta?: Snippet;
    actions?: Snippet;
    /** Sits right of the title, for a help button. */
    info?: Snippet;
  }

  let { title, status, meta, actions, info }: Props = $props();
</script>

<header class="rv-head" class:rv-has-actions={!!actions}>
  <div class="rv-head-main">
    {#if info}
      <div class="rv-title-row"><h1>{title}</h1>{@render info()}</div>
    {:else}
      <h1>{title}</h1>
    {/if}
    {@render status?.()}
    {@render meta?.()}
  </div>
  {#if actions}
    <div class="rv-actions">{@render actions()}</div>
  {/if}
</header>

<style>
  .rv-title-row {
    display: flex;
    align-items: center;
    gap: 10px;
    min-width: 0;
  }
  .rv-title-row :global(h1) {
    margin: 0;
    line-height: 1.1;
  }
  /* Trim the title's line box to the capitals, so centring the icon centres it on the cap height at any title size.
     The padding restores the 1.1 line height, so the header keeps its height. Browsers without text-box-trim keep the plain line box, where the icon sits a few pixels low. */
  @supports (text-box: trim-both cap alphabetic) {
    .rv-title-row :global(h1) {
      text-box: trim-both cap alphabetic;
      padding-block: 0.19em;
    }
  }
</style>
