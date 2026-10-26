<script lang="ts">
  import type { Snippet } from "svelte";
  import "./listDetail.css";

  interface Props {
    title: string;
    status?: Snippet;
    meta?: Snippet;
    actions?: Snippet;
  }

  let { title, status, meta, actions }: Props = $props();
</script>

<header class="rv-head">
  <div class="rv-head-main">
    <h1>{title}</h1>
    {@render status?.()}
    {@render meta?.()}
  </div>
  {#if actions}
    <div class="rv-actions">{@render actions()}</div>
  {/if}
</header>
