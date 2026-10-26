<script lang="ts">
  import type { Snippet } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import "./listDetail.css";

  interface Props {
    title: string;
    icon?: string;
    tone?: "ok" | "warn" | "crit" | "muted";
    role?: "status" | "alert";
    spinning?: boolean;
    children?: Snippet;
  }

  let { title, icon, tone = "muted", role, spinning = false, children }: Props = $props();
</script>

<div class="rv-note" {role}>
  {#if spinning}
    <span class="rv-spin" aria-hidden="true"></span>
  {:else if icon}
    <Icon name={icon} size={28} class="rv-t-{tone}" />
  {/if}
  <div class="rv-note-body">
    <h2>{title}</h2>
    {#if children}{@render children()}{/if}
  </div>
</div>
