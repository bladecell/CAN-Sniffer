<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import "./listDetail.css";

  interface Props {
    tone: "ok" | "warn" | "crit" | "muted";
    icon?: string;
    text: string;
    rest?: string;
  }

  let { tone, icon, text, rest = "" }: Props = $props();
</script>

<p class="rv-status rv-t-{tone}">
  {#if icon}<Icon name={icon} size={20} />{/if}
  <span>{text}</span>
  {#if rest}<span class="rv-rest">{rest}</span>{/if}
</p>
