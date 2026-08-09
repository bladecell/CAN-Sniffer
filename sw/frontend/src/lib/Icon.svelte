<script lang="ts">
  import { iconData, type IconName } from "$lib/icons";

  let { name, size = 24, class: className = "" }: { name: IconName; size?: number; class?: string } = $props();

  const icon = $derived(iconData[name] || { viewBox: "0 0 512 512", path: "" });
</script>

<svg
  viewBox={icon.viewBox}
  class={className}
  width={size}
  height={size}
  aria-hidden="true"
  role="img"
>
  {#if icon.body}
    {@html icon.body}
  {:else if icon.path}
    <path d={icon.path} fill="currentColor" />
  {/if}
</svg>
