<script lang="ts">
  import type { HTMLAttributes } from "svelte/elements";
  import Icon from "$lib/Icon.svelte";
  import Switch from "./Switch.svelte";
  import type { ControlsGridItem, ControlType } from "$lib/types";

  interface Props extends HTMLAttributes<HTMLElement> {
    item?: ControlsGridItem;
  }

  let { item, ...rest }: Props = $props();

  let color = "red";
  let icon = "dial";
  let label = "Label";
  let description = "description";
</script>

<article
  class="dashboard-card pid-card"
  style="--module-accent: {color};"
  {...rest}
>
  <header class="dashboard-card-header">
    <div class="dashboard-card-icon">
      <Icon name={icon} size={32} />
    </div>
    <div class="dashboard-card-titles">
      <div class="dashboard-card-label">{label}</div>
      <div class="dashboard-card-subtitle">
        {description}
      </div>
    </div>
  </header>

  <Switch aria-label={label} />
</article>

<style>
  .pid-card {
    justify-content: space-between;
  }
</style>
