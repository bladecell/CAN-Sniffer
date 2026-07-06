<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import type { ControlsGridItem, ControlType } from "$lib/types";

  interface Props {
    item: ControlsGridItem;
    [key: string]: any;
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

  <label class="switch">
    <input type="checkbox" />
    <span class="slider"></span>
  </label>
</article>

<style>
  .pid-card {
    justify-content: space-between;
  }

  .switch {
    --secondary-container: #3a4b39;
    --primary: #84da89;
    font-size: 17px;
    position: relative;
    display: inline-block;
    width: 3.7em;
    height: 1.8em;
  }

  .switch input {
    display: none;
    opacity: 0;
    width: 0;
    height: 0;
  }

  .slider {
    position: absolute;
    cursor: pointer;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background-color: #313033;
    transition: 0.2s;
    border-radius: 30px;
  }

  .slider:before {
    position: absolute;
    content: "";
    height: 1.4em;
    width: 1.4em;
    border-radius: 20px;
    left: 0.2em;
    bottom: 0.2em;
    background-color: #aeaaae;
    transition: 0.4s;
  }

  input:checked + .slider::before {
    background-color: var(--primary);
  }

  input:checked + .slider {
    background-color: var(--secondary-container);
  }

  input:focus + .slider {
    box-shadow: 0 0 1px var(--secondary-container);
  }

  input:checked + .slider:before {
    transform: translateX(1.9em);
  }
</style>
