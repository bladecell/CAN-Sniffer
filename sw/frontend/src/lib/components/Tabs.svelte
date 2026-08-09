<script lang="ts">
  interface Option {
    value: string;
    label: string;
  }

  interface Props {
    options: Option[];
    selected: string;
  }

  let { options, selected = $bindable() }: Props = $props();
</script>

<div class="tabs-wrapper">
  <fieldset class="tabs-container">
    {#each options as option}
      <label class="tab" class:active={selected === option.value}>
        <input
          type="radio"
          name="tab-radio"
          value={option.value}
          bind:group={selected}
        />
        {option.label}
      </label>
    {/each}
  </fieldset>
</div>

<style>
  .tabs-wrapper {
    display: flex;
    justify-content: flex-start;
    margin-bottom: 2rem;
  }

  .tabs-container {
    display: inline-flex;
    margin: 0;
    padding: 4px;
    background: var(--pico-muted-border-color);
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.05);
    width: fit-content;
  }

  .tab {
    position: relative;
    margin: 0;
    padding: 8px 24px;
    cursor: pointer;
    font-size: 0.9rem;
    font-weight: 500;
    color: var(--pico-muted-color);
    border-radius: 8px;
    transition: all 0.2s ease;
    user-select: none;
  }

  .tab input[type="radio"] {
    position: absolute;
    opacity: 0;
    width: 0;
    height: 0;
    margin: 0;
  }

  .tab:not(.active):hover {
    color: var(--pico-primary);
  }

  .tab.active {
    color: var(--pico-color);
    background: var(--pico-background-color);
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  }
</style>
