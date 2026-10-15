<script lang="ts">
  import type { HTMLLabelAttributes } from "svelte/elements";

  interface Props extends Omit<HTMLLabelAttributes, "onchange"> {
    checked?: boolean;
    disabled?: boolean;
    onchange?: (e: Event) => void;
  }

  let {
    checked = $bindable(),
    disabled = false,
    onchange,
    "aria-label": ariaLabel,
    ...rest
  }: Props = $props();

  function handleToggle(e: Event) {
    if (disabled) return;
    if (onchange) onchange(e);
  }
</script>

<label class="switch" class:disabled {...rest}>
  <input
    type="checkbox"
    bind:checked
    {disabled}
    aria-label={ariaLabel}
    onchange={handleToggle}
  />
  <span class="slider"></span>
</label>

<style>
  /* --- CUSTOM TOGGLE SWITCH --- */
  .switch {
    /* Colors */
    --secondary-container: #3a4b39;
    --primary: #84da89;

    /* Master Dimensions (Change these to scale the whole switch) */
    --toggle-width: 3.25em;
    --toggle-height: 1.75em;
    --toggle-padding: 0.1875em;

    /* Automatic Math (Do not touch) */
    --knob-size: calc(var(--toggle-height) - (var(--toggle-padding) * 2));
    --travel-distance: calc(
      var(--toggle-width) - var(--knob-size) - (var(--toggle-padding) * 2)
    );

    position: relative;
    display: inline-block;
    width: var(--toggle-width);
    height: var(--toggle-height);
    flex-shrink: 0;
  }

  .switch.disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  /* Hidden but still focusable: the visible ring is drawn on .slider below. */
  .switch input {
    position: absolute;
    width: 1px;
    height: 1px;
    margin: 0;
    opacity: 0;
    pointer-events: none;
  }

  .slider {
    position: absolute;
    cursor: pointer;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background-color: #313033;
    transition: 0.3s ease;
    border-radius: 34px;
    box-sizing: border-box;
  }

  .switch.disabled .slider {
    cursor: not-allowed;
  }

  .slider:before {
    position: absolute;
    content: "";
    height: var(--knob-size);
    width: var(--knob-size);
    left: var(--toggle-padding);
    bottom: var(--toggle-padding);
    background-color: #aeaaae;
    transition: 0.3s ease;
    border-radius: 50%;
    box-sizing: border-box;
  }

  input:checked + .slider {
    background-color: var(--secondary-container);
  }

  input:checked + .slider:before {
    background-color: var(--primary);
    transform: translateX(var(--travel-distance));
  }

  input:focus-visible + .slider {
    box-shadow: 0 0 0 2px var(--primary);
  }
</style>
