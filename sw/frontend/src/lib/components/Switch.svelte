<script lang="ts">
  import type { HTMLLabelAttributes } from "svelte/elements";

  interface Props extends Omit<HTMLLabelAttributes, "onchange"> {
    checked?: boolean;
    disabled?: boolean;
    label?: string;
    subLabel?: string;
    onchange?: (checked: boolean) => void;
  }

  let {
    checked = $bindable(false),
    disabled = false,
    label,
    subLabel,
    onchange,
    "aria-label": ariaLabel,
    ...rest
  }: Props = $props();

  function handleToggle(e: Event) {
    if (disabled) return;
    checked = (e.currentTarget as HTMLInputElement).checked;
    onchange?.(checked);
  }
</script>

{#snippet toggle()}
  <label class="switch" class:disabled {...rest}>
    <input
      type="checkbox"
      {checked}
      {disabled}
      aria-label={ariaLabel ?? label}
      onchange={handleToggle}
    />
    <span class="slider"></span>
  </label>
{/snippet}

{#if label}
  <div class="toggle-container">
    <div class="toggle-text-group">
      <div class="toggle-label">{label}</div>
      {#if subLabel}
        <div class="toggle-sub-label">{subLabel}</div>
      {/if}
    </div>
    {@render toggle()}
  </div>
{:else}
  {@render toggle()}
{/if}

<style>
  .toggle-container {
    display: flex;
    align-items: center;
    justify-content: space-between;
    width: 100%;
    gap: 8px 16px;
    min-height: var(--ctl-h);
  }

  .toggle-text-group {
    display: flex;
    flex-direction: column;
    gap: 2px;
    min-width: 0;
  }

  .toggle-label {
    margin: 0;
    font-weight: 700;
  }

  .toggle-sub-label {
    margin: 0;
    color: var(--muted);
    font-size: 0.85rem;
    line-height: 1.35;
  }

  /* Mockup switch: 52x32 track, 24px knob, accent when on. */
  .switch {
    position: relative;
    display: inline-block;
    flex: none;
    width: 52px;
    height: 32px;
  }

  .switch.disabled {
    opacity: 0.55;
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
    inset: 0;
    box-sizing: border-box;
    cursor: pointer;
    border: 1px solid var(--ctl-bd);
    border-radius: 99px;
    background: var(--track);
    transition: background-color 0.2s;
  }

  .switch.disabled .slider {
    cursor: not-allowed;
  }

  .slider::after {
    content: "";
    position: absolute;
    top: 3px;
    left: 3px;
    width: 24px;
    height: 24px;
    border-radius: 50%;
    background: var(--float-solid);
    transition:
      transform 0.2s var(--ease-out, cubic-bezier(0.16, 1, 0.3, 1)),
      background-color 0.2s;
  }

  input:checked + .slider {
    background: var(--accent);
    border-color: var(--accent);
  }

  input:checked + .slider::after {
    transform: translateX(20px);
    background: var(--on-accent);
  }

  input:focus-visible + .slider {
    outline: 2px solid var(--accent);
    outline-offset: 2px;
  }

  @media (prefers-reduced-motion: reduce) {
    .slider,
    .slider::after {
      transition-duration: 0.01ms;
    }
  }
</style>
