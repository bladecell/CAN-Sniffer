<script lang="ts">
  import type { HTMLInputAttributes } from "svelte/elements";

  interface Props extends Omit<HTMLInputAttributes, "onchange" | "type" | "checked"> {
    checked?: boolean;
    onchange?: (checked: boolean) => void;
  }

  let { checked = false, onchange, ...rest }: Props = $props();
</script>

<input type="checkbox" class="cb" {checked} onchange={(e) => onchange?.(e.currentTarget.checked)} {...rest} />

<style>
  /* A choice submitted later (Apply bar). Immediate settings use Switch. */
  .cb {
    appearance: none;
    flex: none;
    box-sizing: border-box;
    display: inline-grid;
    place-content: center;
    width: 26px;
    height: 26px;
    margin: 0;
    border: 2px solid var(--track);
    border-radius: 8px;
    background: transparent;
    cursor: pointer;
    transition: border-color 0.2s;
  }
  .cb::after {
    content: "";
    width: 12px;
    height: 12px;
    border-radius: 4px;
    background: var(--accent);
    transform: scale(0);
    transition: transform 0.15s var(--ease-out);
  }
  .cb:checked {
    border-color: var(--accent);
  }
  .cb:checked::after {
    transform: scale(1);
  }
  .cb:disabled {
    opacity: 0.55;
    cursor: not-allowed;
  }
  .cb:focus-visible {
    outline: 2px solid var(--accent);
    outline-offset: 2px;
  }
  @media (pointer: coarse) {
    .cb {
      width: 30px;
      height: 30px;
    }
  }
  @media (prefers-reduced-motion: reduce) {
    .cb,
    .cb::after {
      transition-duration: 0.01ms;
    }
  }
</style>
