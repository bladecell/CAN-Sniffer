<script lang="ts">
  import { PIN_MAX, pressBackspace, pressDigit } from "$lib/pinPad";

  interface Props {
    /** The PIN of the field the pad is aimed at. */
    value: string;
    onchange: (next: string) => void;
    max?: number;
    disabled?: boolean;
  }

  let { value, onchange, max = PIN_MAX, disabled = false }: Props = $props();

  const digits = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"];

  // A tap must not move focus off the PIN field: the field keeps physical keyboard input, and no OS keyboard pops up.
  function keepFocus(e: MouseEvent) {
    e.preventDefault();
  }
</script>

<div class="pad" role="group" aria-label="PIN pad">
  {#each digits as d, i (d)}
    {#if i === 9}
      <span class="gap" aria-hidden="true"></span>
    {/if}
    <button type="button" class="key" aria-label={`Digit ${d}`} {disabled} onmousedown={keepFocus} onclick={() => onchange(pressDigit(value, d, max))}>{d}</button>
  {/each}
  <button type="button" class="key fn" aria-label="Delete last digit" {disabled} onmousedown={keepFocus} onclick={() => onchange(pressBackspace(value))}>
    <span aria-hidden="true">⌫</span>
  </button>
</div>

<style>
  .pad {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 10px;
  }
  .key {
    display: grid;
    place-items: center;
    aspect-ratio: 3 / 2;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: 700 1.5rem/1 var(--mono);
    font-variant-numeric: tabular-nums;
    cursor: pointer;
    user-select: none;
    touch-action: manipulation;
    transition: background-color 0.15s, transform 0.1s;
  }
  @media (hover: hover) {
    .key:hover:not(:disabled) {
      background: var(--ctl-h2);
    }
  }
  .key:active:not(:disabled) {
    transform: scale(0.95);
  }
  .key:disabled {
    opacity: 0.55;
    cursor: default;
  }
  .key.fn {
    font: 600 1.25rem/1 var(--font);
    color: var(--ctl-sub);
  }
  .gap {
    display: block;
  }
</style>
