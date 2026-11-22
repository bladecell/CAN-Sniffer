<script lang="ts">
  import { PIN_MAX, pinFromText } from "$lib/pinPad";

  interface Props {
    id: string;
    label: string;
    /** The PIN typed so far. */
    value: string;
    onchange: (next: string) => void;
    /** The pad and the next box aim at this field. */
    active?: boolean;
    reveal?: boolean;
    disabled?: boolean;
    invalid?: boolean;
    describedby?: string;
    autocomplete: "current-password" | "new-password";
    autofocus?: boolean;
    onfocus?: () => void;
    max?: number;
  }

  let {
    id,
    label,
    value,
    onchange,
    active = true,
    reveal = false,
    disabled = false,
    invalid = false,
    describedby,
    autocomplete,
    autofocus = false,
    onfocus,
    max = PIN_MAX,
  }: Props = $props();

  const labelId = $derived(`${id}-label`);
  const boxes = $derived(Array.from({ length: max }, (_, i) => i));
  let input: HTMLInputElement | undefined = $state();

  // Desktop only: on a phone, focusing the field would pull up the OS keyboard over the pad.
  $effect(() => {
    if (autofocus && input && matchMedia("(pointer: fine)").matches) input.focus({ preventScroll: true });
  });

  // The hidden input is the real field: physical keys, paste and password managers all write to it.
  function onInput(e: Event & { currentTarget: HTMLInputElement }) {
    const next = pinFromText(e.currentTarget.value, max);
    e.currentTarget.value = next;
    onchange(next);
  }
</script>

<div class="pin-field">
  <span id={labelId} class="lbl">{label}</span>
  <input
    bind:this={input}
    {id}
    class="sr-only"
    type="password"
    inputmode="numeric"
    {autocomplete}
    maxlength={max}
    pattern={`[0-9]{${max}}`}
    {value}
    {disabled}
    required
    aria-labelledby={labelId}
    aria-invalid={invalid ? "true" : undefined}
    aria-describedby={describedby}
    oninput={onInput}
    onfocus={() => onfocus?.()}
  />
  <!-- Visual only: the boxes mirror the hidden input. A tap on them focuses it. -->
  <div class="boxes" class:invalid aria-hidden="true" onclick={() => input?.focus()}>
    {#each boxes as i (i)}
      <span class="box" class:next={active && i === value.length && !disabled}>
        {#if i < value.length}{reveal ? value[i] : "•"}{/if}
      </span>
    {/each}
  </div>
</div>

<style>
  .pin-field {
    position: relative;
  }
  .lbl {
    display: block;
    margin: 0 0 6px;
    font-size: 0.875rem;
    font-weight: 600;
    color: var(--muted);
  }
  .boxes {
    display: grid;
    grid-template-columns: repeat(6, minmax(0, 1fr));
    gap: 8px;
    cursor: text;
  }
  .box {
    display: grid;
    place-items: center;
    aspect-ratio: 5 / 6;
    border-radius: var(--r-sm);
    background: var(--row);
    color: var(--text);
    font: 700 1.5rem/1 var(--mono);
    box-shadow: inset 0 0 0 2px transparent;
  }
  .box.next {
    box-shadow: inset 0 0 0 2px var(--accent);
  }
  .boxes.invalid .box {
    box-shadow: inset 0 0 0 2px var(--crit);
  }
  .boxes.invalid .box.next {
    box-shadow: inset 0 0 0 2px var(--crit);
  }
  .pin-field:has(input:disabled) .boxes {
    opacity: 0.55;
    cursor: default;
  }
</style>
