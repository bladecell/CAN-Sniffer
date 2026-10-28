<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { stepValue } from "$lib/editor/stepper";

  /** Number field with minus and plus buttons. Arrow keys step, PageUp and PageDown step by ten, holding a button repeats. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: number | undefined;
    step?: number;
    min?: number;
    max?: number;
    invalid?: boolean;
    disabled?: boolean;
    placeholder?: string;
  }

  let {
    id,
    labelId,
    describedBy,
    value = $bindable(),
    step = 1,
    min,
    max,
    invalid = false,
    disabled = false,
    placeholder,
  }: Props = $props();

  const fmt = (v: number | undefined) => (v === undefined ? "" : String(v));
  let text = $state(fmt(value));

  // Keep the text in step with the bound value when it changes outside this field.
  $effect(() => {
    const t = text.trim();
    if (t !== "" && !Number.isFinite(Number(t))) return; // keep half-typed text
    if ((t === "" ? undefined : Number(t)) !== value) text = fmt(value);
  });

  function onInput(e: Event) {
    text = (e.currentTarget as HTMLInputElement).value;
    const t = text.trim();
    if (t === "") value = undefined;
    else if (Number.isFinite(Number(t))) value = Number(t);
  }

  function nudge(dir: 1 | -1, multiplier = 1) {
    if (disabled) return;
    value = stepValue(value, dir, { step, min, max, multiplier });
    text = fmt(value);
  }

  function onKey(e: KeyboardEvent) {
    if (e.key === "ArrowUp" || e.key === "ArrowDown") {
      e.preventDefault();
      nudge(e.key === "ArrowUp" ? 1 : -1);
    } else if (e.key === "PageUp" || e.key === "PageDown") {
      e.preventDefault();
      nudge(e.key === "PageUp" ? 1 : -1, 10);
    }
  }

  // Hold to repeat: one step on press, then steps every 90 ms after a 400 ms pause.
  let holdTimer: ReturnType<typeof setTimeout> | undefined;
  let repeatTimer: ReturnType<typeof setInterval> | undefined;
  function stopHold() {
    clearTimeout(holdTimer);
    clearInterval(repeatTimer);
    holdTimer = undefined;
    repeatTimer = undefined;
  }
  function startHold(dir: 1 | -1, e: PointerEvent) {
    if (e.button !== 0 || disabled) return;
    nudge(dir);
    holdTimer = setTimeout(() => {
      repeatTimer = setInterval(() => nudge(dir), 90);
    }, 400);
  }
  $effect(() => () => stopHold());
</script>

<div class="step" class:bad={invalid} class:off={disabled}>
  <button
    type="button"
    class="sb"
    aria-label="Decrease"
    aria-controls={id}
    {disabled}
    onpointerdown={(e) => startHold(-1, e)}
    onpointerup={stopHold}
    onpointerleave={stopHold}
    onpointercancel={stopHold}
    onkeydown={(e) => {
      if (e.key === " " || e.key === "Enter") e.preventDefault();
    }}
    onclick={(e) => {
      if (e.detail === 0) nudge(-1);
    }}
  >
    <Icon name="minus" size={20} />
  </button>
  <input
    {id}
    class="in"
    type="text"
    inputmode="decimal"
    role="spinbutton"
    autocomplete="off"
    spellcheck="false"
    aria-labelledby={labelId}
    aria-describedby={describedBy}
    aria-invalid={invalid ? "true" : undefined}
    aria-valuemin={min}
    aria-valuemax={max}
    aria-valuenow={value}
    {placeholder}
    {disabled}
    value={text}
    oninput={onInput}
    onkeydown={onKey}
  />
  <button
    type="button"
    class="sb"
    aria-label="Increase"
    aria-controls={id}
    {disabled}
    onpointerdown={(e) => startHold(1, e)}
    onpointerup={stopHold}
    onpointerleave={stopHold}
    onpointercancel={stopHold}
    onkeydown={(e) => {
      if (e.key === " " || e.key === "Enter") e.preventDefault();
    }}
    onclick={(e) => {
      if (e.detail === 0) nudge(1);
    }}
  >
    <Icon name="plus" size={20} />
  </button>
</div>

<style>
  .step {
    display: grid;
    grid-template-columns: var(--ctl-h) minmax(0, 1fr) var(--ctl-h);
    min-height: var(--ctl-h);
    border-radius: var(--r-ctl);
    background: var(--ctl);
  }
  .step.bad {
    box-shadow: inset 0 0 0 2px var(--crit);
  }
  .step.off {
    opacity: 0.6;
  }
  /* The value sits on the stepper's own fill, as in the mockup. */
  .step .in {
    background: transparent !important;
    color: var(--ctl-text);
  }
  .in {
    box-sizing: border-box;
    width: 100%;
    min-width: 0;
    min-height: var(--ctl-h);
    padding: 0 2px;
    border: 0;
    background: transparent;
    color: var(--ctl-text);
    font: inherit;
    font-variant-numeric: tabular-nums;
    text-align: center;
    outline: 0;
  }
  .in::placeholder {
    color: var(--ctl-sub);
    opacity: 1;
  }
  .sb {
    display: grid;
    place-items: center;
    width: var(--ctl-h);
    height: var(--ctl-h);
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: transparent;
    color: var(--ctl-text);
    cursor: pointer;
    touch-action: manipulation;
    user-select: none;
  }
  .sb:hover {
    background: var(--ctl-h2);
  }
  .sb:active {
    background: var(--ctl-h2);
  }
  .sb:disabled {
    cursor: default;
  }
  /* Keyboard focus is a background step, not a ring. */
  .step:focus-within {
    background: var(--ctl-h2);
  }
  .in:focus-visible {
    outline: none;
  }
</style>
