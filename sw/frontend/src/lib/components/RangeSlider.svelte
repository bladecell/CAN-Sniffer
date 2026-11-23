<script lang="ts">
  import "./slider.css";
  import { keyTarget, moveThumb, nearestThumb, tickAt, type RangeSpec } from "$lib/rangeSlider";

  /**
   * The one slider of the app. One thumb (value on the right) or two (low value left, high value right).
   * Linear on min to max by default; `toPos` and `fromPos` give a non-linear track (log scales). Pointer presses snap to a tick within 10 px.
   * With `parse` the values are typed text: Enter or leaving the field commits, Escape and unreadable text put the old value back.
   * Keys: arrows move keyStep, PageUp and PageDown page, Home and End the ends; `keyFor` replaces that. Two thumbs never cross and keep `gap` apart.
   */
  interface Props {
    /** One id per thumb: the element that names it. */
    labelIds: string[];
    values: number[];
    min: number;
    max: number;
    /** Pointer and typed values snap to this. Leave it out on a non-linear track. */
    step?: number;
    keyStep?: number;
    page?: number;
    gap?: number;
    toPos?: (v: number) => number;
    fromPos?: (x: number) => number;
    keyFor?: (i: number, v: number, key: string) => number | undefined;
    format: (v: number) => string;
    /** Screen reader text of a value; the readout text by default. */
    valueText?: (v: number) => string;
    /** Makes the readouts editable. Returns the value for the text, or null when it is not one. */
    parse?: (text: string) => number | null;
    /** Dots on the track and snap points; lit between the thumbs (or up to the single thumb). */
    ticks?: number[];
    /** Labels under the track; `at` overrides the position, `edge` pins one to an end. */
    scale?: { value: number; text: string; at?: number; edge?: "left" | "right" }[];
    /** Hatched stop at one end: `share` of the track, solid and with a plain thumb when `on`. A left stop also starts the fill after it. */
    stop?: { side: "left" | "right"; share: number; on: boolean };
    /** Fixed width (for rows beside a title) instead of the full width of the parent. */
    fit?: boolean;
    /** false hides the value readout (the thumb and scale carry the meaning). */
    readouts?: boolean;
    id?: string;
    describedBy?: string;
    invalid?: boolean;
    onChange: (values: number[]) => void;
  }

  let {
    labelIds,
    values,
    min,
    max,
    step,
    keyStep = step ?? 1,
    page = keyStep,
    gap,
    toPos,
    fromPos,
    keyFor,
    format,
    valueText,
    parse,
    ticks = [],
    scale = [],
    stop,
    fit = false,
    readouts = true,
    id,
    describedBy,
    invalid = false,
    onChange,
  }: Props = $props();

  const spec: RangeSpec = $derived({ min, max, step, gap });
  const two = $derived(values.length === 2);
  const clampPos = (x: number) => Math.min(1, Math.max(0, x));
  const pos = (v: number) => clampPos(toPos ? toPos(v) : (Math.min(max, Math.max(min, v)) - min) / (max - min));
  const valueOf = (x: number) => (fromPos ? fromPos(x) : min + x * (max - min));
  const stopEnd = $derived(stop?.side === "left" ? stop.share : 0);
  const from = $derived(Math.max(stopEnd, two ? pos(values[0]) : 0));
  const to = $derived(pos(values[values.length - 1]));

  let track: HTMLElement | undefined = $state();
  let active = $state(0);

  function move(i: number, v: number) {
    const next = moveThumb(values, i, v, spec);
    if (next.some((n, k) => n !== values[k])) onChange(next);
  }

  /** Track position of a pointer event, or null while the track has no size. */
  function posAt(e: PointerEvent): number | null {
    const box = track?.getBoundingClientRect();
    if (!box || box.width === 0) return null;
    return clampPos((e.clientX - box.left) / box.width);
  }

  function valueAtPointer(x: number): number {
    return tickAt(x, ticks, pos, track?.getBoundingClientRect().width ?? 0) ?? valueOf(x);
  }

  function onDown(e: PointerEvent) {
    if (e.button !== 0 || (e.target as HTMLElement).closest("input")) return;
    const x = posAt(e);
    if (x === null) return;
    active = nearestThumb(values.map(pos), x);
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
    (e.currentTarget as HTMLElement).querySelector<HTMLElement>(`[data-t="${active}"]`)?.focus();
    move(active, valueAtPointer(x));
  }

  function onMove(e: PointerEvent) {
    if (e.buttons !== 1 || (e.target as HTMLElement).closest("input")) return;
    const x = posAt(e);
    if (x !== null) move(active, valueAtPointer(x));
  }

  function onKey(e: KeyboardEvent, i: number) {
    const target = keyFor ? keyFor(i, values[i], e.key) : keyTarget(values[i], e.key, spec, keyStep, page);
    if (target === undefined) return;
    e.preventDefault();
    move(i, target);
  }

  // Typed readouts: the draft shows while a field has focus.
  let editing = $state<number | null>(null);
  let draft = $state("");
  const shown = (i: number) => (editing === i ? draft : format(values[i]));

  function commit(i: number) {
    if (editing !== i) return;
    editing = null;
    const v = parse?.(draft);
    if (v !== null && v !== undefined) move(i, v);
  }

  function onEditKey(e: KeyboardEvent) {
    if (e.key === "Enter") {
      e.preventDefault();
      (e.currentTarget as HTMLInputElement).blur();
    } else if (e.key === "Escape") {
      editing = null;
      (e.currentTarget as HTMLInputElement).blur();
    }
  }

  function onEditFocus(e: FocusEvent, i: number) {
    editing = i;
    draft = format(values[i]);
    (e.currentTarget as HTMLInputElement).select();
  }
</script>

{#snippet readout(i: number, first: boolean)}
  {#if parse}
    <input
      class="sld-readout sld-edit"
      class:first
      type="text"
      inputmode="decimal"
      autocomplete="off"
      spellcheck="false"
      aria-label="Type a value"
      value={shown(i)}
      oninput={(e) => (draft = e.currentTarget.value)}
      onfocus={(e) => onEditFocus(e, i)}
      onblur={() => commit(i)}
      onkeydown={onEditKey}
    />
  {:else}
    <span class="sld-readout" class:first>{format(values[i])}</span>
  {/if}
{/snippet}

<div class="sld rs" class:two class:fit class:end={stop?.on} class:bad={invalid}>
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <div class="sld-pad" onpointerdown={onDown} onpointermove={onMove}>
    {#if two && readouts}{@render readout(0, true)}{/if}
    <div class="sld-track" bind:this={track}>
      {#if stop}<span class="sld-stop" class:left={stop.side === "left"} style="width: calc({stop.share * 100}% + 18px)" aria-hidden="true"></span>{/if}
      <span class="sld-fill" style="left: {from * 100}%; width: {Math.max(0, to - from) * 100}%"></span>
      {#each ticks as t (t)}
        <span class="sld-tick" class:on={pos(t) >= from && pos(t) <= to} style="left: {pos(t) * 100}%"></span>
      {/each}
      {#each values as v, i (i)}
        <span
          id={i === 0 ? id : undefined}
          class="sld-thumb"
          data-t={i}
          role="slider"
          tabindex="0"
          aria-labelledby={labelIds[i]}
          aria-describedby={describedBy}
          aria-invalid={invalid ? "true" : undefined}
          aria-valuemin={two && i === 1 ? values[0] + (gap ?? 0) : min}
          aria-valuemax={two && i === 0 ? values[1] - (gap ?? 0) : max}
          aria-valuenow={Math.min(max, Math.max(min, v))}
          aria-valuetext={valueText ? valueText(v) : format(v)}
          style="left: {pos(v) * 100}%"
          onkeydown={(e) => onKey(e, i)}
        ></span>
      {/each}
    </div>
    {#if readouts}
      {@render readout(values.length - 1, false)}
    {/if}
  </div>
  {#if scale.length > 0}
    <div class="sld-scale" aria-hidden="true">
      {#each scale as s (s.text)}
        <span class:l={s.edge === "left"} class:r={s.edge === "right"} style="left: {s.edge === 'right' ? 100 : s.edge === 'left' ? 0 : (s.at ?? pos(s.value)) * 100}%">{s.text}</span>
      {/each}
    </div>
  {/if}
</div>

<style>
  .rs.fit {
    width: clamp(240px, 40vw, 360px);
    min-width: 240px;
  }
  /* Two thumbs: a value at each end, so the scale line is inset on both sides. */
  .rs.two :global(.sld-scale) {
    margin: 0 98px;
  }
</style>
