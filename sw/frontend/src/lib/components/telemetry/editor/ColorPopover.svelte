<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { PALETTE, hexToHsl, hslToHex, normalizeHex, type Hsl } from "$lib/editor/colorMath";
  import { closeOnOutsideScroll, fieldBox } from "$lib/popoverAnchor";
  import { portal } from "$lib/layers";

  /** Colour field: swatch, hex input and chevron in one field. The popover holds the palette and hue and saturation sliders. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: string;
    invalid?: boolean;
  }

  let { id, labelId, describedBy, value = $bindable(), invalid = false }: Props = $props();

  let hsl = $state<Hsl>(hexToHsl(normalizeHex(value) ?? "#01AAFF"));
  let hexText = $state("");
  // The field shows the saved colour; typing changes it on blur or Enter.
  $effect(() => {
    hexText = current;
  });
  // Hue and saturation follow the colour from any source (field, palette, form reset). A slider move already
  // matches, so this does not rewrite hue when the colour is grey (saturation 0 has no hue).
  $effect(() => {
    if (hslToHexSafe(hsl) !== current) hsl = hexToHsl(current);
  });
  function hslToHexSafe(v: Hsl) {
    return hslToHex(v).toUpperCase();
  }
  let hexError = $state("");
  let trigger: HTMLButtonElement | undefined = $state();
  let pop: HTMLDivElement | undefined = $state();
  const popId = $derived(`${id}-pop`);

  const current = $derived(normalizeHex(value) ?? "#888888");

  function setColor(hex: string, fromHsl = false) {
    value = hex;
    hexText = hex;
    hexError = "";
    if (!fromHsl) hsl = hexToHsl(hex);
  }

  function onHue(e: Event) {
    hsl = { ...hsl, h: Number((e.currentTarget as HTMLInputElement).value) };
    setColor(hslToHex(hsl), true);
  }

  function onSat(e: Event) {
    hsl = { ...hsl, s: Number((e.currentTarget as HTMLInputElement).value) };
    setColor(hslToHex(hsl), true);
  }

  function applyHex() {
    const hex = normalizeHex(hexText);
    if (!hex) {
      hexError = "Use hex like #3498DB";
      return;
    }
    setColor(hex);
  }

  function place() {
    if (!trigger || !pop) return;
    const r = trigger.getBoundingClientRect();
    const field = fieldBox(trigger);
    const width = Math.min(304, window.innerWidth - 16);
    const height = pop.offsetHeight;
    const left = Math.min(Math.max(8, field.left), window.innerWidth - width - 8);
    const below = r.bottom + 8;
    const top = below + height > window.innerHeight - 8 ? Math.max(8, field.top - height - 8) : below;
    pop.style.width = `${width}px`;
    pop.style.left = `${left}px`;
    pop.style.top = `${top}px`;
  }

  let stopWatch: (() => void) | undefined;

  function onToggle(e: Event) {
    const open = (e as ToggleEvent).newState === "open";
    if (open) {
      hexText = current;
      hexError = "";
      place();
      // Scrolling the page or the sheet closes the popover: it never drifts from its field.
      stopWatch = closeOnOutsideScroll(pop, () => pop?.hidePopover());
    } else {
      stopWatch?.();
      stopWatch = undefined;
      trigger?.focus();
    }
  }

  function toggle() {
    if (!pop) return;
    if (pop.matches(":popover-open")) pop.hidePopover();
    else pop.showPopover();
  }

  const hueTrack = "linear-gradient(to right, hsl(0 100% 50%), hsl(60 100% 50%), hsl(120 100% 50%), hsl(180 100% 50%), hsl(240 100% 50%), hsl(300 100% 50%), hsl(360 100% 50%))";
  const satTrack = $derived(`linear-gradient(to right, hsl(${hsl.h} 0% ${hsl.l}%), hsl(${hsl.h} 100% ${hsl.l}%))`);
</script>

<div class="cfield" class:bad={invalid}>
  <button type="button" bind:this={trigger} class="swbtn" aria-label="Choose colour" aria-haspopup="dialog" aria-controls={popId} onclick={toggle}>
    <span class="sw" style="--cv: {current}"></span>
  </button>
  <input
    {id}
    class="hexin"
    type="text"
    maxlength="7"
    autocomplete="off"
    spellcheck="false"
    aria-labelledby={labelId}
    aria-describedby={hexError ? `${id}-hexmsg` : describedBy}
    aria-invalid={hexError ? "true" : undefined}
    bind:value={hexText}
    onkeydown={(e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        applyHex();
      }
    }}
    onblur={applyHex}
  />
  <button type="button" class="cchev" tabindex="-1" aria-hidden="true" onclick={toggle}>
    <Icon name="chevron-down" size={18} />
  </button>
</div>
{#if hexError}<p class="hexmsg" id="{id}-hexmsg" role="alert">{hexError}</p>{/if}

<!-- Portalled out of the field and the scrolling sheet body (lib/layers.ts): a popover inside either would lose its blur. -->
<div bind:this={pop} use:portal id={popId} class="pop" popover="auto" role="dialog" aria-label="Colour" ontoggle={onToggle}>
  <div class="pal" role="radiogroup" aria-label="Palette">
    {#each PALETTE as hex (hex)}
      {@const selected = current === hex}
      <button type="button" role="radio" aria-checked={selected} aria-label={hex} class="chip" onclick={() => setColor(hex)}>
        <span class="dot" style="--cv: {hex}">{#if selected}<Icon name="check" size={18} />{/if}</span>
      </button>
    {/each}
  </div>

  <div class="slider">
    <label for={`${id}-hue`}>Hue</label>
    <input id={`${id}-hue`} class="rng" type="range" min="0" max="360" step="1" value={hsl.h} oninput={onHue} style="--track: {hueTrack}; --cv: {current};" />
    <output class="val" for={`${id}-hue`}>{Math.round(hsl.h)}</output>
  </div>
  <div class="slider">
    <label for={`${id}-sat`}>Saturation</label>
    <input id={`${id}-sat`} class="rng" type="range" min="0" max="100" step="1" value={hsl.s} oninput={onSat} style="--track: {satTrack}; --cv: {current};" />
    <output class="val" for={`${id}-sat`}>{Math.round(hsl.s)}%</output>
  </div>
</div>

<style>
  .cfield {
    display: flex;
    align-items: center;
    width: 100%;
    min-height: var(--ctl-h);
    border-radius: var(--r-ctl);
    background: var(--ctl);
    min-width: 0;
  }
  .cfield.bad {
    box-shadow: inset 0 0 0 2px var(--crit);
  }
  .swbtn,
  .cchev {
    display: grid;
    place-items: center;
    flex: none;
    width: 44px;
    height: 44px;
    padding: 0;
    border: 0;
    border-radius: 12px;
    background: transparent;
    color: var(--ctl-text);
    cursor: pointer;
  }
  .cchev {
    cursor: pointer;
  }
  /* Pico draws a ring on focused buttons. Focus is a background step only. */
  .swbtn:focus,
  .swbtn:focus-visible {
    box-shadow: none;
  }
  .swbtn:focus-visible,
  .cchev:focus-visible,
  .chip:focus-visible {
    outline: none;
    background: var(--ctl-h2);
  }
  @media (hover: hover) {
    .swbtn:hover,
    .cchev:hover {
      background: var(--ctl-h2);
    }
  }
  .sw {
    width: 26px;
    height: 26px;
    border-radius: 8px;
    background: var(--cv);
  }
  .hexin {
    flex: 1;
    min-width: 0;
    height: 100%;
    min-height: var(--ctl-h);
    padding: 0 6px !important;
    border: 0;
    border-radius: 0 !important;
    background: transparent !important;
    color: var(--ctl-text);
    font: inherit;
    font-family: var(--mono);
    font-size: 0.9375rem;
    text-transform: uppercase;
  }
  /* One fill per field: the input never paints its own background. Focus is shown on the field. */
  .hexin,
  .hexin:hover,
  .hexin:focus,
  .hexin:focus-visible {
    background: transparent !important;
  }
  .hexin:focus-visible {
    outline: none;
  }
  .cfield:has(.hexin:focus-visible) {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .hexmsg {
    margin: 4px 2px 0;
    font-size: 0.85rem;
    font-weight: 700;
    color: var(--crit);
  }
  /* Field.svelte resets its controls (margin, slider height). A popover is portalled out of the field, so it resets its own. */
  .pop :global(button) {
    margin: 0 !important;
  }
  .pop :global(input) {
    margin: 0 !important;
    height: var(--ctl-h) !important;
    padding-top: 0 !important;
    padding-bottom: 0 !important;
  }
  .pop {
    inset: auto;
    box-sizing: border-box;
    width: min(304px, calc(100vw - 16px));
    margin: 0;
    padding: 14px;
    border: 0;
    border-radius: var(--r-float);
    background: var(--float);
    -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
    box-shadow: var(--shadow-float);
    color: var(--text);
  }
  .pal {
    display: grid;
    grid-template-columns: repeat(6, minmax(0, 1fr));
    gap: 8px;
    margin-bottom: 10px;
  }
  .chip {
    display: grid;
    place-items: center;
    height: 44px;
    padding: 0;
    border: 0;
    border-radius: 12px;
    background: transparent;
    cursor: pointer;
  }
  .dot {
    display: grid;
    place-items: center;
    width: 100%;
    height: 100%;
    border-radius: 12px;
    background: var(--cv);
    color: #fff;
    box-shadow: inset 0 0 0 1.5px rgba(0, 0, 0, 0.3);
  }
  .slider {
    display: grid;
    grid-template-columns: 1fr auto;
    gap: 2px 8px;
    align-items: center;
    margin-top: 8px;
  }
  .slider label {
    grid-column: 1 / -1;
    font-size: 0.8125rem;
    font-weight: 700;
  }
  .rng {
    grid-column: 1;
    width: 100%;
    height: 44px;
    margin: 0;
    background: transparent;
    appearance: none;
  }
  .rng::-webkit-slider-runnable-track {
    height: 14px;
    border-radius: 7px;
    background: var(--track);
    box-shadow: inset 0 0 0 1.5px rgba(0, 0, 0, 0.25);
  }
  .rng::-webkit-slider-thumb {
    appearance: none;
    box-sizing: border-box;
    width: 30px;
    height: 30px;
    margin-top: -8px;
    border-radius: 50%;
    background: var(--cv);
    border: 3px solid #fff;
    box-shadow: 0 0 0 1.5px #1e1a16;
  }
  .rng::-moz-range-track {
    height: 14px;
    border-radius: 7px;
    background: var(--track);
  }
  .rng::-moz-range-thumb {
    box-sizing: border-box;
    width: 30px;
    height: 30px;
    border-radius: 50%;
    background: var(--cv);
    border: 3px solid #fff;
    box-shadow: 0 0 0 1.5px #1e1a16;
  }
  .val {
    grid-column: 2;
    grid-row: 2;
    min-width: 40px;
    text-align: right;
    font-variant-numeric: tabular-nums;
    font-weight: 700;
    font-size: 0.85rem;
  }
</style>
