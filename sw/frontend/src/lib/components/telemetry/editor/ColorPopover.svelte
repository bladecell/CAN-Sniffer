<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { PALETTE, hexToHsl, hslToHex, normalizeHex, type Hsl } from "$lib/editor/colorMath";

  /** Colour field: a trigger that opens a popover with a palette, a hex input and hue and saturation sliders. */
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
    const width = Math.min(320, window.innerWidth - 16);
    const height = pop.offsetHeight;
    const left = Math.min(Math.max(8, r.left), window.innerWidth - width - 8);
    const below = r.bottom + 8;
    const top = below + height > window.innerHeight - 8 ? Math.max(8, r.top - height - 8) : below;
    pop.style.width = `${width}px`;
    pop.style.left = `${left}px`;
    pop.style.top = `${top}px`;
  }

  function onToggle(e: Event) {
    const open = (e as ToggleEvent).newState === "open";
    if (open) {
      hexText = current;
      hexError = "";
      place();
    } else {
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

<div class="cp" class:bad={invalid}>
  <button
    type="button"
    bind:this={trigger}
    id={id}
    class="trig"
    aria-labelledby={`${labelId} ${id}-v`}
    aria-describedby={describedBy}
    aria-haspopup="dialog"
    aria-controls={popId}
    onclick={toggle}
  >
    <span class="sw" style="--cv: {current}" aria-hidden="true"></span>
    <span class="mono" id={`${id}-v`}>{current}</span>
    <Icon name="chevron-down" size={16} class="chev" />
  </button>

  <div bind:this={pop} id={popId} class="pop" popover="auto" role="dialog" aria-label="Colour" ontoggle={onToggle}>
    <div class="pal" role="radiogroup" aria-label="Palette">
      {#each PALETTE as hex (hex)}
        {@const selected = current === hex}
        <button
          type="button"
          role="radio"
          aria-checked={selected}
          aria-label={hex}
          class="chip"
          class:on={selected}
          onclick={() => setColor(hex)}
        >
          <span class="dot" style="--cv: {hex}">{#if selected}<Icon name="check" size={16} />{/if}</span>
        </button>
      {/each}
    </div>

    <div class="hexrow">
      <label class="hl" for={`${id}-hex`}>Hex</label>
      <input
        id={`${id}-hex`}
        class="in mono"
        type="text"
        maxlength="7"
        autocomplete="off"
        spellcheck="false"
        bind:value={hexText}
        aria-invalid={hexError ? "true" : undefined}
        aria-describedby={`${id}-hexmsg`}
        onkeydown={(e) => {
          if (e.key === "Enter") {
            e.preventDefault();
            applyHex();
          }
        }}
        onblur={applyHex}
      />
    </div>
    <p class="hexmsg" id={`${id}-hexmsg`}>{hexError}</p>

    <div class="slider">
      <label for={`${id}-hue`}>Hue</label>
      <input
        id={`${id}-hue`}
        class="rng"
        type="range"
        min="0"
        max="360"
        step="1"
        value={hsl.h}
        oninput={onHue}
        style="--track: {hueTrack}"
      />
      <output class="val" for={`${id}-hue`}>{Math.round(hsl.h)}</output>
    </div>
    <div class="slider">
      <label for={`${id}-sat`}>Saturation</label>
      <input
        id={`${id}-sat`}
        class="rng"
        type="range"
        min="0"
        max="100"
        step="1"
        value={hsl.s}
        oninput={onSat}
        style="--track: {satTrack}"
      />
      <output class="val" for={`${id}-sat`}>{Math.round(hsl.s)}%</output>
    </div>
  </div>
</div>

<style>
  .cp {
    min-width: 0;
  }
  .trig {
    display: flex;
    align-items: center;
    gap: 10px;
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    padding: 0 12px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
    font-weight: 600;
    text-align: left;
    cursor: pointer;
  }
  .trig:hover {
    background: var(--ctl-h2);
  }
  .trig :global(.chev) {
    margin-left: auto;
    flex: none;
  }
  .cp.bad .trig {
    outline: 2px solid var(--crit);
    outline-offset: 0;
  }
  .sw {
    flex: none;
    width: 24px;
    height: 24px;
    border-radius: 50%;
    background: var(--cv);
    box-shadow: inset 0 0 0 1.5px color-mix(in srgb, var(--text) 40%, transparent);
  }
  .mono {
    font-family: var(--mono);
  }
  .pop {
    box-sizing: border-box;
    margin: 0;
    padding: 12px;
    border: 0;
    border-radius: var(--r-float);
    background: var(--float-solid);
    color: var(--text);
    box-shadow: var(--shadow-float);
    max-width: calc(100vw - 16px);
    max-height: calc(100dvh - 16px);
    overflow: auto;
    position: fixed;
    inset: auto;
  }
  .pop:popover-open {
    animation: pop-in 0.22s var(--ease-out);
  }
  @supports (background: color-mix(in srgb, red 50%, blue)) {
    .pop {
      background: var(--float);
      -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
      backdrop-filter: blur(var(--blur)) saturate(1.35);
    }
  }
  .pal {
    display: grid;
    grid-template-columns: repeat(6, minmax(0, 1fr));
    gap: 0;
    margin-bottom: 8px;
  }
  .chip {
    display: grid;
    place-items: center;
    width: 100%;
    min-width: 44px;
    height: 44px;
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: transparent;
    cursor: pointer;
  }
  .chip:hover .dot {
    transform: scale(1.08);
  }
  .dot {
    display: grid;
    place-items: center;
    width: 30px;
    height: 30px;
    border-radius: 50%;
    background: var(--cv);
    color: #fff;
    box-shadow: inset 0 0 0 1.5px color-mix(in srgb, var(--text) 40%, transparent);
    transition: transform 0.15s;
  }
  .chip.on {
    outline: 2px solid var(--accent);
    outline-offset: -2px;
  }
  .hexrow {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr);
    gap: 10px;
    align-items: center;
  }
  .hl,
  .slider label {
    font-size: 0.85rem;
    font-weight: 700;
  }
  .in {
    box-sizing: border-box;
    width: 100%;
    min-height: 40px;
    padding: 0 12px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
  }
  .hexmsg {
    min-height: 18px;
    margin: 2px 0 6px;
    color: var(--crit);
    font-size: 0.78rem;
    font-weight: 700;
  }
  .slider {
    display: grid;
    grid-template-columns: 84px minmax(0, 1fr) 48px;
    gap: 10px;
    align-items: center;
    min-height: 44px;
  }
  .val {
    font-variant-numeric: tabular-nums;
    font-size: 0.82rem;
    text-align: right;
  }
  .rng {
    -webkit-appearance: none;
    appearance: none;
    width: 100%;
    height: 44px;
    margin: 0;
    background: transparent;
    cursor: pointer;
  }
  .rng::-webkit-slider-runnable-track {
    height: 10px;
    border-radius: 10px;
    background: var(--track);
    box-shadow: inset 0 0 0 1px color-mix(in srgb, var(--text) 25%, transparent);
  }
  .rng::-moz-range-track {
    height: 10px;
    border-radius: 10px;
    background: var(--track);
    box-shadow: inset 0 0 0 1px color-mix(in srgb, var(--text) 25%, transparent);
  }
  .rng::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 26px;
    height: 26px;
    margin-top: -8px;
    border: 3px solid #fff;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 1px 4px rgba(0, 0, 0, 0.4);
  }
  .rng::-moz-range-thumb {
    width: 22px;
    height: 22px;
    border: 3px solid #fff;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 1px 4px rgba(0, 0, 0, 0.4);
  }
  .rng:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
    border-radius: 8px;
  }
  @keyframes pop-in {
    from {
      opacity: 0;
      transform: translateY(-6px) scale(0.97);
    }
  }
  @media (prefers-reduced-motion: reduce) {
    .pop:popover-open {
      animation: none;
    }
  }
</style>
