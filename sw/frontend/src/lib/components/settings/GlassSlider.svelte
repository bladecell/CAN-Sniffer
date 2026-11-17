<script lang="ts">
  import { GLASS_MAX, GLASS_MIN } from "$lib/appearance";

  /** 0 (most opaque) to 100 (most see-through). Arrow keys step by 5, PageUp and PageDown by 20, Home and End go to the ends. */
  interface Props {
    labelId: string;
    value: number;
    onChange: (v: number) => void;
  }

  let { labelId, value, onChange }: Props = $props();

  let track: HTMLElement | undefined = $state();
  const clamp = (n: number) => Math.min(GLASS_MAX, Math.max(GLASS_MIN, Math.round(n)));

  function onKey(e: KeyboardEvent) {
    const next: Record<string, number> = {
      ArrowRight: value + 5,
      ArrowUp: value + 5,
      ArrowLeft: value - 5,
      ArrowDown: value - 5,
      PageUp: value + 20,
      PageDown: value - 20,
      Home: GLASS_MIN,
      End: GLASS_MAX,
    };
    const target = next[e.key];
    if (target === undefined) return;
    e.preventDefault();
    onChange(clamp(target));
  }

  function atPointer(e: PointerEvent) {
    const box = track?.getBoundingClientRect();
    if (!box || box.width === 0) return;
    onChange(clamp(((e.clientX - box.left) / box.width) * GLASS_MAX));
  }

  function onDown(e: PointerEvent) {
    if (e.button !== 0) return;
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
    (e.currentTarget as HTMLElement).focus();
    atPointer(e);
  }

  function onMove(e: PointerEvent) {
    if (e.buttons === 1) atPointer(e);
  }
</script>

<div
  class="pad"
  role="slider"
  tabindex="0"
  aria-labelledby={labelId}
  aria-valuemin={GLASS_MIN}
  aria-valuemax={GLASS_MAX}
  aria-valuenow={value}
  aria-valuetext={`${value} percent`}
  onkeydown={onKey}
  onpointerdown={onDown}
  onpointermove={onMove}
>
  <div class="track" bind:this={track}>
    <span class="fill" style="width: {value}%"></span>
    <span class="thumb" style="left: {value}%"></span>
  </div>
  <span class="readout">{value}%</span>
</div>

<style>
  .pad {
    display: flex;
    align-items: center;
    box-sizing: border-box;
    width: 100%;
    height: var(--ctl-h);
    border-radius: var(--r-ctl);
    background: var(--ctl);
    touch-action: none;
    user-select: none;
    cursor: pointer;
  }
  .pad:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .track {
    position: relative;
    flex: 1;
    min-width: 0;
    height: 44px;
    margin: 0 18px;
  }
  .track::before {
    content: "";
    position: absolute;
    left: 0;
    right: 0;
    top: 50%;
    height: 6px;
    margin-top: -3px;
    border-radius: 3px;
    background: var(--ctl-sub);
    opacity: 0.4;
  }
  .fill {
    position: absolute;
    left: 0;
    top: 50%;
    height: 6px;
    margin-top: -3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .thumb {
    position: absolute;
    top: 50%;
    width: 30px;
    height: 30px;
    margin: -15px 0 0 -15px;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 0 0 4px var(--ctl);
  }
  .readout {
    flex: none;
    width: 64px;
    padding-right: 14px;
    text-align: right;
    font-weight: 800;
    font-size: 0.9375rem;
  }
</style>
