<script lang="ts">
  import {
    SLIDER_MAX_MS,
    STATIC_MS,
    brief,
    intervalToPos,
    pageDown,
    pageUp,
    posToInterval,
    stepDown,
    stepUp,
  } from "$lib/editor/pollRate";

  /** Log slider from Static (far left) to 10 s. Arrow keys step, PageUp and PageDown jump, Home and End go to the ends. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: number | undefined;
    invalid?: boolean;
  }

  let { id, labelId, describedBy, value = $bindable(), invalid = false }: Props = $props();

  const ms = $derived(value ?? STATIC_MS);
  const pos = $derived(intervalToPos(ms));
  const ticks = [100, 500, 1000, 5000];

  // Moving off Static returns to the last polled interval, not a default.
  let lastPolled = $state(512);
  $effect(() => {
    if (ms > 0) lastPolled = ms;
  });

  let track: HTMLElement | undefined = $state();

  function onKey(e: KeyboardEvent) {
    const next: Record<string, number> = {
      ArrowUp: stepUp(ms, lastPolled),
      ArrowRight: stepUp(ms, lastPolled),
      ArrowDown: stepDown(ms),
      ArrowLeft: stepDown(ms),
      PageUp: pageUp(ms),
      PageDown: pageDown(ms),
      Home: STATIC_MS,
      End: SLIDER_MAX_MS,
    };
    const target = next[e.key];
    if (target !== undefined) {
      e.preventDefault();
      value = target;
    }
  }

  function atPointer(e: PointerEvent) {
    const box = track?.getBoundingClientRect();
    if (!box) return;
    value = posToInterval(Math.min(1, Math.max(0, (e.clientX - box.left) / box.width)));
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

<div class="slider" class:off={ms === 0} class:bad={invalid}>
  <div
    {id}
    class="pad"
    role="slider"
    tabindex="0"
    aria-labelledby={labelId}
    aria-describedby={describedBy}
    aria-invalid={invalid ? "true" : undefined}
    aria-valuemin={STATIC_MS}
    aria-valuemax={SLIDER_MAX_MS}
    aria-valuenow={Math.min(ms, SLIDER_MAX_MS)}
    aria-valuetext={ms === 0 ? "Static, read once and never polled" : `${ms} milliseconds`}
    onkeydown={onKey}
    onpointerdown={onDown}
    onpointermove={onMove}
  >
    <div class="track" bind:this={track}>
      <span class="stop" aria-hidden="true"></span>
      <span class="fill" style="width: {Math.max(0, pos - 0.12) * 100}%"></span>
      {#each ticks as t (t)}
        <span class="tick" class:on={ms > 0 && ms >= t} style="left: {intervalToPos(t) * 100}%"></span>
      {/each}
      <span class="thumb" style="left: {pos * 100}%"></span>
    </div>
    <span class="readout">{brief(ms)}</span>
  </div>
  <div class="scale" aria-hidden="true">
    <span style="left: 6%">Static</span>
    <span style="left: {intervalToPos(100) * 100}%">100 ms</span>
    <span style="left: {intervalToPos(1000) * 100}%">1 s</span>
    <span style="left: 100%; transform: translateX(-100%)">10 s</span>
  </div>
</div>

<style>
  .pad {
    position: relative;
    display: flex;
    align-items: center;
    box-sizing: border-box;
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
    margin: 0 18px 0 18px;
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
  /* The Static stop: hatched, and solid when Static is chosen. */
  .stop {
    position: absolute;
    top: 0;
    bottom: 0;
    left: -18px;
    width: calc(12% + 18px);
    border-radius: 8px;
    background: repeating-linear-gradient(135deg, transparent 0 3px, var(--ctl-sub) 3px 4px);
    opacity: 0.6;
  }
  .slider.off .stop {
    opacity: 1;
  }
  .fill {
    position: absolute;
    left: 12%;
    top: 50%;
    height: 6px;
    margin-top: -3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .tick {
    position: absolute;
    top: 50%;
    width: 10px;
    height: 10px;
    margin: -5px 0 0 -5px;
    border-radius: 50%;
    background: var(--ctl-sub);
  }
  .tick.on {
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
  .slider.off .thumb {
    background: var(--ctl-text);
  }
  .readout {
    flex: none;
    width: 92px;
    padding-right: 14px;
    text-align: right;
    font-weight: 800;
    font-size: 0.9375rem;
    white-space: nowrap;
  }
  .slider.bad .pad {
    outline: 2px solid var(--crit);
    outline-offset: 0;
  }
  .scale {
    position: relative;
    height: 20px;
    margin: 2px 92px 0 18px;
    font-size: 0.75rem;
    font-weight: 700;
    color: var(--muted);
  }
  .scale span {
    position: absolute;
    top: 0;
    transform: translateX(-50%);
    white-space: nowrap;
  }
  .scale span:first-child {
    transform: translateX(-50%);
  }
  @media (prefers-reduced-motion: reduce) {
    .thumb {
      transition: none;
    }
  }
</style>
