<script lang="ts">
  /** Slider on a 0..1 track with the look of the PID edit interval slider: ticks, a readout on the right, a scale under it and an optional hatched stop at the right end. The parent maps positions to values. */
  interface Props {
    labelId: string;
    pos: number;
    readout: string;
    min: number;
    max: number;
    now: number;
    text: string;
    /** Dots on the track; `on` when the value has passed them. */
    ticks: { pos: number; on: boolean }[];
    /** Labels under the track; `edge` pins one to an end instead of centring it on `pos`. */
    scale: { pos: number; text: string; edge?: "left" | "right" }[];
    /** Hatched stop at the right end (the repeat slider's Endless). `stopOn` fills it. */
    stop?: boolean;
    stopOn?: boolean;
    onPos: (x: number) => void;
    /** Returns true when the key was used. */
    onKey: (key: string) => boolean;
  }

  let { labelId, pos, readout, min, max, now, text, ticks, scale, stop = false, stopOn = false, onPos, onKey }: Props = $props();

  let track: HTMLElement | undefined = $state();

  function atPointer(e: PointerEvent) {
    const box = track?.getBoundingClientRect();
    if (!box || box.width === 0) return;
    onPos(Math.min(1, Math.max(0, (e.clientX - box.left) / box.width)));
  }

  function onDown(e: PointerEvent) {
    if (e.button !== 0) return;
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
    (e.currentTarget as HTMLElement).focus();
    atPointer(e);
  }
</script>

<div class="slider" class:end={stopOn}>
  <div
    class="pad"
    role="slider"
    tabindex="0"
    aria-labelledby={labelId}
    aria-valuemin={min}
    aria-valuemax={max}
    aria-valuenow={now}
    aria-valuetext={text}
    onkeydown={(e) => onKey(e.key) && e.preventDefault()}
    onpointerdown={onDown}
    onpointermove={(e) => e.buttons === 1 && atPointer(e)}
  >
    <div class="track" bind:this={track}>
      {#if stop}<span class="stop" aria-hidden="true"></span>{/if}
      <span class="fill" style="width: {pos * 100}%"></span>
      {#each ticks as t (t.pos)}
        <span class="tick" class:on={t.on} style="left: {t.pos * 100}%"></span>
      {/each}
      <span class="thumb" style="left: {pos * 100}%"></span>
    </div>
    <span class="readout">{readout}</span>
  </div>
  <div class="scale" aria-hidden="true">
    {#each scale as s (s.text)}
      <span class:l={s.edge === "left"} class:r={s.edge === "right"} style="left: {s.edge === "right" ? 100 : s.edge === "left" ? 0 : s.pos * 100}%">{s.text}</span>
    {/each}
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
  .stop {
    position: absolute;
    top: 0;
    bottom: 0;
    right: -18px;
    width: calc(12% + 18px);
    border-radius: var(--r-xs);
    background: repeating-linear-gradient(135deg, transparent 0 3px, var(--ctl-sub) 3px 4px);
    opacity: 0.6;
  }
  .end .stop {
    opacity: 1;
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
    width: 26px;
    height: 26px;
    margin: -13px 0 0 -13px;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 0 0 4px var(--ctl);
  }
  .end .thumb {
    background: var(--ctl-text);
  }
  .readout {
    flex: none;
    width: 80px;
    padding-right: 14px;
    text-align: right;
    font-weight: 800;
    font-size: 0.9375rem;
    white-space: nowrap;
  }
  .scale {
    position: relative;
    height: 18px;
    margin: 0 80px 0 18px;
    font-size: 0.72rem;
    font-weight: 700;
    color: var(--muted);
  }
  .scale span {
    position: absolute;
    top: 0;
    transform: translateX(-50%);
    white-space: nowrap;
  }
  .scale .l {
    transform: none;
  }
  .scale .r {
    transform: translateX(-100%);
  }
</style>
