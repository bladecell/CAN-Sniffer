<script lang="ts">
  import { untrack } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import RangeSlider from "$lib/components/RangeSlider.svelte";
  import { chartFrames } from "$lib/chartFrames.svelte";
  import { elapsedClock } from "$lib/recorderView";
  import { atEnd, clampMs, playheadOf } from "$lib/telemetryChart/playback";
  import type { TelemetryView } from "$lib/telemetryChart/viewState.svelte";
  import { freezeWin } from "$lib/telemetryChart/window";
  import type { LoadedTrip } from "$lib/tripViewer.svelte";

  interface Props {
    view: TelemetryView;
    trip: LoadedTrip;
  }

  let { view, trip }: Props = $props();

  const span = $derived(Math.max(1, trip.endMs - trip.startMs));
  const labelId = "trip-pos-label";
  const playing = $derived(view.win.pausedAt === null);
  // Read by the slider and the clock text. Updated from the frame loop, at most 4 times a second.
  let pos = $state(0);
  const label = $derived(playing ? "Pause" : atEnd(pos, trip.endMs) ? "Replay" : "Play");

  $effect(() => {
    void trip;
    const start = trip.startMs;
    untrack(() => {
      pos = start;
      view.seek(start, false);
    });
  });

  // The frame loop moves the clock; this reads it back and stops at the last sample.
  $effect(() =>
    chartFrames.subscribe((time) => {
      const present = view.presentAt(time);
      if (present === null) return;
      if (view.win.pausedAt === null && atEnd(present, trip.endMs)) view.set({ ...view.win, endAgo: 0, pausedAt: trip.endMs });
      const next = playheadOf(view.win, present, trip.startMs, trip.endMs);
      if (Math.abs(next - pos) >= 250 || (next !== pos && !playing)) pos = next;
    }),
  );

  function toggle() {
    if (playing) view.apply((w, now) => freezeWin(w, now));
    else view.seek(atEnd(pos, trip.endMs) ? trip.startMs : pos, true);
  }

  function seek(elapsedMs: number) {
    pos = clampMs(trip.startMs + elapsedMs, trip.startMs, trip.endMs);
    view.seek(pos, playing);
  }
</script>

<div class="player">
  <div class="card p1">
    <button type="button" class="ibtn" onclick={toggle} aria-label={label} title={label}>
      {#if playing}
        <Icon name="pause" size={20} />
      {:else if atEnd(pos, trip.endMs)}
        <svg class="replay" viewBox="0 0 24 24" aria-hidden="true"><path d="M5 4.5v4h4" /><path d="M5.6 8.5A7.5 7.5 0 1 1 4.5 12" /></svg>
      {:else}
        <Icon name="play" size={20} />
      {/if}
    </button>
    <span class="name" title={trip.name}>{trip.name}</span>
    <div class="seek">
      <span id={labelId} class="sr">Position in {trip.name}</span>
      <RangeSlider
        labelIds={[labelId]}
        values={[pos - trip.startMs]}
        min={0}
        max={span}
        step={100}
        keyStep={1000}
        page={10_000}
        readouts={false}
        format={elapsedClock}
        onChange={([v]) => seek(v)}
      />
    </div>
    <span class="clock" aria-hidden="true">{elapsedClock(pos - trip.startMs)} <i>/</i> {elapsedClock(span)}</span>
  </div>
</div>

<style>
  /* Replay player, mockup 1 (plan/mockups/replay-player-1.html): one compact row that folds to two rows on a narrow card. */
  .player {
    container-type: inline-size;
  }
  .p1 {
    display: grid;
    grid-template-columns: auto auto minmax(80px, 1fr) auto;
    align-items: center;
    gap: 14px;
    padding: var(--pad-card);
  }
  .ibtn {
    display: inline-grid;
    place-items: center;
    width: 40px;
    height: 40px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: inherit;
    cursor: pointer;
  }
  .ibtn:hover {
    background: var(--ctl-h2, var(--hov));
  }
  .replay {
    width: 20px;
    height: 20px;
    fill: none;
    stroke: currentColor;
    stroke-width: 2.2;
    stroke-linecap: round;
    stroke-linejoin: round;
  }
  .name {
    min-width: 0;
    overflow: hidden;
    color: var(--muted);
    font-family: var(--mono, ui-monospace, SFMono-Regular, Menlo, Consolas, monospace);
    font-size: 0.85rem;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .seek {
    min-width: 0;
  }
  .clock {
    font-family: var(--mono, ui-monospace, SFMono-Regular, Menlo, Consolas, monospace);
    font-size: 0.85rem;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .clock i {
    color: var(--muted);
    font-style: normal;
  }
  .sr {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip: rect(0 0 0 0);
    white-space: nowrap;
  }
  /* Narrow: play and name on the first row; the slider and the clock under them. 44 px touch targets. */
  @container (max-width: 640px) {
    .p1 {
      grid-template-columns: auto 1fr auto;
      grid-template-areas:
        "play name name"
        "seek seek clock";
      gap: 4px 10px;
      padding: 6px;
    }
    .ibtn {
      grid-area: play;
      width: 44px;
      height: 44px;
    }
    .name {
      grid-area: name;
    }
    .seek {
      grid-area: seek;
    }
    .clock {
      grid-area: clock;
    }
  }
</style>
