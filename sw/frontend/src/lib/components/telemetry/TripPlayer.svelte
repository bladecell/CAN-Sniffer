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
    onBack: () => void;
  }

  let { view, trip, onBack }: Props = $props();

  const span = $derived(Math.max(1, trip.endMs - trip.startMs));
  const labelId = "trip-pos-label";
  const playing = $derived(view.win.pausedAt === null);
  // Read by the slider and the clock text. Updated from the frame loop, at most 4 times a second.
  let pos = $state(0);

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

<div class="card bar player">
  <div class="title">
    <h2 class="nm mono">{trip.name}</h2>
    <button type="button" class="btn" onclick={onBack}>Back to live</button>
  </div>
  <div class="controls">
    <button type="button" class="btn pri play" onclick={toggle} aria-label={playing ? "Pause" : atEnd(pos, trip.endMs) ? "Play again" : "Play"}>
      <Icon name={playing ? "pause" : "play"} size={18} />
      {playing ? "Pause" : atEnd(pos, trip.endMs) ? "Replay" : "Play"}
    </button>
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
    <span class="clock mono" aria-hidden="true">{elapsedClock(pos - trip.startMs)} / {elapsedClock(span)}</span>
  </div>
</div>

<style>
  .player {
    display: grid;
    gap: 12px;
    padding: var(--pad-card);
    border-radius: var(--r-card);
  }
  .title {
    display: flex;
    align-items: center;
    gap: 12px;
    min-width: 0;
  }
  .nm {
    flex: 1 1 auto;
    min-width: 0;
    margin: 0;
    overflow: hidden;
    font-size: 1.15rem;
    font-weight: 800;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .title .btn {
    flex: none;
  }
  .controls {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr) auto;
    align-items: center;
    gap: 12px 20px;
  }
  .seek {
    min-width: 0;
  }
  .play {
    justify-content: center;
    min-width: 8rem;
  }
  .clock {
    color: var(--muted);
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }
  .sr {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip: rect(0 0 0 0);
    white-space: nowrap;
  }
  /* Phone: the clock wraps to its own line under the slider. */
  @media (max-width: 859px) {
    .controls {
      grid-template-columns: auto minmax(0, 1fr);
    }
    .play {
      min-width: 0;
    }
    .clock {
      grid-column: 1 / -1;
      justify-self: end;
    }
  }
</style>
