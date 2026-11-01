<script lang="ts">
  import { getContext } from "svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { buildPlotModel, type PlotMode, type PlotSeries, type PlotModel } from "$lib/telemetryChart/plotModel";
  import { createSeriesFeed } from "$lib/telemetryChart/feed.svelte";
  import { TELEMETRY_VIEW, type TelemetryView } from "$lib/telemetryChart/viewState.svelte";
  import PlotSvg from "./PlotSvg.svelte";

  interface Props {
    n: number;
    pids: number[];
    active: boolean;
    fullscreen: boolean;
    /** Four or six plots: the legend stays on one line. */
    dense: boolean;
    plotHeight: number;
    laneHeight: number;
    onActivate: () => void;
    onFullscreen: () => void;
    onStats: (rows: PlotModel["stats"]) => void;
  }

  let { n, pids, active, fullscreen, dense, plotHeight, laneHeight, onActivate, onFullscreen, onStats }: Props = $props();
  const view = getContext<TelemetryView>(TELEMETRY_VIEW);

  let mode = $state<PlotMode>("overlay");
  let fit = $state(false);
  let trayOpen = $state(false);
  /** Full screen keeps the tools visible; the Exit button is the one exit control. */
  const trayShown = $derived(trayOpen || fullscreen);
  let width = $state(0);

  const feed = createSeriesFeed(
    () => pids,
    (t) => view.observe(t),
  );

  const defs = $derived(new Map(vehicleStore.pidDefinitions.map((d) => [d.pid, d])));

  const series = $derived.by((): PlotSeries[] => {
    void feed.version;
    const now = view.presentAt();
    const w = view.win;
    const fromT = now === null ? -Infinity : now - (w.endAgo + w.len) * 1000 - 1000;
    return pids.map((pid) => {
      const def = defs.get(pid);
      const samples = feed.read(pid, fromT);
      const range: [number, number] | null =
        def && Number.isFinite(def.minValue) && Number.isFinite(def.maxValue) ? [def.minValue, def.maxValue] : null;
      return {
        id: pid,
        name: def?.name || `0x${pid.toString(16).toUpperCase()}`,
        unit: def?.unit ?? "",
        color: safeHexColor(def?.color),
        range,
        holdMs: Math.max(1500, 2 * (def?.update_interval_ms || 500)),
        t: samples.t,
        v: samples.v,
      };
    });
  });

  const model = $derived(
    buildPlotModel({
      width,
      height: plotHeight,
      mode,
      fit,
      laneH: laneHeight,
      win: view.win,
      now: view.presentAt() ?? 0,
      series,
      hoverF: view.hoverF,
    }),
  );

  $effect(() => {
    if (active) onStats(model.stats);
  });

  let csel = $state<HTMLButtonElement>();
  let exitBtn = $state<HTMLButtonElement>();

  // Full screen is a dialog-like layer: focus moves in to its exit button and returns to the plot's name on exit.
  $effect(() => {
    if (!fullscreen) return;
    queueMicrotask(() => exitBtn?.focus());
    return () => csel?.focus();
  });

  const emptyText = $derived(pids.length === 0 ? "Tick a PID in the list to chart it" : "Waiting for data...");
</script>

<section
  class="cell"
  class:fs={fullscreen}
  role={fullscreen ? "dialog" : undefined}
  aria-label={fullscreen ? `Plot ${n}, full screen` : `Plot ${n}`}
  aria-current={active ? "true" : undefined}
  onpointerdown={onActivate}
>
  <div class="head" class:dense>
    <button type="button" class="csel" bind:this={csel} aria-pressed={active} onclick={onActivate}>Plot {n}</button>

    <div class="legend" class:dense>
      {#each model.legend as l (l.id)}
        <span class="lg">
          <i style:--sw={l.color}></i>
          <span class="nm">{l.name}</span>
        </span>
      {/each}
    </div>

    <div class="tools">
      <div class="tray" class:open={trayShown}>
        <div class="trayin" inert={!trayShown}>
          <div class="tel-seg pm" role="group" aria-label="Plot style">
            <button
              type="button"
              aria-pressed={mode === "overlay"}
              aria-label="Overlay: one axis, each PID as % of its range"
              title="Overlay"
              onclick={() => (mode = "overlay")}
            >
              <svg class="ic" viewBox="0 0 24 24" aria-hidden="true"><path d="M12 3 3 8l9 5 9-5z" /><path d="m3 13 9 5 9-5" /></svg>
            </button>
            <button
              type="button"
              aria-pressed={mode === "lanes"}
              aria-label="Lanes: one lane per PID, real units"
              title="Lanes"
              onclick={() => (mode = "lanes")}
            >
              <svg class="ic" viewBox="0 0 24 24" aria-hidden="true"><path d="M4 6h16M4 12h16M4 18h16" /></svg>
            </button>
          </div>
          <button type="button" class="tbtn" aria-label="Reset view" title="Reset view to 1 min, live" onclick={() => view.reset()}>
            <svg class="ic" viewBox="0 0 24 24" aria-hidden="true"><path d="M3 12a9 9 0 1 0 2.6-6.4L3 8" /><path d="M3 3v5h5" /></svg>
          </button>
          <button
            type="button"
            class="tbtn"
            aria-pressed={fit}
            aria-label="Fit Y to visible data"
            title="Fit Y to the visible data"
            onclick={() => (fit = !fit)}
          >
            <svg class="ic" viewBox="0 0 24 24" aria-hidden="true"><path d="M5 3h14M5 21h14M12 7v10M9 10l3-3 3 3M9 14l3 3 3-3" /></svg>
          </button>
          {#if !fullscreen}
          <button
            type="button"
            class="tbtn"
            aria-pressed={fullscreen}
            aria-label={fullscreen ? "Exit full screen" : "Full screen"}
            title={fullscreen ? "Exit full screen (Esc)" : "Full screen"}
            onclick={onFullscreen}
          >
            <svg class="ic" viewBox="0 0 24 24" aria-hidden="true">
              {#if fullscreen}<path d="M9 4v5H4M15 4v5h5M9 20v-5H4M15 20v-5h5" />{:else}<path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5" />{/if}
            </svg>
          </button>
          {/if}
        </div>
      </div>
      {#if fullscreen}
        <button type="button" class="tbtn exit" bind:this={exitBtn} aria-label="Exit full screen" title="Exit full screen (Esc)" onclick={onFullscreen}>
          <svg class="ic" viewBox="0 0 24 24" aria-hidden="true"><path d="M9 4v5H4M15 4v5h5M9 20v-5H4M15 20v-5h5" /></svg>
        </button>
      {/if}
      {#if !fullscreen}
      <button
        type="button"
        class="tbtn exp"
        aria-expanded={trayOpen}
        aria-label="Plot tools"
        title="Plot tools"
        onclick={() => (trayOpen = !trayOpen)}
      >
        <svg class="ic" viewBox="0 0 24 24" aria-hidden="true"><path d="m15 6-6 6 6 6" /></svg>
      </button>
      {/if}
    </div>
  </div>

  {#if pids.length === 0}
    <p class="empty">Tick a PID in the list to chart it</p>
  {:else}
    <PlotSvg {model} key="plot-{n}" {emptyText} bind:width />
  {/if}
</section>

<style>
  /* Active plot: a 3 px accent bar inset 22 px top and bottom (mockup .mark). */
  .cell[aria-current='true']::before {
    content: '';
    position: absolute;
    left: 0;
    top: 22px;
    bottom: 22px;
    width: 3px;
    border-radius: 0 3px 3px 0;
    background: var(--accent);
  }
  .cell {
    position: relative;
    display: flex;
    flex-direction: column;
    gap: 0;
    min-width: 0;
    padding: 10px 18px 6px 20px;
    border-radius: var(--r-card);
    background: var(--card);
  }
  /* Full screen: a viewport overlay. The overview stays pinned below it (PlotOverview). */
  /* Full screen covers the page and the nav dock. Opaque, so nothing shows through; the pinned overview sits at its bottom. */
  .cell.fs {
    position: fixed;
    inset: 0;
    height: 100dvh;
    z-index: var(--fs-z);
    /* Side insets = the pinned overview's 12 px offset plus its 20 / 18 px card padding, so both align. */
    padding: calc(14px + env(safe-area-inset-top, 0px)) 30px 220px 32px;
    border-radius: 0;
    background: var(--float-solid);
    overflow: hidden;
  }
  /* One row of 44 px: the legend (names, ellipsis) and the tools. */
  .head {
    display: flex;
    flex-wrap: nowrap;
    align-items: center;
    gap: 6px 12px;
    min-height: 44px;
    padding-bottom: 0;
  }
  .csel {
    all: unset;
    box-sizing: border-box;
    display: inline-flex;
    align-items: center;
    min-height: 34px;
    padding: 4px 6px;
    border-radius: 8px;
    cursor: pointer;
    color: var(--muted);
    font-weight: 800;
  }
  .csel[aria-pressed='true'] {
    color: var(--accent-text);
  }
  .csel:focus-visible,
  .tbtn:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .legend {
    display: flex;
    flex: 1 1 0;
    flex-wrap: nowrap;
    align-items: center;
    gap: 4px 14px;
    min-width: 0;
    overflow: hidden;
  }
  /* Dense layouts: one line of compact swatch, name and value; the name ellipses first. */
  /* Dense (four or six plots, half-width cards): the head wraps, and the legend takes its own full-width row with
     each name whole and wrapping onto the next line, never truncated. */
  .head.dense {
    flex-wrap: wrap;
  }
  /* Title row: the name and the tools. The legend follows on its own row. */
  .head.dense .csel {
    order: 1;
  }
  .head.dense .tools {
    order: 2;
    margin-left: auto;
  }
  .head.dense .legend {
    order: 3;
  }
  .legend.dense {
    flex: 1 1 100%;
    flex-wrap: wrap;
    gap: 2px 12px;
    overflow: visible;
    font-size: 0.85rem;
  }
  .legend.dense .lg .nm {
    overflow: visible;
    text-overflow: clip;
    max-width: none;
    white-space: normal;
  }
  .lg {
    display: inline-flex;
    align-items: baseline;
    gap: 8px;
    min-width: 0;
    white-space: nowrap;
  }
  .lg i {
    align-self: center;
    width: 14px;
    height: 4px;
    border-radius: 2px;
    background: var(--sw);
    flex: none;
  }
  .lg .nm {
    overflow: hidden;
    text-overflow: ellipsis;
    font-weight: 700;
  }
  .empty {
    margin: 0;
    padding: 18px 0 14px;
    color: var(--muted);
    font-size: 0.95rem;
    font-weight: 700;
    text-align: center;
  }
  .tools {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: flex-end;
    margin-left: auto;
    min-width: 0;
    max-width: 100%;
  }
  .tray {
    display: grid;
    min-width: 0;
    grid-template-columns: 0fr;
    transition:
      grid-template-columns 0.28s var(--ease-out),
      margin 0.28s var(--ease-out);
  }
  .tray.open {
    grid-template-columns: 1fr;
    margin-right: 6px;
  }
  .trayin {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
    padding: 3px;
    margin: -3px;
    overflow: hidden;
    visibility: hidden;
    transition: visibility 0s 0.28s;
  }
  .tray.open .trayin {
    visibility: visible;
    transition-delay: 0s;
  }
  /* Tool buttons: 40 px square fill (mockup .btn.icon.sm). */
  .tbtn {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    box-sizing: border-box;
    min-width: 40px;
    min-height: 40px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    cursor: pointer;
  }
  .tbtn:hover {
    background: var(--ctl-h2);
  }
  .tbtn[aria-pressed='true'] {
    background: var(--accent);
    color: var(--on-accent);
  }
  .exp {
    margin-left: 0;
  }
  .exit {
    margin-left: 6px;
  }
  .exp .ic {
    transition: transform 0.28s var(--ease-out);
  }
  .exp[aria-expanded='true'] .ic {
    transform: rotate(180deg);
  }
  .ic {
    width: 20px;
    height: 20px;
    fill: none;
    stroke: currentColor;
    stroke-width: 2;
    stroke-linecap: round;
    stroke-linejoin: round;
  }
  @media (pointer: coarse) {
    .tbtn {
      min-width: 44px;
      min-height: 44px;
    }
    .csel {
      min-height: 44px;
    }
  }
  @media (prefers-reduced-motion: reduce) {
    .tray,
    .exp .ic,
    .trayin {
      transition: none;
    }
  }
  /* Phone: 12 px card padding, so the plot takes the card's width. The title row holds the name and the chevron; the
     legend takes a full-width row under it, so PID names are not squeezed. */
  @media (max-width: 859px) {
    .cell {
      padding: 10px 12px 6px 12px;
    }
    .head {
      flex-wrap: wrap;
    }
    .csel {
      order: 1;
    }
    .tools {
      order: 2;
      margin-left: auto;
    }
    .legend {
      order: 3;
      flex: 1 1 100%;
      flex-wrap: wrap;
    }
  }
</style>
