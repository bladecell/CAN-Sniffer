<script lang="ts">
  import { getContext } from "svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { TELEMETRY_VIEW, type TelemetryView } from "$lib/telemetryChart/viewState.svelte";
  import { createSeriesFeed } from "$lib/telemetryChart/feed.svelte";
  import { OV_L, OV_R, overviewModel, overviewDrag, grabAt, jumpTo, ovAgo, toViewX, type DrawnRect, type Grab } from "$lib/telemetryChart/overview";
  import { SPAN_SEC, WINDOW_LENS, shiftWin, zoomWin, type Win } from "$lib/telemetryChart/window";

  interface Props {
    pids: number[];
    pinned: boolean;
  }

  let { pids, pinned }: Props = $props();
  const view = getContext<TelemetryView>(TELEMETRY_VIEW);

  let width = $state(0);
  const feed = createSeriesFeed(
    () => pids,
    (t) => view.observe(t),
  );
  const defs = $derived(new Map(vehicleStore.pidDefinitions.map((d) => [d.pid, d])));

  const model = $derived.by(() => {
    void feed.version;
    const now = view.presentAt() ?? 0;
    const fromT = now - SPAN_SEC * 1000 - 1000;
    const series = pids.map((pid) => {
      const def = defs.get(pid);
      const s = feed.read(pid, fromT);
      const range: [number, number] | null =
        def && Number.isFinite(def.minValue) && Number.isFinite(def.maxValue) ? [def.minValue, def.maxValue] : null;
      return { color: safeHexColor(def?.color), range, t: s.t, v: s.v };
    });
    return overviewModel({ width, height: 84, now, win: view.win, series });
  });

  let drag: { grab: Exclude<Grab, "jump">; clientX0: number; rect: DrawnRect; start: Win; now: number } | null = null;

  /** The svg holds pointer capture, so currentTarget is the svg. */
  const drawnRect = (e: PointerEvent): DrawnRect => {
    const r = (e.currentTarget as SVGSVGElement).getBoundingClientRect();
    return { left: r.left, width: r.width };
  };

  function onDown(e: PointerEvent) {
    const rect = drawnRect(e);
    const x = toViewX(e.clientX, rect.left, rect.width, width);
    const now = view.presentAt() ?? 0;
    const grab = grabAt(x, view.win, width);
    (e.currentTarget as SVGSVGElement).setPointerCapture(e.pointerId);
    if (grab === "jump") {
      view.apply((w, t) => jumpTo(w, ovAgo(x, width), t));
      return;
    }
    drag = { grab, clientX0: e.clientX, rect, start: view.win, now };
  }

  function onMove(e: PointerEvent) {
    if (!drag) return;
    const { grab, clientX0, rect, start, now } = drag;
    view.set(overviewDrag(start, grab, rect, clientX0, e.clientX, width, now));
  }

  function onUp() {
    drag = null;
  }

  /** Keyboard: arrows move the frame, +/- zoom about its centre. */
  function onKey(e: KeyboardEvent) {
    const step = e.shiftKey ? 30 : 10;
    if (e.key === "ArrowLeft") view.apply((w, now) => shiftWin(w, 0, step, now));
    else if (e.key === "ArrowRight") view.apply((w, now) => shiftWin(w, 0, -step, now));
    else if (e.key === "+" || e.key === "=") view.apply((w, now) => zoomWin(w, 1 / 1.5, 0.5, now));
    else if (e.key === "-") view.apply((w, now) => zoomWin(w, 1.5, 0.5, now));
    else return;
    e.preventDefault();
  }

  const frameLabel = $derived(
    view.win.endAgo > 0.5 ? `${Math.round(view.win.endAgo)} seconds ago` : "now",
  );
</script>

<section class="ov card" class:pinned aria-label="Time window">
  <div class="cap">
    <span>Shared by all plots. Drag the frame to move it, its edges to zoom.</span>
    <div class="tel-seg sm" role="group" aria-label="Visible window">
      {#each WINDOW_LENS as s (s)}
        <button
          type="button"
          aria-pressed={view.win.len === s}
          onclick={() => view.apply((w, now) => shiftWin(w, s - w.len, 0, now))}
        >
          {s === 30 ? "30 s" : s === 60 ? "1 min" : "5 min"}
        </button>
      {/each}
    </div>
  </div>

  <!-- Measured without padding: the viewBox width equals the drawn width, so strip units are CSS px. -->
  <div class="track" bind:clientWidth={width}>
    {#if width > 0}
    <svg
      role="slider"
      tabindex="0"
      aria-label="Visible time window"
      aria-valuemin={0}
      aria-valuemax={SPAN_SEC}
      aria-valuenow={Math.round(view.win.endAgo)}
      aria-valuetext="{view.win.len} seconds, ending {frameLabel}"
      viewBox="0 0 {width} {model.bottom + 22}"
      width={width}
      height={model.bottom + 22}
      onpointerdown={onDown}
      onpointermove={onMove}
      onpointerup={onUp}
      onpointercancel={onUp}
      onkeydown={onKey}
    >
      {#each model.lines as line, i (i)}
        <path class="ln" d={line.d} stroke={line.color} />
      {/each}
      <rect class="dim" x={OV_L} y={model.top} width={Math.max(0, model.frame.x0 - OV_L)} height={model.bottom - model.top} />
      <rect class="dim" x={model.frame.x1} y={model.top} width={Math.max(0, width - OV_R - model.frame.x1)} height={model.bottom - model.top} />
      <rect class="frame" x={model.frame.x0} y={model.top} width={Math.max(1, model.frame.x1 - model.frame.x0)} height={model.bottom - model.top} rx="8" />
      <rect class="handle" x={model.frame.x0 - 3} y={model.top + 10} width="6" height={model.bottom - model.top - 20} rx="3" />
      <rect class="handle" x={model.frame.x1 - 3} y={model.top + 10} width="6" height={model.bottom - model.top - 20} rx="3" />
      {#each model.ticks as t (t.x)}
        <text x={t.x} y={model.bottom + 16} text-anchor={t.x > width - 30 ? "end" : t.x < 60 ? "start" : "middle"}>{t.text}</text>
      {/each}
    </svg>
    {/if}
  </div>
</section>

<style>
  /* In flow, directly under the plot grid: a card like the plot cards (same width and insets). It does not stick
     or float. Only full screen pins it (below). */
  .ov {
    display: grid;
    gap: 10px;
    min-width: 0;
    /* Same side insets as the plot cells (20 left, 18 right), so the strip's plot area lines up with the plots. */
    padding: 16px 18px 12px 20px;
    border-radius: var(--r-card);
    background: var(--card);
  }
  /* Phone: the same 12 px side padding as the plot cards, so the strip's plot area lines up with the plots. */
  @media (max-width: 859px) {
    .ov {
      padding: 12px 12px 10px 12px;
    }
  }
  /* Full screen: pinned above the nav dock and the page, with the safe-area inset at the bottom. */
  .ov.pinned {
    position: fixed;
    left: 12px;
    right: 12px;
    bottom: calc(12px + env(safe-area-inset-bottom, 0px));
    z-index: var(--fs-z);
    background: var(--float-solid);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
    box-shadow: var(--shadow-float);
  }
  .cap {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
    color: var(--muted);
    font-size: 0.85rem;
    font-weight: 600;
  }
  @media (pointer: coarse) {
    .tel-seg button {
      min-height: 44px;
    }
  }
  .track {
    min-width: 0;
  }
  svg {
    display: block;
    width: 100%;
    height: auto;
    touch-action: none;
    cursor: grab;
    user-select: none;
    -webkit-user-select: none;
  }
  svg:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  text {
    fill: var(--muted);
    font: 600 11.5px var(--font);
    font-variant-numeric: tabular-nums;
  }
  .ln {
    fill: none;
    stroke-width: 1.5;
    stroke-linejoin: round;
    stroke-linecap: round;
    opacity: 0.85;
  }
  .dim {
    fill: var(--bg);
    opacity: 0.55;
  }
  .frame {
    fill: none;
    stroke: var(--accent);
    stroke-width: 3;
  }
  .handle {
    fill: var(--accent);
  }
</style>
