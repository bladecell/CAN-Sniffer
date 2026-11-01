<script lang="ts">
  import { getContext, untrack } from "svelte";
  import { TELEMETRY_VIEW, type TelemetryView } from "$lib/telemetryChart/viewState.svelte";
  import { zoomWin, panWin, keyWin, winStatus, type Win } from "$lib/telemetryChart/window";
  import { agoLabel, clamp, formatValue } from "$lib/telemetryChart/math";
  import { toViewX } from "$lib/telemetryChart/overview";
  import type { PlotModel } from "$lib/telemetryChart/plotModel";

  interface Props {
    model: PlotModel;
    /** Unique per plot: the tooltip shows only where the pointer is. */
    key: string;
    emptyText: string;
    width?: number;
  }

  let { model, key, emptyText, width = $bindable(0) }: Props = $props();
  // role="application" is the WAI pattern for a chart with its own keys (pan, zoom, reset). Svelte does not count it as interactive.
  const view = getContext<TelemetryView>(TELEMETRY_VIEW);
  // Unique per plot instance: the clip path is referenced by id from this svg only.
  const clipId = `plot-clip-${Math.random().toString(36).slice(2, 10)}`;
  const descId = `${clipId}-desc`;

  type Drag =
    | { kind: "pan"; x0: number; start: Win; now: number; pw: number }
    | { kind: "pinch"; d0: number; anchor: number; start: Win; now: number };

  const pointers = new Map<number, number>();
  let drag: Drag | null = null;

  /** Fraction of the plot width under a client x. */
  function fracOf(svg: Element, clientX: number): number {
    const r = svg.getBoundingClientRect();
    return clamp((toViewX(clientX, r.left, r.width, model.W) - model.L) / model.pw, 0, 1);
  }

  function onDown(e: PointerEvent) {
    const svg = e.currentTarget as SVGSVGElement;
    svg.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, e.clientX);
    const now = view.presentAt() ?? 0;
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      drag = { kind: "pinch", d0: Math.abs(a - b) || 1, anchor: fracOf(svg, (a + b) / 2), start: view.win, now };
    } else {
      drag = { kind: "pan", x0: e.clientX, start: view.win, now, pw: model.pw };
    }
  }

  function onMove(e: PointerEvent) {
    const svg = e.currentTarget as SVGSVGElement;
    if (!pointers.has(e.pointerId)) {
      if (e.pointerType === "mouse") view.hover(fracOf(svg, e.clientX), key);
      return;
    }
    pointers.set(e.pointerId, e.clientX);
    if (!drag) return;
    if (drag.kind === "pinch" && pointers.size >= 2) {
      const [a, b] = [...pointers.values()];
      view.set(zoomWin(drag.start, drag.d0 / (Math.abs(a - b) || 1), drag.anchor, drag.now));
    } else if (drag.kind === "pan") {
      view.set(panWin(drag.start, (e.clientX - drag.x0) / drag.pw, drag.now));
    }
  }

  function onUp(e: PointerEvent) {
    pointers.delete(e.pointerId);
    if (pointers.size === 1) {
      const [x] = [...pointers.values()];
      drag = { kind: "pan", x0: x, start: view.win, now: view.presentAt() ?? 0, pw: model.pw };
    } else if (pointers.size === 0) {
      drag = null;
    }
  }

  function onLeave(e: PointerEvent) {
    if (e.pointerType === "mouse" && !pointers.size) view.hover(null, key);
  }

  /** Wheel zooms around the cursor. Attached without passive so the page does not scroll. */
  function wheelZoom(node: SVGSVGElement) {
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const factor = e.deltaY > 0 ? 1.15 : 1 / 1.15;
      const f = fracOf(node, e.clientX);
      view.apply((w, now) => zoomWin(w, factor, f, now));
    };
    node.addEventListener("wheel", onWheel, { passive: false });
    return { destroy: () => node.removeEventListener("wheel", onWheel) };
  }

  /** The cursor line and its time label show on every plot; only the plot under the pointer shows the tooltip. */
  /** Keyboard pan and zoom (see keyWin). The plot is focusable; the focus ring is in the style block. */
  function onKey(e: KeyboardEvent) {
    const next = keyWin(view.win, e.key, view.presentAt() ?? 0);
    if (!next) return;
    e.preventDefault();
    view.set(next);
  }

  /** Screen reader summary: the window, then the cursor values (or window averages without a cursor). Live samples do not change it. */
  const desc = $derived.by(() => {
    if (model.legend.length === 0) return "No PIDs on this plot.";
    const w = view.win;
    const sec = Math.round(w.len);
    const len = sec === 60 ? "1 minute" : sec < 60 ? `${sec} seconds` : `${Math.round((sec / 60) * 10) / 10} minutes`;
    const st = winStatus(w);
    const head = `Window ${len}, ${st === "live" ? "live" : st === "paused" ? "paused" : `ending ${Math.round(w.endAgo)} seconds ago`}.`;
    if (!model.hasData) return `${head} Waiting for data.`;
    const unit = (u: string) => (u ? ` ${u}` : "");
    const rows = model.cursor
      ? model.cursor.rows.map((r) => `${r.name} ${r.text}${unit(r.unit)}`)
      : model.stats.map((s) => `${s.name} average ${formatValue(s.avg)}${unit(s.unit)}`);
    if (rows.length === 0) return `${head} Waiting for data.`;
    return `${head} ${model.cursor ? `At ${model.cursor.label}` : "Window averages"}: ${rows.join(", ")}.`;
  });
  // The live region changes once the window has settled (500 ms after the last move), never per sample.
  let announced = $state("");
  let announceTimer: ReturnType<typeof setTimeout> | undefined;
  let windowSeen = false;
  $effect(() => {
    void view.win;
    if (!windowSeen) {
      windowSeen = true;
      return;
    }
    clearTimeout(announceTimer);
    announceTimer = setTimeout(() => (announced = untrack(() => desc)), 500);
  });
  $effect(() => () => clearTimeout(announceTimer));

  const hover = $derived(model.cursor);
  const showTip = $derived(view.hoverKey === key && model.cursor !== null && model.hasData);
  const labelX = (x: number) => Math.min(Math.max(x, model.L + 40), model.W - model.R - 40);
  const flip = $derived(!!hover && hover.x > model.W * 0.55);
</script>

<div class="ch" bind:clientWidth={width}>
  {#if model.W > 0}
    <!-- svelte-ignore a11y_no_noninteractive_tabindex,a11y_no_noninteractive_element_interactions -->
    <svg
      viewBox="0 0 {model.W} {model.H}"
      width={model.W}
      height={model.H}
      role="application"
      tabindex="0"
      aria-label="Chart of {model.legend.map((l) => l.name).join(', ') || 'no PID'}. Arrow keys pan, plus and minus zoom, Home resets."
      aria-describedby={descId}
      use:wheelZoom
      onkeydown={onKey}
      onpointerdown={onDown}
      onpointermove={onMove}
      onpointerup={onUp}
      onpointercancel={onUp}
      onpointerleave={onLeave}
      ondblclick={() => view.reset()}
    >
      {#each model.axis as a (a.y + a.text)}
        <line class="gl" x1={model.L} x2={model.W - model.R} y1={a.y} y2={a.y} />
        <text x={model.L - 4} y={a.y + 4} text-anchor="end">{a.text}</text>
      {/each}

      {#each model.timeTicks as t (t.x)}
        <line class="gl tick" x1={t.x} x2={t.x} y1={model.T} y2={model.bottom} />
        <text x={t.x} y={model.bottom + 18} text-anchor={t.anchor}>{t.text}</text>
      {/each}

      {#if model.caption}<text class="cap" x={model.L} y={model.T - 10}>{model.caption}</text>{/if}
      {#if model.axisUnit}<text x={model.L - 4} y={model.T - 10} text-anchor="end">{model.axisUnit}</text>{/if}

      {#each model.lanes as lane (lane.id)}
        <line class="gl hr" x1={model.L} x2={model.W - model.R} y1={lane.top - 22} y2={lane.top - 22} />
        <rect x={model.L} y={lane.top - 14} width="14" height="4" rx="2" fill={lane.color} />
        <text class="lane" x={model.L + 22} y={lane.top - 8}>{lane.name}</text>
        {#each lane.ticks as k (k.y)}
          <line class="gl" x1={model.L} x2={model.W - model.R} y1={k.y} y2={k.y} />
          <text x={model.L - 4} y={k.y + 4} text-anchor="end">{k.text}</text>
        {/each}
        <!-- Readout in the lane's header band, right-aligned to the plot edge: value, then its unit small beside it. -->
        <text class="unit" x={model.W - model.R} y={lane.top - 6} text-anchor="end">{lane.unit}</text>
        <text class="big" x={model.W - model.R - lane.unit.length * 6.5 - 6} y={lane.top - 4} text-anchor="end">{lane.big}</text>
      {/each}

      <defs>
        <clipPath id={clipId}>
          <rect x={model.plot.x} y={model.plot.y} width={model.plot.w} height={model.plot.h} />
        </clipPath>
      </defs>
      <g clip-path="url(#{clipId})">
        {#each model.lines as line (line.id)}
          {#if line.area}<path class="ar" d={line.area} fill={line.color} />{/if}
          <path class="ln" d={line.d} stroke={line.color} />
        {/each}
      </g>


      {#if hover}
        <line class="cur" x1={hover.x} x2={hover.x} y1={model.T} y2={model.bottom} />
        {#each hover.rows as r (r.id)}
          {#if r.y !== null}<circle class="dt" cx={r.x} cy={r.y} r="5" fill={r.color} />{/if}
        {/each}
        <g class="tb">
          <rect x={labelX(hover.x) - 40} y={hover.chipY} width="80" height="20" rx="10" />
          <text x={labelX(hover.x)} y={hover.chipY + 14} text-anchor="middle">{hover.label}</text>
        </g>
      {/if}

      {#if !model.hasData}
        <text class="empty" x={model.W / 2} y={model.H / 2} text-anchor="middle">{emptyText}</text>
      {/if}
    </svg>

    <p id={descId} class="sr">{desc}</p>
    <p class="sr" aria-live="polite">{announced}</p>

    {#if showTip && hover}
      <div class="tip" style:left="{flip ? hover.x - 174 : hover.x + 14}px" style:top="{model.T + 8}px">
        <div class="t">{hover.label}</div>
        {#each hover.rows as r (r.id)}
          <div class="r">
            <i style:--sw={r.color}></i>{r.name}<b>{r.text}{r.unit ? ` ${r.unit}` : ""}</b>
          </div>
        {/each}
      </div>
    {/if}
  {/if}
</div>

<style>
  .ch {
    position: relative;
    min-width: 0;
  }
  /* Screen reader text: in the accessibility tree, not on screen. */
  .sr {
    position: absolute;
    width: 1px;
    height: 1px;
    margin: -1px;
    padding: 0;
    overflow: hidden;
    clip-path: inset(50%);
    white-space: nowrap;
    border: 0;
  }
  svg {
    display: block;
    width: 100%;
    height: auto;
    overflow: visible;
    touch-action: pan-y;
    cursor: crosshair;
    user-select: none;
    border-radius: 8px;
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
  text.unit {
    font-size: 10.5px;
    font-weight: 600;
    fill: var(--muted);
  }
  text.big {
    fill: var(--text);
    font-weight: 800;
    font-size: 20px;
  }
  text.lane {
    fill: var(--text);
    font-weight: 800;
    font-size: 13px;
  }
  text.empty {
    font-size: 14px;
    font-weight: 700;
  }
  .gl {
    stroke: var(--grid);
    stroke-width: 1;
    opacity: 0.6;
    shape-rendering: crispEdges;
  }
  .gl.hr {
    stroke: var(--hair);
  }
  .gl.tick {
    opacity: 0.55;
  }
  .ln {
    fill: none;
    stroke-width: 2;
    stroke-linejoin: round;
    stroke-linecap: round;
  }
  .ar {
    stroke: none;
    opacity: 0.14;
  }
  .cur {
    stroke: var(--text);
    stroke-width: 1;
    opacity: 0.55;
  }
  .dt {
    stroke: var(--float-solid);
    stroke-width: 2;
  }
  .tb rect {
    fill: var(--float-solid);
  }
  .tb text {
    fill: var(--text);
    font-weight: 800;
  }
  .tip {
    position: absolute;
    z-index: 5;
    pointer-events: none;
    min-width: 150px;
    padding: 10px 12px;
    border-radius: 16px;
    background: var(--float);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
    box-shadow: var(--shadow-float);
    font-size: 0.9rem;
  }
  @supports not (backdrop-filter: blur(1px)) {
    .tip {
      background: var(--float-solid);
    }
  }
  .tip .t {
    color: var(--muted);
    font-weight: 700;
    font-size: 0.8rem;
    margin-bottom: 4px;
  }
  .tip .r {
    display: flex;
    align-items: center;
    gap: 8px;
    line-height: 1.5;
  }
  .tip .r i {
    width: 14px;
    height: 4px;
    border-radius: 2px;
    background: var(--sw);
    flex: none;
  }
  .tip .r b {
    margin-left: auto;
    padding-left: 14px;
    font-variant-numeric: tabular-nums;
  }
</style>
