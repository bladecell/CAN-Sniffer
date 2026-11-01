<script lang="ts">
  import "$lib/components/telemetry/telemetry.css";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { telemetryStore } from "$lib/telemetryStore.svelte";
  import { untrack, setContext } from "svelte";
  import { safeHexColor } from "$lib/color";
  import Icon from "$lib/Icon.svelte";
  import Sheet from "$lib/components/Sheet.svelte";
  import PidSidebar from "$lib/components/telemetry/PidSidebar.svelte";
  import PidTabs, { type PidPanel } from "$lib/components/telemetry/PidTabs.svelte";
  import PidEditorForm from "$lib/components/telemetry/PidEditorForm.svelte";
  import EditorTabs from "$lib/components/telemetry/EditorTabs.svelte";
  import PidEditorActions from "$lib/components/telemetry/editor/PidEditorActions.svelte";
  import PlotCell from "$lib/components/telemetry/PlotCell.svelte";
  import PlotOverview from "$lib/components/telemetry/PlotOverview.svelte";
  import { chartPidsFor, chipAction } from "$lib/telemetryView";
  import { TELEMETRY_VIEW, TelemetryView } from "$lib/telemetryChart/viewState.svelte";
  import { PLOT_COUNTS, gridColumns, migratePlotCount, type PlotCount } from "$lib/telemetryChart/layout";
  import { canToggleTicks, chartPidsOf, deselectOnEscape, deselectOnPointer } from "$lib/telemetryChart/selection";
  import { noPlotHint } from "$lib/telemetryChart/noPlotHint";
  import { statsColumnPx } from "$lib/telemetryChart/statsColumns";
  import { freezeWin, resumeWin } from "$lib/telemetryChart/window";
  import { formatValue } from "$lib/telemetryChart/math";
  import type { PlotModel } from "$lib/telemetryChart/plotModel";

  // Ref-counted: the stream stays on while any view needs it.
  $effect(() => connectionStore.acquireStream());

  // One window and one clock for every plot on the page.
  const view = new TelemetryView();
  setContext(TELEMETRY_VIEW, view);
  $effect(() => view.start());

  let pidPanel = $state<PidPanel>("pids");
  const pidPanelTitle: Record<PidPanel, string> = { pids: "PIDs", changes: "Changes", sd: "SD card", settings: "Settings" };
  let pidSheetOpen = $state(false);
  let editorOpen = $state(false);

  let innerH = $state(800);
  /** PID sheet bounds: the head block (~166 px) plus one row; the nav dock and safe area stay on screen. */
  const PID_SHEET_MIN_PX = 440;
  const PID_SHEET_NAV_PX = 110;
  /** Index of the plot in full screen, or -1. */
  let fsIndex = $state(-1);
  let activeStats = $state<PlotModel["stats"]>([]);
  let statsTableW = $state(0);

  const PLOT_HEIGHT: Record<PlotCount, number> = { 1: 370, 2: 265, 4: 235, 6: 200 };
  const LANE_HEIGHT: Record<PlotCount, number> = { 1: 110, 2: 90, 4: 72, 6: 72 };
  const plotHeight = $derived(fsIndex >= 0 ? Math.max(240, innerH - 330) : PLOT_HEIGHT[telemetryStore.plotCount]);
  const laneHeight = $derived(
    fsIndex >= 0
      ? Math.max(90, (innerH - 366) / Math.max(1, telemetryStore.chartPids[fsIndex]?.length ?? 1))
      : LANE_HEIGHT[telemetryStore.plotCount],
  );

  $effect(() => {
    document.body.classList.toggle("tel-fs", fsIndex >= 0);
    return () => document.body.classList.remove("tel-fs");
  });

  let selectedRow = $derived(
    telemetryStore.activeIndex >= 0 && telemetryStore.activeIndex < telemetryStore.local_piddef.length
      ? telemetryStore.local_piddef[telemetryStore.activeIndex]
      : null,
  );

  // Ticked PIDs (or the selected row when none is ticked) are what the active plot shows.
  $effect(() => {
    const ticked = telemetryStore.local_piddef.filter((r) => r.selected).map((row) => parseInt(row.pid, 16));
    const currentSelectedPids = chartPidsFor(ticked, selectedRow?.pid ? parseInt(selectedRow.pid, 16) : null);

    untrack(() => {
      if (telemetryStore.activeChartIndex >= 0 && telemetryStore.activeChartIndex < telemetryStore.plotCount) {
        telemetryStore.chartPids[telemetryStore.activeChartIndex] = currentSelectedPids;
      }
    });
  });

  function setLayout(n: number) {
    const total = migratePlotCount(n, 1);
    fsIndex = -1;
    telemetryStore.plotCount = total;
    const newChartPids = [...telemetryStore.chartPids];
    while (newChartPids.length < total) newChartPids.push([]);
    telemetryStore.chartPids = newChartPids;
    if (telemetryStore.activeChartIndex >= total) setActiveChart(0);
  }

  function toggleFullscreen(index: number) {
    setActiveChart(index);
    fsIndex = fsIndex === index ? -1 : index;
  }

  /** Esc leaves full screen, unless a dialog or popover is open above it (it closes first). */
  function onWindowKey(e: KeyboardEvent) {
    if (e.key !== "Escape") return;
    if (document.querySelector("dialog[open]") || document.querySelector(":popover-open")) return;
    if (fsIndex >= 0) return (fsIndex = -1);
    const focusInPlot = document.activeElement?.closest(".cell") != null;
    if (deselectOnEscape(telemetryStore.activeChartIndex, focusInPlot) !== telemetryStore.activeChartIndex) {
      setActiveChart(-1);
    }
  }

  /** Stats come from the active plot at every tick; only a change in content replaces the table rows. */
  function setStats(rows: PlotModel["stats"]) {
    if (JSON.stringify(rows) !== JSON.stringify(activeStats)) activeStats = rows;
  }

  function togglePause() {
    if (view.status === "live") view.apply((w, now) => freezeWin(w, now));
    else view.set(resumeWin(view.win));
  }

  const status = $derived(view.status);

  // -1 is no plot selected: the list shows no ticks, and each plot keeps its own PIDs (chartPids) for re-selection.
  function setActiveChart(index: number) {
    // Pointer-down fires on every drag and zoom of the active chart; only a switch of chart rewrites the selection.
    if (index === telemetryStore.activeChartIndex) return;
    telemetryStore.activeChartIndex = index;
    const targetPids = chartPidsOf(telemetryStore.chartPids, index);
    for (const item of telemetryStore.local_piddef) {
      item.selected = targetPids.includes(parseInt(item.pid, 16));
    }
  }

  /** Clicks outside the plots and the related surfaces clear the selection (the page background, gaps, the title). */
  function isRelatedTarget(t: Element): boolean {
    if (t.closest("dialog, [popover], .side, .chips, .ov, .stats, .cell")) return true;
    return t.closest(".bar") !== null && t.closest("button, a, input, select, [role='group']") !== null;
  }

  function onWindowPointer(e: PointerEvent) {
    const t = e.target instanceof Element ? e.target : null;
    if (!t || telemetryStore.activeChartIndex < 0) return;
    const next = deselectOnPointer(telemetryStore.activeChartIndex, isRelatedTarget(t));
    if (next !== telemetryStore.activeChartIndex) setActiveChart(next);
  }

  function newPid() {
    telemetryStore.activeIndex = -1;
    editorOpen = true;
  }

  function editPid(index: number) {
    telemetryStore.activeIndex = index;
    editorOpen = true;
  }

  // Phone chips: the loaded PIDs, pressed when they are on the active chart.
  const activeChartPids = $derived(chartPidsOf(telemetryStore.chartPids, telemetryStore.activeChartIndex));

  function toggleChip(pid: number, pressed: boolean) {
    if (!canToggleTicks(telemetryStore.activeChartIndex)) return noPlotHint();
    const row = telemetryStore.local_piddef.find((r) => parseInt(r.pid, 16) === pid);
    if (!row) return;
    const action = chipAction(pressed, row.selected);
    if (action === "tick") row.selected = true;
    else if (action === "untick") row.selected = false;
    else telemetryStore.activeIndex = -1;
  }

  // Long-press on a phone chip opens that PID's editor; the click that follows is swallowed.
  let pressTimer: ReturnType<typeof setTimeout> | undefined;
  let pressFired = false;
  let pressX = 0;
  let pressY = 0;
  function pressStart(e: PointerEvent, row: (typeof telemetryStore.local_piddef)[number]) {
    pressFired = false;
    pressX = e.clientX;
    pressY = e.clientY;
    clearTimeout(pressTimer);
    pressTimer = setTimeout(() => {
      pressFired = true;
      editPid(telemetryStore.local_piddef.indexOf(row));
    }, 500);
  }
  function pressMove(e: PointerEvent) {
    if (Math.hypot(e.clientX - pressX, e.clientY - pressY) > 8) clearTimeout(pressTimer);
  }
  function pressEnd() {
    clearTimeout(pressTimer);
  }
  function chipClick(pid: number, on: boolean) {
    if (pressFired) pressFired = false;
    else toggleChip(pid, on);
  }

  const editorTitle = $derived(
    selectedRow && telemetryStore.activeIndex >= 0 ? `Edit ${selectedRow.name} (${selectedRow.pid})` : "New PID",
  );
  const count = $derived(telemetryStore.plotCount);
  const activePids = $derived(chartPidsOf(telemetryStore.chartPids, telemetryStore.activeChartIndex));
</script>

<svelte:window bind:innerHeight={innerH} onkeydown={onWindowKey} onpointerdown={onWindowPointer} />

<section class="tel">
  <div class="wb">
    <aside class="side" aria-label="PID list">
      <PidSidebar variant="rail" onNew={newPid} onEdit={editPid} />
    </aside>

    <div class="main">
      <div class="card bar">
        <h1 tabindex="-1">Telemetry</h1>

        <div class="tel-seg layouts" role="group" aria-label="Number of plots">
          {#each PLOT_COUNTS as n (n)}
            <button type="button" aria-pressed={count === n} aria-label="{n} {n === 1 ? 'plot' : 'plots'}" onclick={() => setLayout(n)}>{n}</button>
          {/each}
        </div>

        <button type="button" class="tel-btn" class:pri={status !== "live"} onclick={togglePause} aria-label={status === "live" ? "Pause" : "Back to live"}>
          <svg class="ic" viewBox="0 0 24 24" aria-hidden="true">
            {#if status === "live"}<path d="M8 5v14M16 5v14" />{:else}<path d="M5 5v14l10-7zM19 5v14" />{/if}
          </svg>
          {status === "live" ? "Pause" : "Back to live"}
        </button>

        <button
          type="button"
          class="tel-btn"
          aria-disabled={telemetryStore.activeIndex < 0}
          title={telemetryStore.activeIndex < 0 ? "Select a PID to edit it. New PID is in the list." : undefined}
          onclick={() => telemetryStore.activeIndex >= 0 && (editorOpen = true)}
        >
          <Icon name="edit" size={18} />Edit PID
        </button>

        <button type="button" class="tel-btn phone-only" onclick={() => (pidSheetOpen = true)} aria-haspopup="dialog">
          <Icon name="list" size={18} />PIDs
        </button>
      </div>

      <div class="chips" role="group" aria-label="PIDs on the active plot">
        {#each telemetryStore.local_piddef.filter((r) => r.loaded) as row (row)}
          {@const pid = parseInt(row.pid, 16)}
          {@const on = activeChartPids.includes(pid)}
          <button
            type="button"
            class="tel-pchip"
            aria-pressed={on}
            aria-keyshortcuts="Shift+F10"
            title="Tap to show or hide. Hold to edit."
            onclick={() => chipClick(pid, on)}
            onpointerdown={(e) => pressStart(e, row)}
            onpointermove={pressMove}
            onpointerup={pressEnd}
            onpointercancel={pressEnd}
            onpointerleave={pressEnd}
            oncontextmenu={(e) => e.preventDefault()}
            onkeydown={(e) => (e.key === "ContextMenu" || (e.shiftKey && e.key === "F10")) && (e.preventDefault(), editPid(telemetryStore.local_piddef.indexOf(row)))}
          >
            <i style="--sw: {safeHexColor('#' + Number(row.def.color).toString(16).padStart(6, '0'))}"></i>{row.name}
          </button>
        {/each}
      </div>

      <div class="cgrid" style="--cols: {gridColumns(count)}">
        {#each Array(count) as _, i (i)}
          <PlotCell
            n={i + 1}
            pids={telemetryStore.chartPids[i] || []}
            active={telemetryStore.activeChartIndex === i}
            fullscreen={fsIndex === i}
            dense={count >= 4}
            {plotHeight}
            {laneHeight}
            onActivate={() => setActiveChart(i)}
            onFullscreen={() => toggleFullscreen(i)}
            onStats={setStats}
          />
        {/each}
      </div>

      <PlotOverview pids={activePids} pinned={fsIndex >= 0} />

      <section class="card stats" aria-label="Statistics">
        <div class="stats-wrap" bind:clientWidth={statsTableW} style="--stat-col: {statsColumnPx(statsTableW)}px">
        <table>
          <caption class="sr">Statistics for the active plot, visible window</caption>
          <colgroup>
            <col />
            <col class="n" />
            <col class="n" />
            <col class="n" />
            <col class="n" />
          </colgroup>
          <thead>
            <tr><th>Active plot</th><th class="n">Now</th><th class="n">Min</th><th class="n">Avg</th><th class="n">Max</th></tr>
          </thead>
          <tbody>
            {#each activeStats as r (r.id)}
              <tr>
                <td><span class="sw"><i style="--sw: {r.color}"></i><span>{r.name}</span></span></td>
                <td class="num"><span class="v">{r.now === null ? "N/A" : formatValue(r.now)}</span>{#if r.now !== null && r.unit}<span class="u">{r.unit}</span>{/if}</td>
                <td class="num"><span class="v">{formatValue(r.min)}</span>{#if r.unit}<span class="u">{r.unit}</span>{/if}</td>
                <td class="num"><span class="v">{formatValue(r.avg)}</span>{#if r.unit}<span class="u">{r.unit}</span>{/if}</td>
                <td class="num"><span class="v">{formatValue(r.max)}</span>{#if r.unit}<span class="u">{r.unit}</span>{/if}</td>
              </tr>
            {:else}
              <tr><td colspan="4" class="none">{telemetryStore.activeChartIndex < 0 ? "Select a plot to see its stats" : activePids.length === 0 ? "No PIDs on this plot" : "No samples in the visible window"}</td></tr>
            {/each}
          </tbody>
        </table>
        </div>
      </section>
    </div>
  </div>
</section>

<!-- Resizable: the grab bar drags or takes Arrow keys. Min is the head (title, tabs, strip) plus one row; max is the viewport less the nav dock. -->
<Sheet
  open={pidSheetOpen}
  title={pidPanelTitle[pidPanel]}
  onClose={() => (pidSheetOpen = false)}
  scrolls={false}
  resizable
  focusTitle
  minHeight={PID_SHEET_MIN_PX}
  maxHeight={Math.min(innerH - PID_SHEET_NAV_PX, Math.round(innerH * 0.85))}
>
  {#snippet head()}
    <PidTabs variant="bar" bind:panel={pidPanel} />
  {/snippet}
  <PidSidebar variant="bar" bind:panel={pidPanel} onNew={newPid} onEdit={editPid} />
</Sheet>

<Sheet open={editorOpen} title={editorTitle} onClose={() => (editorOpen = false)} resizable>
  {#snippet head()}
    <EditorTabs />
  {/snippet}
  <PidEditorForm onDone={() => (editorOpen = false)} />
  {#snippet footer()}
    <PidEditorActions formId="pid-editor" />
  {/snippet}
</Sheet>

<style>
  .tel {
    --gap: 20px;
    --stick: 104px;
    /* Full screen layers (plot and overview) sit above the nav dock (z 1000). */
    --fs-z: 1100;
    display: grid;
    gap: var(--gap);
    min-width: 0;
  }
  .wb {
    display: grid;
    gap: var(--gap);
    grid-template-columns: minmax(0, 1fr);
    align-items: start;
  }
  .main {
    display: grid;
    gap: var(--gap);
    min-width: 0;
  }
  .side {
    display: none;
  }
  @media (min-width: 860px) {
    .wb {
      grid-template-columns: clamp(380px, 28vw, 400px) minmax(0, 1fr);
    }
    .side {
      display: flex;
      flex-direction: column;
      /* Header, three rows and the action bar. Below this the rail takes its full height and the page scrolls. */
      min-height: 460px;
      padding: 0;
      box-sizing: border-box;
    }
  }
  /* Sticky only where the rail fits below the page header (the sticky top is 104 px, so 100dvh - 200 px stays on
     screen and clear of the nav dock). In a short viewport the rail scrolls with the page. */
  @media (min-width: 860px) and (min-height: 760px) {
    .side {
      position: sticky;
      top: var(--stick);
      max-height: calc(100dvh - 200px);
    }
  }
  @media (max-width: 859px) {
    .tel {
      --gap: 14px;
    }
  }

  .bar {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 10px var(--gap);
    padding: var(--pad-card);
    border-radius: var(--r-card);
  }
  .bar h1 {
    margin: 0 auto 0 0;
    font-size: 2.1rem;
    font-weight: 800;
    line-height: 1.15;
    letter-spacing: -0.01em;
  }
  h1:focus {
    outline: none;
  }
  @media (max-width: 859px) {
    .bar h1 {
      font-size: 1.7rem;
    }
  }
  .layouts {
    width: max-content;
  }
  /* Header buttons: 46 px tall, 18 px side padding (mockup .btn). */
  .bar :global(.tel-btn) {
    min-height: 46px;
    padding: 0 18px;
  }
  .layouts button {
    min-width: 44px;
  }
  .ic {
    width: 20px;
    height: 20px;
    fill: none;
    stroke: currentColor;
    stroke-width: 2;
    stroke-linecap: round;
    stroke-linejoin: round;
    flex: none;
  }
  .phone-only {
    display: inline-flex;
  }
  @media (min-width: 860px) {
    .phone-only {
      display: none;
    }
  }

  :global(body.tel-fs) {
    overflow: hidden;
  }
  .chips {
    display: flex;
    gap: 8px;
    overflow-x: auto;
    overscroll-behavior-x: contain;
    touch-action: pan-x;
    -webkit-overflow-scrolling: touch;
    padding: 2px 2px 6px;
    scrollbar-width: none;
    /* Fade the right edge: there is more to scroll to. */
    mask-image: linear-gradient(to right, #000 calc(100% - 28px), transparent);
  }
  .chips::-webkit-scrollbar {
    display: none;
  }
  @media (min-width: 860px) {
    .chips {
      display: none;
    }
  }
  .tel-pchip {
    position: relative;
    user-select: none;
    -webkit-touch-callout: none;
    flex: none;
    display: inline-flex;
    align-items: center;
    gap: 8px;
    min-height: var(--ctl-h);
    margin: 0;
    padding: 0 14px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-sub);
    font-weight: 700;
    white-space: nowrap;
    cursor: pointer;
  }
  .tel-pchip i {
    display: inline-block;
    width: 14px;
    height: 4px;
    border-radius: 2px;
    background: var(--sw);
  }
  .tel-pchip::after {
    content: '';
    position: absolute;
    left: 18%;
    right: 18%;
    bottom: 3px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
    transform: scaleX(0);
    transition: transform 0.2s var(--ease-out);
  }
  .tel-pchip[aria-pressed='true'] {
    color: var(--ctl-sel);
    font-weight: 800;
  }
  .tel-pchip[aria-pressed='true']::after {
    transform: scaleX(1);
  }
  @media (pointer: coarse) {
    .tel-pchip {
      min-height: 44px;
    }
  }

  .cgrid {
    display: grid;
    gap: var(--gap);
    grid-template-columns: repeat(var(--cols, 1), minmax(0, 1fr));
    min-width: 0;
  }
  @media (max-width: 700px) {
    .cgrid {
      grid-template-columns: minmax(0, 1fr);
    }
  }

  .stats {
    position: relative;
    padding: 8px var(--pad-card);
    min-width: 0;
  }
  /* Fixed layout, widths from the table itself (statsColumns.ts): the name column takes what is left and ellipses on
     one line; each value column ends at a fixed edge, with its unit in a fixed slot after it. Nothing reflows as values
     change, and all three value columns stay on every width. */
  .stats-wrap {
    min-width: 0;
  }
  .stats table {
    width: 100%;
    table-layout: fixed;
    border-collapse: collapse;
    font-variant-numeric: tabular-nums;
    background: transparent;
  }
  .stats thead,
  .stats tbody,
  .stats th,
  .stats td {
    background: transparent;
  }
  .sr {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip: rect(0 0 0 0);
    white-space: nowrap;
  }
  .stats th {
    padding: 10px 6px 10px 0;
    text-align: left;
    color: var(--muted);
    font-weight: 700;
    font-size: 0.85rem;
    white-space: nowrap;
  }
  .stats th.n {
    text-align: right;
    padding-right: 6px;
  }
  .stats td {
    padding: 8px 6px 8px 0;
    text-align: right;
    font-weight: 700;
    border-top: 1px solid var(--hair);
  }
  .stats td:first-child {
    font-weight: 800;
    text-align: left;
    min-width: 0;
    white-space: normal;
  }
  .stats col.n {
    width: var(--stat-col, 76px);
  }
  .stats .sw {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .stats .sw i {
    width: 14px;
    height: 4px;
    border-radius: 2px;
    background: var(--sw);
    flex: none;
  }
  /* Names are never cut off: they wrap at a space instead. */
  .stats .sw span {
    min-width: 0;
    overflow-wrap: normal;
  }
  /* The value and its unit on one line, the unit after the number. Nothing wraps inside a value. */
  .stats td {
    white-space: nowrap;
  }
  .stats td .u {
    margin-left: 0.2em;
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
  }
  /* Phone: the name takes a full row, and Now, Min, Avg and Max form a fixed four-column grid under it, so no value
     shares a column with a long name. Desktop keeps the table. */
  @media (max-width: 859px) {
    .stats table {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
    }
    .stats col,
    .stats colgroup,
    .stats thead,
    .stats tbody,
    .stats tr {
      display: contents;
    }
    .stats thead th:first-child {
      display: none;
    }
    .stats th.n {
      padding-right: 0;
    }
    .stats td {
      padding: 0 0 6px;
      font-size: 0.95rem;
    }
    .stats td:first-child {
      grid-column: 1 / -1;
      padding: 8px 0 2px;
    }
    .stats td:not(:first-child) {
      border-top: 0;
    }
    .stats td .u {
      font-size: 0.66rem;
    }
  }
  .stats td.none {
    text-align: left;
    color: var(--muted);
    font-weight: 600;
  }
</style>
