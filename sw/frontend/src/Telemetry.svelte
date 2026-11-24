<script lang="ts">
  import "$lib/components/telemetry/telemetry.css";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { telemetryStore } from "$lib/telemetryStore.svelte";
  import { untrack, setContext } from "svelte";
  import { MediaQuery } from "svelte/reactivity";
  import { safeHexColor } from "$lib/color";
    import Sheet from "$lib/components/Sheet.svelte";
  import PidSidebar from "$lib/components/telemetry/PidSidebar.svelte";
  import PidTabs, { type PhoneTab, type PidPanel } from "$lib/components/telemetry/PidTabs.svelte";
  import PidEditorForm from "$lib/components/telemetry/PidEditorForm.svelte";
  import EditorTabs from "$lib/components/telemetry/EditorTabs.svelte";
  import PidEditorActions from "$lib/components/telemetry/editor/PidEditorActions.svelte";
  import PlotCell from "$lib/components/telemetry/PlotCell.svelte";
  import PlotOverview from "$lib/components/telemetry/PlotOverview.svelte";
  import { pressRow, ticksFor } from "$lib/telemetryView";
  import { TELEMETRY_VIEW, TelemetryView } from "$lib/telemetryChart/viewState.svelte";
  import { PLOT_COUNTS, gridColumns, migratePlotCount, type PlotCount } from "$lib/telemetryChart/layout";
  import { canToggleTicks, chartPidsOf, deselectOnEscape, deselectOnPointer } from "$lib/telemetryChart/selection";
  import { statsColumnPx } from "$lib/telemetryChart/statsColumns";
  import { freezeWin, resumeWin } from "$lib/telemetryChart/window";
  import { formatValue } from "$lib/telemetryChart/math";
  import ContextScope from "$lib/components/telemetry/ContextScope.svelte";
  import TripPlayer from "$lib/components/telemetry/TripPlayer.svelte";
  import { tripViewer } from "$lib/tripViewer.svelte";
  import type { PlotModel } from "$lib/telemetryChart/plotModel";

  // Ref-counted: the stream stays on while any view needs it.
  $effect(() => connectionStore.acquireStream());

  // One window and one clock for every plot on the page. Plots draw from chartFrames, not a timer.
  const view = new TelemetryView();
  setContext(TELEMETRY_VIEW, view);

  const phone = new MediaQuery("max-width: 859px");

  // The Replays tab plots a trip from the SD card in the same plots, with its own window and clock (TripPlayer).
  const tripView = new TelemetryView(true);
  // Replay mode: a trip is open. The whole page reads it (plots, PIDs tab, player) until Back to live.
  let railPanel = $state<PidPanel>("pids");
  let rec = $state(false);
  const activeView = $derived(rec ? tripView : view);
  /** PIDs per plot while a trip is open, and the plot the PID chips act on. */
  let recPids = $state<number[][]>([]);
  let recActive = $state(0);

  let sideCollapsed = $state(true);
  let phoneTab = $state<PhoneTab>("chart");
  const showChart = $derived(!phone.current || phoneTab === "chart");
  let editorOpen = $state(false);

  let innerH = $state(800);
  /** Index of the plot in full screen, or -1. */
  let fsIndex = $state(-1);
  let activeStats = $state<PlotModel["stats"]>([]);
  let statsTableW = $state(0);

  const PLOT_HEIGHT: Record<PlotCount, number> = { 1: 370, 2: 265, 4: 235, 6: 200 };
  const LANE_HEIGHT: Record<PlotCount, number> = { 1: 110, 2: 90, 4: 72, 6: 72 };
  const plotHeight = $derived(fsIndex >= 0 ? Math.max(240, innerH - 330) : PLOT_HEIGHT[telemetryStore.plotCount]);
  const laneHeight = $derived(
    fsIndex >= 0
      ? Math.max(90, (innerH - 366) / Math.max(1, plotPids(fsIndex).length))
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

  // Live mode only: replay mode ticks go to the trip's plots (below).
  // With a plot armed, its PIDs are the ticked rows. With none armed, a selection is for Edit and charts nothing.
  $effect(() => {
    if (rec) return;
    const ticked = telemetryStore.local_piddef.filter((r) => r.selected).map((row) => parseInt(row.pid, 16));

    untrack(() => {
      if (telemetryStore.activeChartIndex >= 0 && telemetryStore.activeChartIndex < telemetryStore.plotCount) {
        telemetryStore.chartPids[telemetryStore.activeChartIndex] = ticked;
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
    recPids = Array.from({ length: total }, (_, i) => recPids[i] ?? []);
    if (recActive >= total) recActive = 0;
    if (!rec && telemetryStore.activeChartIndex >= total) setActiveChart(0);
  }

  function plotPids(i: number): number[] {
    return (rec ? recPids[i] : telemetryStore.chartPids[i]) ?? [];
  }
  function activate(i: number) {
    if (!rec) return setActiveChart(i);
    recActive = i;
    syncTicks(recPids[i] ?? []);
  }

  /** Ticks the PID list for a plot's PIDs. The list is the same one live mode uses. */
  function syncTicks(pids: number[]) {
    const on = ticksFor(telemetryStore.local_piddef, (r) => parseInt(r.pid, 16), pids);
    telemetryStore.local_piddef.forEach((r, i) => (r.selected = on[i]));
  }

  function setRecPids(plot: number, pids: number[]) {
    recPids = Array.from({ length: Math.max(recPids.length, plot + 1) }, (_, i) => (i === plot ? pids : (recPids[i] ?? [])));
  }

  // Replay mode: a tick in the PID list puts the PID on the selected plot, as in live mode.
  $effect(() => {
    if (!rec || !tripViewer.trip) return;
    const ticked = telemetryStore.local_piddef.filter((r) => r.selected).map((row) => parseInt(row.pid, 16));
    untrack(() => {
      if (ticked.join() !== (recPids[recActive] ?? []).join()) setRecPids(recActive, ticked);
    });
  });
  const activeIdx = $derived(rec ? recActive : telemetryStore.activeChartIndex);

  function toggleFullscreen(index: number) {
    activate(index);
    fsIndex = fsIndex === index ? -1 : index;
  }

  /** Esc leaves full screen, unless a dialog or popover is open above it (it closes first). */
  function onWindowKey(e: KeyboardEvent) {
    if (e.key !== "Escape") return;
    if (document.querySelector("dialog[open]") || document.querySelector(":popover-open")) return;
    if (fsIndex >= 0) return (fsIndex = -1);
    if (rec) return;
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
    if (rec || !t || telemetryStore.activeChartIndex < 0) return;
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

  function toggleChip(pid: number) {
    const row = telemetryStore.local_piddef.find((r) => parseInt(r.pid, 16) === pid);
    if (row) pressRow(row);
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
  function chipClick(pid: number) {
    if (pressFired) pressFired = false;
    else toggleChip(pid);
  }

  const editorTitle = $derived(
    selectedRow && telemetryStore.activeIndex >= 0 ? `Edit ${selectedRow.name} (${selectedRow.pid})` : "New PID",
  );
  const count = $derived(telemetryStore.plotCount);
  const replayList = $derived(rec ? { pids: tripViewer.trip?.pids ?? [], active: recActive } : undefined);
  const activePids = $derived(rec ? (recPids[recActive] ?? []) : chartPidsOf(telemetryStore.chartPids, telemetryStore.activeChartIndex));

  /** A replay was picked in the Replays tab: the page switches to replay mode, on the PIDs tab (desktop) or the chart (phone). */
  function openReplay(name: string) {
    rec = true;
    if (phone.current) phoneTab = "chart";
    else {
      railPanel = "pids";
      sideCollapsed = false;
    }
    fsIndex = -1;
    activeStats = [];
    void tripViewer.load(name);
  }

  /** The way back: the live ticks return to the list, the rail to the PIDs tab, the phone to its live chart. */
  function backToLive() {
    syncTicks(chartPidsOf(telemetryStore.chartPids, telemetryStore.activeChartIndex));
    rec = false;
    railPanel = "pids";
    fsIndex = -1;
    activeStats = [];
  }

  // The Recorder page asks to open a trip: show it in the Replays tab and load it.
  $effect(() => {
    const name = tripViewer.requested;
    if (!name) return;
    tripViewer.requested = null;
    untrack(() => openReplay(name));
  });

  // A newly opened trip starts with its first PIDs on plot 1.
  $effect(() => {
    const trip = tripViewer.trip;
    untrack(() => {
      recPids = Array.from({ length: telemetryStore.plotCount }, (_, i) => (i === 0 ? (trip?.pids.slice(0, 4) ?? []) : []));
      recActive = 0;
      if (trip) syncTicks(recPids[0]);
    });
  });

  const recNote = $derived.by((): { text: string; alert: boolean } | null => {
    if (tripViewer.loading) return { text: "Loading trip...", alert: false };
    if (tripViewer.error) return { text: `Could not open the trip: ${tripViewer.error}`, alert: true };
    const t = tripViewer.trip;
    if (t) {
      const notes = [
        t.truncated ? "The file has more rows than the viewer reads, so the end is cut off." : "",
        t.skipped > 0 ? `${t.skipped.toLocaleString("en")} unreadable rows skipped.` : "",
      ].filter(Boolean);
      return notes.length ? { text: notes.join(" "), alert: false } : null;
    }
    return { text: "Pick a recording in the Replays tab to plot its PIDs and play it back.", alert: false };
  });
</script>

<svelte:window bind:innerHeight={innerH} onkeydown={onWindowKey} onpointerdown={onWindowPointer} />

<section class="tel">
  <div class="wb" class:collapsed={sideCollapsed && !phone.current}>
    {#if !phone.current}
      <aside class="side" aria-label="PID list">
        <PidSidebar variant="rail" bind:panel={railPanel} bind:collapsed={sideCollapsed} onReplay={openReplay} replay={replayList} onNew={newPid} onEdit={editPid} />
      </aside>
    {/if}

    <div class="main">
      <div class="card bar">
        <h1 tabindex="-1">{rec ? "Telemetry · Replay" : "Telemetry"}</h1>
      </div>

      {#if phone.current}
        <PidTabs variant="strip" chart replay={rec} bind:panel={phoneTab} />
      {/if}

      {#if showChart}
      <div class="card bar ctl">
        <h2 class="ctl-title">Plot</h2>
        <div class="seg" role="group" aria-label="Number of plots">
          {#each PLOT_COUNTS as n (n)}
            <button type="button" aria-pressed={count === n} aria-label="{n} {n === 1 ? 'plot' : 'plots'}" onclick={() => setLayout(n)}>{n}</button>
          {/each}
        </div>

        {#if !rec}
        <button type="button" class="btn" class:pri={status !== "live"} onclick={togglePause} aria-label={status === "live" ? "Pause" : "Back to live"}>
          <svg class="ic" viewBox="0 0 24 24" aria-hidden="true">
            {#if status === "live"}<path d="M8 5v14M16 5v14" />{:else}<path d="M5 5v14l10-7zM19 5v14" />{/if}
          </svg>
          {status === "live" ? "Pause" : "Back to live"}
        </button>
        {/if}
      </div>

      {#if rec}
        {#if tripViewer.trip}
          <TripPlayer view={tripView} trip={tripViewer.trip} onBack={backToLive} />
        {/if}
        {#if recNote}
          <p class="rec-note" class:warn={recNote.alert} role={recNote.alert ? "alert" : "status"}>{recNote.text}</p>
        {/if}
      {/if}
      {/if}

      {#if !showChart}
        <div class="card phone-panel">
          <PidSidebar variant="bar" onReplay={openReplay} replay={replayList} bind:panel={() => (phoneTab === "chart" ? "pids" : phoneTab), (v) => (phoneTab = v)} onNew={newPid} onEdit={editPid} />
        </div>
      {:else if !rec || tripViewer.trip}
      {#key rec ? tripViewer.trip : "live"}
      <ContextScope view={activeView} source={rec ? tripViewer.source : undefined}>
      {#if !rec}
      <div class="chips" role="group" aria-label="PIDs on the active plot">
        {#each telemetryStore.local_piddef.filter((r) => r.loaded) as row (row)}
          {@const pid = parseInt(row.pid, 16)}
          {@const on = activeChartPids.includes(pid)}
          <button
            type="button"
            class="tel-pchip"
            aria-pressed={on}
            aria-keyshortcuts="Shift+F10"
            title="Tap to chart it, or select it when no plot is selected. Hold to edit."
            onclick={() => chipClick(pid)}
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
      {/if}

      <div class="cgrid" style="--cols: {gridColumns(count)}">
        {#each Array(count) as _, i (i)}
          <PlotCell
            n={i + 1}
            pids={plotPids(i)}
            active={activeIdx === i}
            fullscreen={fsIndex === i}
            dense={count >= 4}
            {plotHeight}
            {laneHeight}
            onActivate={() => activate(i)}
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
              <tr><td colspan="4" class="none">{activeIdx < 0 ? "Select a plot to see its stats" : activePids.length === 0 ? "No PIDs on this plot" : "No samples in the visible window"}</td></tr>
            {/each}
          </tbody>
        </table>
        </div>
      </section>
      </ContextScope>
      {/key}
      {/if}
    </div>
  </div>
</section>

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
      grid-template-columns: clamp(420px, 31vw, 470px) minmax(0, 1fr);
    }
    /* The side column is the 56 px rail alone when closed; the track animates like the plot tools tray. */
    .wb {
      transition: grid-template-columns 0.28s var(--ease-out);
    }
    .wb.collapsed {
      grid-template-columns: 56px minmax(0, 1fr);
    }
    @media (prefers-reduced-motion: reduce) {
      .wb {
        transition: none;
      }
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
  /* Phone header, same pattern as CAN Bus: title, then the plot-count control full width, then the two actions side by side. */
  @media (max-width: 859px) {
    .bar h1 {
      font-size: 1.7rem;
      flex-basis: 100%;
    }

  }
  /* Chart controls: fixed size, right-aligned on phone and desktop. */
  .ctl {
    justify-content: flex-end;
    flex-wrap: wrap;
  }
  .ctl-title {
    margin: 0 auto 0 0;
    color: var(--muted);
    font-size: 1.05rem;
    font-weight: 700;
  }
  .ctl > .seg,
  .bar.ctl > .btn {
    flex: none;
  }
  .bar.ctl > .btn {
    justify-content: center;
    width: 8rem;
    padding: 0 10px;
  }
  /* Header buttons: 46 px tall, 18 px side padding (mockup .btn). */
  .bar :global(.btn) {
    min-height: 46px;
    padding: 0 18px;
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
  .rec-note {
    margin: 0;
    padding: 0 4px;
    color: var(--muted);
    font-weight: 600;
  }
  .rec-note.warn {
    color: var(--warn, var(--muted));
  }
  .phone-panel {
    padding: 8px 12px 0;
    border-radius: var(--r-card);
    overflow: hidden;
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
    container: stats / inline-size;
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
     shares a column with a long name. Wider cards keep the table, with the name beside the values. The switch is
     on the card's own width, not the window's, so a narrow desktop window with the side panel open still gets it. */
  @container stats (max-width: 520px) {
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
    .stats th {
      padding: 6px 0 8px;
    }
    .stats th.n {
      padding-right: 0;
    }
    .stats td {
      padding: 0 0 12px;
      font-size: 0.95rem;
    }
    .stats td:first-child {
      grid-column: 1 / -1;
      padding: 12px 0 6px;
      border-top: 1px solid var(--hair);
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
