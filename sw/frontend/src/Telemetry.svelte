<script lang="ts">
  import "$lib/components/telemetry/telemetry.css";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { chartSyncStore } from "$lib/chartSyncStore.svelte";
  import { telemetryStore } from "$lib/telemetryStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { untrack } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import Sheet from "$lib/components/Sheet.svelte";
  import SDCardImportSheet from "$lib/components/SDCardImportSheet.svelte";
  import PidSidebar from "$lib/components/telemetry/PidSidebar.svelte";
  import PidEditorForm from "$lib/components/telemetry/PidEditorForm.svelte";
  import PidEditorActions from "$lib/components/telemetry/editor/PidEditorActions.svelte";
  import ChartCell from "$lib/components/telemetry/ChartCell.svelte";
  import FloatPanel from "$lib/components/telemetry/FloatPanel.svelte";
  import LayoutGrid from "$lib/components/telemetry/LayoutGrid.svelte";
  import { chartPidsFor, chipAction } from "$lib/telemetryView";

  // Ref-counted: the stream stays on while any view needs it.
  $effect(() => connectionStore.acquireStream());

  let showSdImportModal = $state(false);
  let pidSheetOpen = $state(false);
  let editorOpen = $state(false);

  let selectedRow = $derived(
    telemetryStore.activeIndex >= 0 && telemetryStore.activeIndex < telemetryStore.local_piddef.length
      ? telemetryStore.local_piddef[telemetryStore.activeIndex]
      : null,
  );

  // Ticked PIDs (or the selected row when none is ticked) are what the active chart shows.
  $effect(() => {
    const ticked = telemetryStore.local_piddef.filter((r) => r.selected).map((row) => parseInt(row.pid, 16));
    const currentSelectedPids = chartPidsFor(ticked, selectedRow?.pid ? parseInt(selectedRow.pid, 16) : null);

    untrack(() => {
      if (telemetryStore.activeChartIndex >= 0 && telemetryStore.activeChartIndex < telemetryStore.chartRows * telemetryStore.chartCols) {
        telemetryStore.chartPids[telemetryStore.activeChartIndex] = currentSelectedPids;
      }
    });
  });

  function handleLayoutSelect(r: number, c: number) {
    telemetryStore.chartRows = r;
    telemetryStore.chartCols = c;
    const total = r * c;
    const newChartPids = [...telemetryStore.chartPids];
    while (newChartPids.length < total) newChartPids.push([]);
    telemetryStore.chartPids = newChartPids;
    if (telemetryStore.activeChartIndex >= total) setActiveChart(0);
  }

  function setActiveChart(index: number) {
    // Pointer-down fires on every drag and zoom of the active chart; only a switch of chart rewrites the selection.
    if (index === telemetryStore.activeChartIndex) return;
    telemetryStore.activeChartIndex = index;
    const targetPids = telemetryStore.chartPids[index] || [];
    for (const item of telemetryStore.local_piddef) {
      item.selected = targetPids.includes(parseInt(item.pid, 16));
    }
  }

  async function handleImport() {
    await vehicleStore.requestSDInfo();
    if (!vehicleStore.sdInfo?.is_mounted) {
      alertStore.add("SD Card is not mounted or present.", "warning");
      return;
    }
    showSdImportModal = true;
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
  const activeChartPids = $derived(telemetryStore.chartPids[telemetryStore.activeChartIndex] ?? []);

  function toggleChip(pid: number, pressed: boolean) {
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
  const cellCount = $derived(telemetryStore.chartRows * telemetryStore.chartCols);
</script>

<SDCardImportSheet isOpen={showSdImportModal} onClose={() => (showSdImportModal = false)} />

{#snippet syncSwitch()}
  <span class="sync">
    <button
      type="button"
      class="tel-switch"
      role="switch"
      aria-checked={chartSyncStore.syncEnabled}
      aria-label="Sync zoom"
      onclick={() => (chartSyncStore.syncEnabled = !chartSyncStore.syncEnabled)}
    ></button>
    <span aria-hidden="true">Sync zoom</span>
  </span>
{/snippet}

<section class="tel">
  <div class="wb">
    <aside class="card side" aria-label="PID list">
      <PidSidebar variant="rail" onImport={handleImport} onNew={newPid} onEdit={editPid} />
    </aside>

    <div class="main">
      <div class="card bar">
        <h1 tabindex="-1">Telemetry</h1>

        <div class="src">
          <div class="tel-seg" role="group" aria-label="Data source">
            <button type="button" aria-pressed="true">Live</button>
            <button
              type="button"
              aria-disabled="true"
              aria-describedby="src-tag"
              onclick={() => alertStore.add("Recorded trips open here from Recorder, Analyze. Not available yet.", "info")}
            >
              Recorded
            </button>
          </div>
          <span class="tel-tag" id="src-tag">Recorded: needs API</span>
        </div>

        <div class="desk">
          <FloatPanel class="tel-btn" ariaLabel="Chart layout">
            {#snippet trigger()}
              <Icon name="grid" size={18} />
              <span>Layout <b class="num">{telemetryStore.chartRows} &times; {telemetryStore.chartCols}</b></span>
            {/snippet}
            {#snippet children(close)}
              <LayoutGrid
                rows={telemetryStore.chartRows}
                cols={telemetryStore.chartCols}
                onPick={(r, c) => {
                  handleLayoutSelect(r, c);
                  close();
                }}
              />
            {/snippet}
          </FloatPanel>
          {@render syncSwitch()}
        </div>

        <div class="phone">
          <button type="button" class="tel-btn" onclick={() => (pidSheetOpen = true)} aria-haspopup="dialog">
            <Icon name="list" size={18} />PIDs
          </button>
          <FloatPanel class="tel-btn" ariaLabel="Chart options">
            {#snippet trigger()}
              <Icon name="grid" size={18} />Chart
            {/snippet}
            {#snippet children(close)}
              <LayoutGrid
                rows={telemetryStore.chartRows}
                cols={telemetryStore.chartCols}
                onPick={(r, c) => {
                  handleLayoutSelect(r, c);
                  close();
                }}
              />
              <div class="opt">{@render syncSwitch()}</div>
            {/snippet}
          </FloatPanel>
        </div>

        <button
          type="button"
          class="tel-btn"
          aria-disabled={telemetryStore.activeIndex < 0}
          title={telemetryStore.activeIndex < 0 ? "Select a PID to edit it. New PID is in the list." : undefined}
          onclick={() => telemetryStore.activeIndex >= 0 && (editorOpen = true)}
        >
          <Icon name="edit" size={18} />Edit PID
        </button>
      </div>

      <div class="chips" role="group" aria-label="PIDs on the active chart">
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

      <div class="cgrid" style="--cols: {telemetryStore.chartCols}">
        {#each Array(cellCount) as _, i (i)}
          <ChartCell
            n={i + 1}
            pids={telemetryStore.chartPids[i] || []}
            active={telemetryStore.activeChartIndex === i}
            onActivate={() => setActiveChart(i)}
          />
        {/each}
      </div>
    </div>
  </div>
</section>

<Sheet open={pidSheetOpen} title="PIDs" onClose={() => (pidSheetOpen = false)}>
  <PidSidebar variant="bar" onImport={handleImport} onNew={newPid} onEdit={editPid} />
</Sheet>

<Sheet open={editorOpen} title={editorTitle} onClose={() => (editorOpen = false)} resizable>
  <PidEditorForm onDone={() => (editorOpen = false)} />
  {#snippet footer()}
    <PidEditorActions formId="pid-editor" />
  {/snippet}
</Sheet>

<style>
  .tel {
    --gap: 20px;
    --stick: 104px;
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
    .tel {
      --gap: 20px;
    }
    .wb {
      grid-template-columns: clamp(340px, 19vw, 400px) minmax(0, 1fr);
    }
    .side {
      display: flex;
      flex-direction: column;
      position: sticky;
      top: var(--stick);
      max-height: calc(100dvh - var(--stick) - var(--gap));
      min-height: 0;
      padding: 12px;
      box-sizing: border-box;
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
  .src {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
  }
  .desk {
    display: none;
    flex-wrap: wrap;
    align-items: center;
    gap: 14px;
  }
  .phone {
    display: flex;
    gap: 8px;
  }
  @media (min-width: 860px) {
    .desk {
      display: flex;
    }
    .phone {
      display: none;
    }
  }
  .sync {
    display: inline-flex;
    align-items: center;
    gap: 10px;
    font-weight: 700;
  }
  .opt {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 16px;
    margin-top: 14px;
    font-weight: 700;
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
</style>
