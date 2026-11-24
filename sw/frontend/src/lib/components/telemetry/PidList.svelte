<script lang="ts">
  import { canToggleTicks } from "$lib/telemetryChart/selection";
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { footerActions, groupPids, pidHex, pressRow, removeRows, replayGroups } from "$lib/telemetryView";
  import Icon from "$lib/Icon.svelte";
  import { slide } from "svelte/transition";
  import { accordion } from "$lib/components/accordion";
  import { swipeRow } from "./swipeRow";
  import PidCard from "./PidCard.svelte";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  interface Props {
    /** Phone: a swipe left edits and right deletes, and the edit button is hidden. */
    phone?: boolean;
    onNew: () => void;
    onEdit: (index: number) => void;
    /** Replay mode: the list holds the open trip's PIDs, ticks go to plot `active`, and definitions cannot be edited or removed. */
    replay?: { pids: number[]; active: number };
  }

  let { phone = false, onNew, onEdit, replay }: Props = $props();

  const pidColor = (row: PidRow) => safeHexColor("#" + Number(row.def.color).toString(16).padStart(6, "0"));
  const hasValue = (row: PidRow) => typeof row.value === "number";
  const onDevice = (row: PidRow) => vehicleStore.pidDefinitions.some((d) => Number(d.pid) === parseInt(row.pid, 16));

  // Rows the device will load. Removed rows are not listed here; Changes shows them.
  const loaded = $derived(telemetryStore.local_piddef.filter((r) => r.loaded));
  const groups = $derived(
    replay
      ? replayGroups(loaded, (row) => parseInt(row.pid, 16), replay.pids)
      : groupPids(loaded, (row) => ({
          supported: row.isSupported,
          hasValue: hasValue(row),
          mode: row.mode,
          modeLabel: row.modeDescription,
        })),
  );
  const shownCount = $derived(groups.reduce((n, g) => n + g.items.length, 0));

  let pendingCount = $derived(telemetryStore.local_piddef.filter((r) => r.pendingChanges === "Yes").length);
  let supportedCount = $derived(loaded.filter((r) => r.isSupported).length);
  let selectedCount = $derived((replay ? groups.flatMap((g) => g.items) : loaded).filter((r) => r.selected).length);

  // Groups and sections the user has toggled. Not supported starts collapsed: it is most of the list on a real car.
  let toggled = $state<Record<string, boolean>>({});
  const startsOpen = (key: string) => key !== "unsupported";
  const isOpen = (key: string) => toggled[key] ?? startsOpen(key);

  // Icons per group, as in the mockup: available is a check, no data a waiting mark, not supported a cross.
  const groupIcon: Record<string, { name: string; cls: string }> = {
    available: { name: "circle-check", cls: "t-ok" },
    nodata: { name: "clock", cls: "t-warn" },
    unsupported: { name: "circle-minus", cls: "t-muted" },
    trip: { name: "timeline-arrow", cls: "t-muted" },
  };

  // With a plot selected, a press ticks the PID onto that plot. With none, a press selects one PID for Edit.
  let noPlot = $derived(!canToggleTicks(replay ? replay.active : telemetryStore.activeChartIndex));

  function selectAll(on: boolean) {
    for (const row of replay ? groups.flatMap((g) => g.items) : loaded) row.selected = on;
  }

  function pressPid(row: PidRow) {
    pressRow(row);
  }

  function removeWhere(matches: (row: PidRow) => boolean, count: number) {
    telemetryStore.local_piddef = removeRows([...telemetryStore.local_piddef], matches, onDevice);
    alertStore.add(`${count} ${count === 1 ? "PID" : "PIDs"} added to Changes as removed`);
  }

  function removeSelected() {
    if (selectedCount === 0) return;
    removeWhere((r) => r.selected, selectedCount);
  }

  function editSelected() {
    const row = loaded.find((r) => r.selected);
    if (row) onEdit(telemetryStore.local_piddef.indexOf(row));
  }

  function removeOne(row: PidRow) {
    removeWhere((r) => r.pid === row.pid, 1);
  }
</script>

{#snippet head(key: string, title: string, count: number, icon?: { name: string; cls: string })}
  <button
    type="button"
    class="sb-gh"
    class:sub={!icon}
    aria-expanded={isOpen(key)}
    aria-controls="grpl-{key}"
    onclick={() => (toggled[key] = !isOpen(key))}
  >
    <span class="chev"><Icon name="chevron-down" size={18} /></span>
    {#if icon}<span class="gi {icon.cls}"><Icon name={icon.name} size={18} /></span>{/if}
    <span>{title}</span>
    <span class="cnt">{count}</span>
  </button>
{/snippet}

{#snippet cards(items: PidRow[])}
  <ul class="sb-list">
    {#each items as row (row)}
      {@const index = telemetryStore.local_piddef.indexOf(row)}
      {@const hex = pidHex(parseInt(row.pid, 16), row.mode)}
      <PidCard
        selected={row.selected}
        color={pidColor(row)}
        name={row.name}
        {hex}
        {phone}
        onToggle={() => pressPid(row)}
        swipe={replay ? undefined : swipeRow({ left: () => onEdit(index), right: () => removeOne(row) })}
      >
        {#snippet behind()}
          {#if phone && !replay}
            <div class="sb-lay sb-lay-l sb-lay-del" aria-hidden="true">Delete</div>
            <div class="sb-lay sb-lay-r sb-lay-apply" aria-hidden="true">Edit</div>
          {/if}
        {/snippet}
        {#snippet tags()}
          {#if !replay && row.isSupported && !row.isValid}
            <span class="sb-tag k-crit" title="Invalid: the formula gives no usable value"><i class="dot"></i>Invalid</span>
          {/if}
          {#if !replay && row.pendingChanges === "Yes"}
            <span class="sb-tag k-warn"><i class="dot"></i>Pending</span>
          {/if}
          {#if !replay && row.updateInterval === 0}
            <span class="sb-tag" title="Static: never polled"><i class="dot"></i>Static</span>
          {/if}
        {/snippet}
        {#snippet value()}
          {#if !replay && row.isSupported}
            <span class="sb-val">
              {#if hasValue(row)}
                <span class="v">{row.value}</span>
                <span class="u">{row.metricUnit}</span>
              {:else}
                <span class="na">N/A</span>
              {/if}
            </span>
          {/if}
        {/snippet}
        {#snippet after()}
          {#if phone && !replay}
            <!-- Keyboard path to the swipe actions. Hidden until focused, then the card slides to show the action. -->
            <button type="button" class="sb-sra sb-sra-l" onclick={() => removeOne(row)}>Delete {row.name}</button>
            <button type="button" class="sb-sra sb-sra-r" onclick={() => onEdit(index)}>Edit {row.name}</button>
          {/if}
        {/snippet}
      </PidCard>
    {/each}
  </ul>
{/snippet}

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={replay ? SIDEBAR_HINTS.replayPids : SIDEBAR_HINTS.pids} />
    <h2>PIDs</h2>
    <p class="sb-counts num">
      {#if replay}
        {shownCount} {shownCount === 1 ? "PID" : "PIDs"} in this trip
      {:else}
        {loaded.length} PIDs &middot; {supportedCount} supported &middot; {pendingCount} pending
      {/if}
    </p>
    <div class="sb-selrow">
      <span class="num">
        {#if noPlot}{selectedCount ? `${selectedCount} selected` : "No plot selected"}{:else}{selectedCount ? `${selectedCount} on chart` : "None on chart"}{/if}
      </span>
      <span class="sb-acts">
        {#if !noPlot && selectedCount < (replay ? shownCount : loaded.length)}
          <button type="button" class="sb-txt" onclick={() => selectAll(true)}>Select all</button>
        {/if}
        {#if selectedCount > 0}
          <button type="button" class="sb-txt" onclick={() => selectAll(false)}>Clear</button>
        {/if}
      </span>
    </div>
  {/snippet}

  <div class="pl-groups">
    {#each groups as group (group.key)}
      <section class="sb-group" aria-labelledby="grp-{group.key}">
        <h3 id="grp-{group.key}" class="sb-gtop">
          {@render head(group.key, group.title, group.items.length, groupIcon[group.key])}
        </h3>
        {#if isOpen(group.key)}
          <div class="sb-gl" id="grpl-{group.key}" transition:slide={accordion()}>
            {#if group.sections}
              {#each group.sections as section (section.key)}
                <section class="sb-group">
                  <h4 style="margin: 0; font-size: inherit;">{@render head(section.key, section.title, section.items.length)}</h4>
                  {#if isOpen(section.key)}
                    <div id="grpl-{section.key}" transition:slide={accordion()}>{@render cards(section.items)}</div>
                  {/if}
                </section>
              {/each}
            {:else}
              {@render cards(group.items)}
            {/if}
          </div>
        {/if}
      </section>
    {:else}
      {#if replay}
        <p class="sb-empty"><b>No PIDs to plot</b>None of this trip's PIDs is in the PID list, so there is nothing to tick.</p>
      {:else}
        <p class="sb-empty"><b>No PIDs here</b>Add one with New PID, or open a file on the SD tab.</p>
      {/if}
    {/each}
  </div>

  {#snippet footer()}
    {#each replay ? [] : footerActions(selectedCount) as action (action)}
      {#if action === "new"}
        <button type="button" class="btn pri" onclick={onNew}><Icon name="plus" size={18} />New PID</button>
      {:else if action === "edit"}
        <button type="button" class="btn" onclick={editSelected}><Icon name="edit" size={18} />Edit</button>
      {:else}
        <button type="button" class="btn rm" onclick={removeSelected}><Icon name="trash" size={18} />Remove {selectedCount}</button>
      {/if}
    {/each}
  {/snippet}
</SidebarPanel>

<style>
  /* Lists inset 18px from the panel edge, as in the mockup: headings and cards share that edge. */
  .pl-groups {
    display: flex;
    flex-direction: column;
    gap: 8px;
    padding: 0 4px;
  }
  /* Top-level group headings stick under the PIDs header on the shared float layer, so cards scroll under them blurred. */
  /* Group headings scroll with their cards. */
  .sb-gtop {
    margin: 0;
    padding: 0;
    font-size: inherit;
  }
  .sb-group > h3,
  .sb-group > h4 {
    display: block;
  }
</style>
