<script lang="ts">
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { footerActions, groupPids, pidHex, removeRows } from "$lib/telemetryView";
  import Icon from "$lib/Icon.svelte";
  import { swipeRow } from "./swipeRow";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  interface Props {
    /** Phone: a swipe left edits and right deletes, and the edit button is hidden. */
    phone?: boolean;
    onNew: () => void;
    onEdit: (index: number) => void;
  }

  let { phone = false, onNew, onEdit }: Props = $props();

  const pidColor = (row: PidRow) => safeHexColor("#" + Number(row.def.color).toString(16).padStart(6, "0"));
  const hasValue = (row: PidRow) => typeof row.value === "number";
  const onDevice = (row: PidRow) => vehicleStore.pidDefinitions.some((d) => Number(d.pid) === parseInt(row.pid, 16));

  // Rows the device will load. Removed rows are not listed here; Changes shows them.
  const loaded = $derived(telemetryStore.local_piddef.filter((r) => r.loaded));
  const groups = $derived(
    groupPids(loaded, (row) => ({
      supported: row.isSupported,
      hasValue: hasValue(row),
      mode: row.mode,
      modeLabel: row.modeDescription,
    })),
  );

  let pendingCount = $derived(telemetryStore.local_piddef.filter((r) => r.pendingChanges === "Yes").length);
  let supportedCount = $derived(loaded.filter((r) => r.isSupported).length);
  let selectedCount = $derived(loaded.filter((r) => r.selected).length);

  // Groups and sections the user has toggled. Not supported starts collapsed: it is most of the list on a real car.
  let toggled = $state<Record<string, boolean>>({});
  const startsOpen = (key: string) => key !== "unsupported";
  const isOpen = (key: string) => toggled[key] ?? startsOpen(key);

  // Icons per group, as in the mockup: available is a check, no data a waiting mark, not supported a cross.
  const groupIcon: Record<string, { name: string; cls: string }> = {
    available: { name: "circle-check", cls: "t-ok" },
    nodata: { name: "clock", cls: "t-warn" },
    unsupported: { name: "circle-minus", cls: "t-muted" },
  };

  function selectAll(on: boolean) {
    for (const row of loaded) row.selected = on;
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
      <li class="sb-row prow" class:sel={row.selected} class:phone>
        {#if phone}
          <div class="lay lay-l" aria-hidden="true">Delete</div>
          <div class="lay lay-r" aria-hidden="true">Edit</div>
        {/if}
        <div class="sb-slide" {@attach swipeRow({ left: () => onEdit(index), right: () => removeOne(row) })}>
          <button type="button" class="sb-main" aria-pressed={row.selected} onclick={() => (row.selected = !row.selected)}>
            <i class="sb-sw" style="--sw: {pidColor(row)}"></i>
            <span class="sb-nm">
              <span class="sb-nl">
                <b>{row.name}</b>
                {#if row.isSupported && !row.isValid}
                  <span class="sb-tag k-crit" title="Invalid: the formula gives no usable value"><i class="dot"></i>Invalid</span>
                {/if}
                {#if row.pendingChanges === "Yes"}
                  <span class="sb-tag k-warn"><i class="dot"></i>Pending</span>
                {/if}
                {#if row.updateInterval === 0}
                  <span class="sb-tag" title="Static: never polled"><i class="dot"></i>Static</span>
                {/if}
              </span>
              {#if hex}<span class="sb-hx">{hex}</span>{/if}
            </span>
            {#if row.isSupported}
              <span class="sb-val">
                {#if hasValue(row)}
                  <span class="v">{row.value}</span>
                  <span class="u">{row.metricUnit}</span>
                {:else}
                  <span class="na">N/A</span>
                {/if}
              </span>
            {/if}
          </button>
        </div>
        {#if phone}
          <!-- Keyboard path to the swipe actions. Hidden until focused, then the card slides to show the action. -->
          <button type="button" class="sra sra-l" onclick={() => removeOne(row)}>Delete {row.name}</button>
          <button type="button" class="sra sra-r" onclick={() => onEdit(index)}>Edit {row.name}</button>
        {/if}
      </li>
    {/each}
  </ul>
{/snippet}

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={SIDEBAR_HINTS.pids} />
    <h2>PIDs</h2>
    <p class="sb-counts num">
      {loaded.length} PIDs &middot; {supportedCount} supported &middot; {pendingCount} pending
    </p>
    <div class="sb-selrow">
      <span class="num">{selectedCount ? `${selectedCount} on chart` : "None on chart"}</span>
      <span class="sb-acts">
        {#if selectedCount < loaded.length}
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
          <div class="sb-gl" id="grpl-{group.key}">
            {#if group.sections}
              {#each group.sections as section (section.key)}
                <section class="sb-group">
                  <h4 style="margin: 0; font-size: inherit;">{@render head(section.key, section.title, section.items.length)}</h4>
                  {#if isOpen(section.key)}
                    <div id="grpl-{section.key}">{@render cards(section.items)}</div>
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
      <p class="sb-empty"><b>No PIDs here</b>Add one with New PID, or open a file on the SD tab.</p>
    {/each}
  </div>

  {#snippet footer()}
    {#each footerActions(selectedCount) as action (action)}
      {#if action === "new"}
        <button type="button" class="tel-btn pri" onclick={onNew}><Icon name="plus" size={18} />New PID</button>
      {:else if action === "edit"}
        <button type="button" class="tel-btn" onclick={editSelected}><Icon name="edit" size={18} />Edit</button>
      {:else}
        <button type="button" class="tel-btn rm" onclick={removeSelected}><Icon name="trash" size={18} />Remove {selectedCount}</button>
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
  .sra {
    position: absolute;
    z-index: 3;
    top: 50%;
    width: 1px;
    height: 1px;
    padding: 0;
    border: 0;
    background: none;
    color: transparent;
    font-size: 0;
    white-space: nowrap;
    clip-path: inset(50%);
    overflow: hidden;
    transform: translateY(-50%);
  }
  .sra:focus-visible {
    top: 0;
    bottom: 0;
    width: 112px;
    height: auto;
    transform: none;
    clip-path: none;
    overflow: visible;
    outline: none;
  }
  .sra-l {
    left: 0;
  }
  .sra-r {
    right: 0;
  }
  /* Phone swipe: the layers sit under the card and show the action for the drag direction. */
  .lay {
    position: absolute;
    inset: 0;
    z-index: -1;
    display: flex;
    align-items: center;
    padding: 0 20px;
    font-weight: 800;
    font-size: 0.95rem;
    visibility: hidden;
  }
  .lay-l {
    justify-content: flex-start;
    background: var(--crit);
    color: var(--bg);
  }
  .lay-r {
    justify-content: flex-end;
    background: var(--accent);
    color: var(--on-accent);
  }
  .prow:global([data-dir='left']) .lay-r,
  .prow:global([data-dir='right']) .lay-l {
    visibility: visible;
  }
  .prow:has(.sra-l:focus-visible) .slide,
  .prow:has(.sra-l:focus-visible) :global(.sb-slide) {
    transform: translateX(112px);
  }
  .prow:has(.sra-r:focus-visible) :global(.sb-slide) {
    transform: translateX(-112px);
  }
  .prow:has(.sra-l:focus-visible) .lay-l,
  .prow:has(.sra-r:focus-visible) .lay-r {
    visibility: visible;
  }
  .prow.phone :global(.sb-slide) {
    background: linear-gradient(var(--row), var(--row)), var(--float-solid);
    touch-action: pan-y;
    user-select: none;
    -webkit-user-select: none;
  }
  .prow.phone .lay {
    border-radius: var(--r-ctl);
  }
</style>
