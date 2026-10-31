<script lang="ts">
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { alertStore } from "$lib/alertStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { definitionsToApply, pendingKind, pidHex, type PendingKind } from "$lib/telemetryView";
  import Icon from "$lib/Icon.svelte";
  import { swipeRow } from "./swipeRow";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  interface Props {
    /** Phone: a swipe left applies the change and right reverts it. The Revert button is hidden. */
    phone?: boolean;
  }

  let { phone = false }: Props = $props();

  const pidOf = (row: PidRow) => parseInt(row.pid, 16);
  const pidColor = (row: PidRow) => safeHexColor("#" + Number(row.def.color).toString(16).padStart(6, "0"));

  const KIND_ORDER: PendingKind[] = ["added", "modified", "removed"];
  const GROUP: Record<PendingKind, { title: string; icon: string; cls: string; why: string }> = {
    added: { title: "Added", icon: "plus", cls: "t-ok", why: "Not on the device yet" },
    modified: { title: "Modified", icon: "edit", cls: "t-warn", why: "Differs from the device" },
    removed: { title: "Removed", icon: "trash", cls: "t-crit", why: "Will be removed from the device" },
  };

  const devices = $derived(new Map(vehicleStore.pidDefinitions.map((def) => [Number(def.pid), def])));

  const changes = $derived(
    telemetryStore.local_piddef
      .filter((row) => row.pendingChanges === "Yes")
      .map((row) => ({ row, pid: pidOf(row), kind: pendingKind(row.loaded, devices.has(pidOf(row))) })),
  );
  const sections = $derived(
    KIND_ORDER.map((kind) => ({ kind, items: changes.filter((c) => c.kind === kind) })).filter((s) => s.items.length > 0),
  );
  const pendingCount = $derived(changes.length);
  const isSelected = (pid: number) => !telemetryStore.deselectedChanges.has(pid);
  const selectedCount = $derived(changes.filter((c) => isSelected(c.pid)).length);

  function toggle(pid: number) {
    if (isSelected(pid)) telemetryStore.deselectedChanges.add(pid);
    else telemetryStore.deselectedChanges.delete(pid);
  }
  function selectAll() {
    telemetryStore.deselectedChanges.clear();
  }
  function selectNone() {
    for (const c of changes) telemetryStore.deselectedChanges.add(c.pid);
  }

  let toggled = $state<Record<string, boolean>>({});
  const isOpen = (key: string) => toggled[key] ?? true;

  // A failed Apply keeps its PIDs so Retry runs the same write again. The list is inert while Apply runs.
  let applying = $state(false);
  let failure = $state<{ reason: string; pids: number[]; saveOnly?: boolean } | null>(null);

  /** Apply step 1 writes the chosen changes to the device. Step 2 saves the device's set to its file. */
  async function apply(pids: number[]) {
    if (applying || pids.length === 0) return;
    const saveOnly = failure?.saveOnly;
    applying = true;
    failure = null;
    if (saveOnly) return saveToDevice(pids);
    const chosen = new Set(pids);
    const items = telemetryStore.local_piddef.map((row) => ({
      pid: pidOf(row),
      loaded: row.loaded,
      pending: row.pendingChanges === "Yes",
      def: row.def,
      deviceDef: devices.get(pidOf(row)),
    }));

    const written = await vehicleStore.writePidDefinitions(definitionsToApply(items, chosen));
    if (written) {
      applying = false;
      failure = { reason: written, pids };
      return;
    }
    telemetryStore.restoreFromDevice(chosen);
    for (const pid of chosen) telemetryStore.deselectedChanges.delete(pid);

    await saveToDevice(pids);
  }

  async function saveToDevice(pids: number[]) {
    failure = null;
    const saved = await vehicleStore.saveConfiguredPids();
    applying = false;
    if (saved) failure = { reason: `It is on the device, but the file was not saved. ${saved}`, pids, saveOnly: true };
    else alertStore.add(`Applied ${pids.length} ${pids.length === 1 ? "change" : "changes"} and saved them on the device`, "success");
  }

  function applyChosen() {
    void apply(failure ? failure.pids : changes.filter((c) => isSelected(c.pid)).map((c) => c.pid));
  }

  function revert(row: PidRow) {
    const pid = pidOf(row);
    telemetryStore.restoreFromDevice(new Set([pid]));
    telemetryStore.deselectedChanges.delete(pid);
    alertStore.add(`Reverted ${row.name}`, "info");
  }

  const applyLabel = $derived(
    applying
      ? `Applying ${selectedCount}`
      : failure
        ? `Retry Apply ${failure.pids.length}`
        : selectedCount > 0
          ? `Apply ${selectedCount}`
          : pendingCount > 0
            ? "Select changes to apply"
            : "Nothing to apply",
  );
  const applyDisabled = $derived(applying || (!failure && selectedCount === 0));
</script>

<SidebarPanel>
  {#snippet header()}
    <SidebarHint text={SIDEBAR_HINTS.changes} />
    <h2>Changes</h2>
    <p class="sb-counts num">{pendingCount === 0 ? "Matches the device" : `${pendingCount} not on the device`}</p>
    {#if pendingCount > 0}
      <div class="sb-selrow">
        <span class="num">{selectedCount} of {pendingCount} selected</span>
        <span class="sb-acts">
          {#if selectedCount < pendingCount}
            <button type="button" class="sb-txt" onclick={selectAll}>Select all</button>
          {/if}
          {#if selectedCount > 0}
            <button type="button" class="sb-txt" onclick={selectNone}>Select none</button>
          {/if}
        </span>
      </div>
    {/if}
  {/snippet}

  <div class="chg" inert={applying} aria-busy={applying}>
    {#if sections.length === 0}
      <div class="sb-empty"><b>Nothing to apply</b>The device has these PIDs. Edit, add, remove or open a file to see changes here.</div>
    {:else}
      {#each sections as section (section.kind)}
        {@const g = GROUP[section.kind]}
        <section class="sb-group">
          <h3 class="chg-h">
            <button
              type="button"
              class="sb-gh"
              aria-expanded={isOpen(section.kind)}
              aria-controls="chg-{section.kind}"
              onclick={() => (toggled[section.kind] = !isOpen(section.kind))}
            >
              <span class="chev"><Icon name="chevron-down" size={18} /></span>
              <span class="gi {g.cls}"><Icon name={g.icon} size={18} /></span>
              <span>{g.title}</span>
              <span class="cnt">{section.items.length}</span>
            </button>
          </h3>
          {#if isOpen(section.kind)}
            <ul class="sb-list" id="chg-{section.kind}">
              {#each section.items as change (change.row)}
                {@const hex = pidHex(change.pid, change.row.mode)}
                <li class="sb-row" class:sel={isSelected(change.pid)} class:phone>
                  {#if phone}
                    <div class="sb-lay sb-lay-l sb-lay-rev" aria-hidden="true">Revert</div>
                    <div class="sb-lay sb-lay-r sb-lay-apply" aria-hidden="true">Apply</div>
                  {/if}
                  <div class="sb-slide" {@attach swipeRow({ left: () => void apply([change.pid]), right: () => revert(change.row) })}>
                    <button type="button" class="sb-main" aria-pressed={isSelected(change.pid)} onclick={() => toggle(change.pid)}>
                      <i class="sb-sw" style="--sw: {pidColor(change.row)}"></i>
                      <span class="sb-nm">
                        <span class="sb-nl"><b>{change.row.name}</b></span>
                        {#if hex}<span class="sb-hx">{hex}</span>{/if}
                        <span class="sb-why">{g.why}</span>
                      </span>
                    </button>
                    {#if !phone}
                      <button type="button" class="rvb" onclick={() => revert(change.row)} aria-label="Revert {change.row.name}">
                        <Icon name="arrow-u-turn-up-left" size={18} />Revert
                      </button>
                    {/if}
                  </div>
                  {#if phone}
                    <!-- Keyboard path to the swipe actions. Hidden until focused, then the card slides to show the action. -->
                    <button type="button" class="sb-sra sb-sra-l" onclick={() => revert(change.row)}>Revert {change.row.name}</button>
                    <button type="button" class="sb-sra sb-sra-r" onclick={() => void apply([change.pid])}>Apply {change.row.name}</button>
                  {/if}
                </li>
              {/each}
            </ul>
          {/if}
        </section>
      {/each}
    {/if}
  </div>

  {#snippet footer()}
    {#if sections.length > 0}
      <div class="apply">
        {#if failure}
          <div class="fail float-layer" role="alert"><b>Apply failed.</b> {failure.reason}</div>
        {/if}
        <button type="button" class="tel-btn pri" aria-disabled={applyDisabled} onclick={applyChosen}>
          <Icon name="reload" size={18} class={applying ? "spin" : ""} />{applyLabel}
        </button>
      </div>
    {/if}
  {/snippet}
</SidebarPanel>

<style>
  .chg {
    display: grid;
    gap: 8px;
  }
  .chg[inert] {
    opacity: 0.6;
  }
  .chg-h {
    margin: 0;
    font-size: inherit;
  }
  .sb-group {
    display: grid;
    gap: 4px;
  }
  .sb-why {
    font-size: 0.8rem;
    font-weight: 600;
    line-height: 1.3;
    color: var(--muted);
  }
  .rvb {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    align-self: center;
    min-height: 44px;
    padding: 0 12px;
    margin-right: 4px;
    border: 0;
    border-radius: 12px;
    background: none;
    color: var(--accent-text);
    font: 800 0.9rem/1 var(--font);
    white-space: nowrap;
    cursor: pointer;
  }
  @media (hover: hover) {
    .rvb:hover {
      background: var(--hov);
    }
  }
  .sb-gh .gi {
    display: inline-grid;
  }
  .sb-gh .gi.t-ok {
    color: var(--ok);
  }
  .sb-gh .gi.t-warn {
    color: var(--warn);
  }
  .sb-gh .gi.t-crit {
    color: var(--crit);
  }
  /* The Apply bar spans the footer. Its failure note is a float layer too, so it blurs over the list. */
  .apply {
    grid-column: 1 / -1;
    display: grid;
    gap: 8px;
  }
  .fail {
    padding: 10px 12px;
    border-radius: 16px;
    background: linear-gradient(var(--ctl), var(--ctl)), var(--float);
    box-shadow: inset 0 0 0 2px var(--crit), var(--shadow-float);
    font-size: 0.85rem;
    font-weight: 600;
    line-height: 1.4;
  }
  .fail b {
    font-weight: 800;
    color: var(--crit);
  }
</style>
