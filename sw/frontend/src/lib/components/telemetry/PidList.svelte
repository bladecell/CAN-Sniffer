<script lang="ts">
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { applyPidView, pidStatusChips, seenLabel, PID_FIELDS, PID_SORT_KEYS, type PidField } from "$lib/telemetryView";
  import { lastReceivedMs } from "$lib/telemetryLastSeen";
  import { pidView } from "$lib/telemetryViewState.svelte";
  import type { FilterOperator } from "$lib/tableRows";
  import type { SelectMenuOption } from "$lib/selectMenu";
  import Icon from "$lib/Icon.svelte";
  import Dialog from "$lib/components/Dialog.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import FloatPanel from "./FloatPanel.svelte";

  interface Props {
    onImport: () => void;
    onNew: () => void;
    onEdit: (index: number) => void;
  }

  let { onImport, onNew, onEdit }: Props = $props();

  const sortOptions: SelectMenuOption<string>[] = [
    { value: "", label: "Default order" },
    ...PID_SORT_KEYS.map(([value, label]) => ({ value, label })),
  ];
  const operatorOptions: SelectMenuOption<FilterOperator>[] = [
    { value: "contains", label: "contains" },
    { value: "equals", label: "equals" },
    { value: "starts_with", label: "starts with" },
    { value: "gt", label: "> greater than" },
    { value: "lt", label: "< less than" },
  ];

  const pidColor = (row: PidRow) => safeHexColor("#" + Number(row.def.color).toString(16).padStart(6, "0"));

  function cellsOf(row: PidRow) {
    return {
      name: row.name,
      pid: row.def.pid,
      mode: row.modeDescription,
      value: typeof row.value === "number" ? row.value : null,
      interval: row.updateInterval,
      priority: row.priority,
      supported: row.supported,
      valid: row.validDisplay,
    };
  }

  const visibleRows = $derived(
    applyPidView(
      telemetryStore.local_piddef.map((row) => ({ row, cells: cellsOf(row) })),
      pidView.rules,
      pidView.sortKey ? { key: pidView.sortKey, dir: pidView.sortDir } : null,
    ),
  );

  let dirtyCount = $derived(telemetryStore.local_piddef.filter((r) => r.pendingChanges === "Yes").length);
  let loadedCount = $derived(telemetryStore.local_piddef.filter((r) => r.loaded).length);
  let supportedCount = $derived(telemetryStore.local_piddef.filter((r) => r.isSupported).length);
  let selectedRows = $derived(telemetryStore.local_piddef.filter((r) => r.selected));
  let selectedCount = $derived(selectedRows.length);
  let selectedDirtyCount = $derived(selectedRows.filter((r) => r.pendingChanges === "Yes").length);
  let allShownSelected = $derived(visibleRows.length > 0 && visibleRows.every((r) => r.selected));

  let updateText = $derived(selectedCount > 0 ? `Update ${selectedCount} Selected` : `Update ${dirtyCount} Pending`);
  let updateDisabled = $derived(selectedCount > 0 ? selectedDirtyCount !== selectedCount : dirtyCount === 0);

  // Ages tick once a second, only while the Value age field is shown.
  let now = $state(Date.now());
  $effect(() => {
    if (!pidView.fields.has("age")) return;
    const timer = setInterval(() => (now = Date.now()), 1000);
    return () => clearInterval(timer);
  });

  function ageLabel(pid: number) {
    const received = lastReceivedMs(pid);
    return seenLabel(received === null ? null : now - received);
  }

  function selectRow(index: number) {
    telemetryStore.activeIndex = telemetryStore.activeIndex === index ? -1 : index;
  }

  function setField(key: PidField, on: boolean) {
    if (on) pidView.fields.add(key);
    else pidView.fields.delete(key);
  }

  function addRule() {
    pidView.rules = [...pidView.rules, { id: crypto.randomUUID(), column: "name", operator: "contains", value: "" }];
  }

  function removeRule(id: string) {
    pidView.rules = pidView.rules.filter((r) => r.id !== id);
  }

  function toggleSelectAll(on: boolean) {
    for (const row of visibleRows) row.selected = on;
  }

  function exportDefinitions() {
    const blob = new Blob([JSON.stringify(vehicleStore.pidDefinitions, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "can_sniffer_pid_definitions.json";
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  let confirmOpen = $state(false);

  async function confirmRemove() {
    const removed = new Set(selectedRows.map((r) => parseInt(r.pid, 16)));
    const desired = telemetryStore.local_piddef.filter((row) => row.loaded && !removed.has(parseInt(row.pid, 16)));
    confirmOpen = false;
    await vehicleStore.replacePids(desired);
  }

  function requestRemove() {
    if (selectedCount === 0) return;
    confirmOpen = true;
  }

  async function update() {
    if (updateDisabled) return;
    await vehicleStore.updatePids(telemetryStore.local_piddef);
  }
</script>

<div class="tel-pidlist">
  <header class="tel-head">
    <h2>PIDs</h2>
    <button type="button" class="tel-btn pri sm" onclick={onNew}>
      <Icon name="plus" size={16} />New PID
    </button>
  </header>

  <div class="tel-tools">
    <button type="button" class="tel-btn sm" onclick={onImport}>
      <Icon name="import" size={16} />Import
    </button>
    <button type="button" class="tel-btn sm" onclick={exportDefinitions}>
      <Icon name="export" size={16} />Export
    </button>
    <FloatPanel class="tel-btn sm" ariaLabel="Filter PIDs">
      {#snippet trigger()}
        <Icon name="filter" size={16} />Filter
        {#if pidView.rules.length > 0}<span class="badge">{pidView.rules.length}</span>{/if}
      {/snippet}
        <div class="ph">Filter by</div>
        <div class="rules">
          {#each pidView.rules as rule (rule.id)}
            <div class="rule">
              <div class="rule-sel">
                <SelectMenu bind:value={rule.column} options={PID_SORT_KEYS.map(([value, label]) => ({ value, label }))} ariaLabel="Filter column" />
              </div>
              <div class="rule-sel">
                <SelectMenu bind:value={rule.operator} options={operatorOptions} ariaLabel="Filter operator" />
              </div>
              <input class="tel-in" type="text" placeholder="Value" aria-label="Filter value" bind:value={rule.value} />
              <button type="button" class="tel-btn icon sm" aria-label="Remove filter" onclick={() => removeRule(rule.id)}>
                <Icon name="trash" size={16} />
              </button>
            </div>
          {:else}
            <p class="muted">No active filters.</p>
          {/each}
        </div>
        <div class="acts">
          <button type="button" class="tel-btn sm pri" onclick={addRule}>Add filter</button>
          {#if pidView.rules.length > 0}
            <button type="button" class="tel-btn sm" onclick={() => (pidView.rules = [])}>Reset filters</button>
          {/if}
        </div>
    </FloatPanel>
    <FloatPanel class="tel-btn sm" ariaLabel="Show fields">
      {#snippet trigger()}
        <Icon name="eye" size={16} />View
      {/snippet}
        <div class="ph">Show on each PID</div>
        <div class="vlist">
          {#each PID_FIELDS as [key, label] (key)}
            <label class="vrow">
              <input class="tel-check" type="checkbox" checked={pidView.fields.has(key)} onchange={(e) => setField(key, e.currentTarget.checked)} />
              {label}
            </label>
          {/each}
          <p class="note">Formula error text <span class="tel-tag">needs API</span></p>
        </div>
    </FloatPanel>
  </div>

  <div class="tel-sortrow">
    <div class="sort-sel">
      <SelectMenu bind:value={pidView.sortKey} options={sortOptions} ariaLabel="Sort by" />
    </div>
    <button
      type="button"
      class="tel-btn icon sm"
      aria-label="Sort direction, {pidView.sortDir > 0 ? 'ascending' : 'descending'}"
      onclick={() => (pidView.sortDir = pidView.sortDir > 0 ? -1 : 1)}
    >
      <span class="dir" class:down={pidView.sortDir < 0}><Icon name="chevron-down" size={16} /></span>
    </button>
  </div>

  <div class="tel-counts">
    <label class="selall">
      <input class="tel-check" type="checkbox" checked={allShownSelected} onchange={(e) => toggleSelectAll(e.currentTarget.checked)} aria-label="Select all PIDs shown" />
      Select all
    </label>
    <span class="num">{loadedCount} loaded &middot; {supportedCount} supported &middot; {dirtyCount} pending</span>
  </div>

  <ul class="tel-plist">
    {#each visibleRows as row (row)}
      {@const index = telemetryStore.local_piddef.indexOf(row)}
      {@const isCurrent = telemetryStore.activeIndex === index}
      {@const chips = pidStatusChips(
        { loaded: row.loaded, supported: row.isSupported, valid: row.isValid, pending: row.pendingChanges === "Yes" },
        pidView.fields,
      )}
      {@const sub = [
        row.pid,
        pidView.fields.has("mode") ? row.modeDescription : null,
        pidView.fields.has("interval") ? (row.updateInterval ? `${row.updateInterval} ms` : "Polling off") : null,
        pidView.fields.has("length") ? (row.mode === 0x45 ? "no length" : `${row.len} B`) : null,
        pidView.fields.has("priority") ? `prio ${row.priority}` : null,
        pidView.fields.has("range") ? `${row.min_val} to ${row.max_val}` : null,
      ].filter(Boolean)}
      <li class="tel-prow tel-bar-mark" aria-current={isCurrent ? "true" : undefined}>
        <label class="pcheck">
          <input
            class="tel-check"
            type="checkbox"
            checked={row.selected}
            onchange={(e) => (row.selected = e.currentTarget.checked)}
            aria-label="Show {row.name} on chart"
          />
        </label>
        <button type="button" class="pmain" onclick={() => selectRow(index)}>
          <span class="nm"><i style="--sw: {pidColor(row)}"></i>{row.name}</span>
          <span class="psub num">{sub.join(" · ")}</span>
          {#if pidView.fields.has("formula")}<span class="psub mono">{row.formula}</span>{/if}
          {#if pidView.fields.has("age")}<span class="tel-tag num">{ageLabel(row.def.pid)}</span>{/if}
          {#if chips.length > 0}
            <span class="chips">
              {#each chips as chip (chip.label)}
                <span class="tel-chip k-{chip.kind}">
                  {#if chip.icon === "check"}<Icon name="check" size={14} />{:else if chip.icon === "warn"}<Icon name="circle-exclamantion" size={14} />{:else if chip.icon === "clock"}<Icon name="clock" size={14} />{/if}
                  {chip.label}
                </span>
              {/each}
            </span>
          {/if}
        </button>
        <span class="vcol">
          <span class="v num">{row.value}</span>
          {#if typeof row.value === "number"}<small>{row.metricUnit}</small>{/if}
        </span>
        {#if pidView.fields.has("loaded")}
          <button
            type="button"
            class="tel-switch loaded"
            role="switch"
            aria-checked={row.loaded}
            aria-label="Loaded: {row.name}"
            onclick={() => (row.loaded = !row.loaded)}
          ></button>
        {/if}
        <button type="button" class="tel-btn icon sm pedit" aria-label="Edit {row.name}" onclick={() => onEdit(index)}>
          <Icon name="edit" size={18} />
        </button>
      </li>
    {:else}
      <li class="empty">No PID matches the filter.</li>
    {/each}
  </ul>

  <footer class="tel-pfoot">
    <div class="row2">
      <button type="button" class="tel-btn danger" aria-disabled={selectedCount === 0} onclick={requestRemove}>
        <Icon name="trash" size={16} />Remove{selectedCount > 0 ? ` ${selectedCount}` : ""}
      </button>
      <button type="button" class="tel-btn pri" aria-disabled={updateDisabled} onclick={update}>{updateText}</button>
    </div>
    <button type="button" class="tel-btn" onclick={() => vehicleStore.savePids(telemetryStore.local_piddef)} title="Save the PID map to the SD card">
      <Icon name="save" size={16} />Save to SD card
    </button>
  </footer>
</div>

<Dialog
  open={confirmOpen}
  title="Remove {selectedCount} PID{selectedCount === 1 ? '' : 's'}?"
  message="Removes the definitions from the device. The SD card file changes when you press Save to SD card."
  confirmLabel="Remove {selectedCount}"
  danger
  onConfirm={confirmRemove}
  onCancel={() => (confirmOpen = false)}
/>

<style>
  .tel-pidlist {
    display: flex;
    flex-direction: column;
    gap: 10px;
    min-height: 0;
    flex: 1 1 auto;
  }
  .tel-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    padding: 6px 8px 0 10px;
  }
  .tel-head h2 {
    font-size: 1.15rem;
    margin: 0;
  }
  .tel-tools {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 6px;
    padding: 0 6px;
  }
  .tel-tools :global(.float-panel),
  .tel-tools > .tel-btn {
    min-width: 0;
    width: 100%;
  }
  .tel-tools :global(.float-panel > .tel-btn) {
    width: 100%;
  }
  .tel-tools .tel-btn {
    padding: 0 6px;
    font-size: 0.85rem;
    gap: 5px;
  }
  .tel-tools .tel-btn :global(.badge) {
    font-size: 0.7rem;
  }
  .tel-sortrow {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 0 6px;
  }
  .sort-sel {
    flex: 1;
    min-width: 0;
  }
  .dir {
    display: inline-grid;
    transition: transform 0.2s;
  }
  .dir.down {
    transform: scaleY(-1);
  }
  .tel-counts {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 4px 12px;
    padding: 0 10px;
    font-size: 0.85rem;
    color: var(--muted);
  }
  .selall {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 700;
    color: var(--text);
    cursor: pointer;
  }
  .tel-plist {
    display: flex;
    flex-direction: column;
    gap: 8px;
    min-height: 120px;
    flex: 1 1 auto;
    overflow: auto;
    padding: 2px;
  }
  .tel-prow {
    display: flex;
    flex: none;
    align-items: stretch;
    gap: 2px;
    min-height: 72px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  @media (hover: hover) {
    .tel-prow:hover {
      background: var(--hov);
    }
  }
  .pcheck {
    display: grid;
    place-items: center;
    width: 36px;
    flex: none;
    cursor: pointer;
  }
  .pmain {
    all: unset;
    box-sizing: border-box;
    flex: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
    gap: 2px;
    padding: 8px 4px;
    cursor: pointer;
  }
  .pmain > * {
    flex: none;
  }
  .pmain:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
    border-radius: var(--r-ctl);
  }
  .nm {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 800;
    overflow-wrap: anywhere;
  }
  .nm i {
    display: inline-block;
    width: 14px;
    height: 4px;
    border-radius: 2px;
    background: var(--sw);
    flex: none;
  }
  .tel-prow[aria-current='true'] .nm {
    color: var(--accent-text);
  }
  .psub {
    display: block;
    min-width: 0;
    color: var(--muted);
    font-size: 0.8rem;
    line-height: 1.3;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .mono {
    font-family: var(--mono);
  }
  .chips {
    display: flex;
    flex-wrap: wrap;
    gap: 4px;
    margin-top: 3px;
  }
  .chips .tel-chip {
    font-size: 0.7rem;
    line-height: 1.55;
    padding: 0 8px 0 6px;
  }
  .tel-prow .tel-tag {
    align-self: flex-start;
    margin-top: 2px;
  }
  .vcol {
    display: flex;
    flex-direction: column;
    align-items: flex-end;
    justify-content: center;
    min-width: 52px;
    padding-right: 4px;
    font-variant-numeric: tabular-nums;
  }
  .vcol .v {
    font-weight: 800;
    white-space: nowrap;
  }
  .vcol small {
    color: var(--muted);
    font-size: 0.72rem;
  }
  .tel-switch.loaded {
    align-self: center;
    flex: none;
  }
  .pedit {
    align-self: center;
    width: 36px;
    background: transparent;
    margin-right: 2px;
  }
  /* Edit shows on hover, keyboard focus and for the active row; touch has no hover, so it stays visible there. */
  @media (hover: hover) and (pointer: fine) {
    .pedit {
      opacity: 0;
    }
    .tel-prow:hover .pedit,
    .tel-prow:focus-within .pedit,
    .tel-prow[aria-current="true"] .pedit {
      opacity: 1;
    }
  }
  .empty {
    padding: 12px;
    color: var(--muted);
  }
  .tel-pfoot {
    display: grid;
    gap: 6px;
    padding: 4px 6px 4px;
  }
  .tel-pfoot .row2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
  }
  .tel-pfoot .tel-btn {
    width: 100%;
    padding: 0 8px;
    font-size: 0.88rem;
  }
  .ph {
    font-weight: 800;
    margin-bottom: 8px;
  }
  .rules {
    display: grid;
    gap: 8px;
    margin-bottom: 10px;
  }
  .rule {
    display: grid;
    grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
    gap: 6px;
    align-items: center;
  }
  .rule-sel {
    min-width: 0;
  }
  .rule .tel-in {
    grid-column: 1 / -1;
    min-height: calc(var(--ctl-h) - 8px);
    width: auto;
  }
  .rule .icon {
    justify-self: end;
  }
  .acts {
    display: flex;
    gap: 8px;
    justify-content: space-between;
    margin-top: 6px;
    flex-wrap: wrap;
  }
  .vlist {
    display: grid;
    gap: 2px;
  }
  .vrow {
    display: flex;
    align-items: center;
    gap: 12px;
    min-height: 40px;
    padding: 0 6px;
    border-radius: var(--r-ctl);
    font-weight: 600;
    cursor: pointer;
  }
  @media (hover: hover) {
    .vrow:hover {
      background: var(--hov);
    }
  }
  .note {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
    padding: 6px 6px 0;
    color: var(--muted);
    font-size: 0.8rem;
  }
  .muted {
    color: var(--muted);
  }
</style>
