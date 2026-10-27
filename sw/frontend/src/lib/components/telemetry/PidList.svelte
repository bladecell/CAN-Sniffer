<script lang="ts">
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { groupPids } from "$lib/telemetryView";
  import Icon from "$lib/Icon.svelte";

  interface Props {
    onNew: () => void;
    onEdit: (index: number) => void;
  }

  let { onNew, onEdit }: Props = $props();

  const pidColor = (row: PidRow) => safeHexColor("#" + Number(row.def.color).toString(16).padStart(6, "0"));
  const hasValue = (row: PidRow) => typeof row.value === "number";

  const groups = $derived(
    groupPids(telemetryStore.local_piddef, (row) => ({
      loaded: row.loaded,
      supported: row.isSupported,
      hasValue: hasValue(row),
      mode: row.mode,
      modeLabel: row.modeDescription,
    })),
  );

  let dirtyCount = $derived(telemetryStore.local_piddef.filter((r) => r.pendingChanges === "Yes").length);
  let loadedCount = $derived(telemetryStore.local_piddef.filter((r) => r.loaded).length);
  let supportedCount = $derived(telemetryStore.local_piddef.filter((r) => r.isSupported).length);
  let selectedCount = $derived(telemetryStore.local_piddef.filter((r) => r.selected).length);
  let allSelected = $derived(telemetryStore.local_piddef.length > 0 && selectedCount === telemetryStore.local_piddef.length);

  // Groups the user has toggled. Unsupported and off-device rows start collapsed; they are most of the list on a real car.
  let toggled = $state<Record<string, boolean>>({});
  const startsOpen = (key: string) => key !== "unsupported" && key !== "notondevice";
  const isOpen = (key: string) => toggled[key] ?? startsOpen(key);

  function selectRow(index: number) {
    telemetryStore.activeIndex = telemetryStore.activeIndex === index ? -1 : index;
  }

  function toggleSelectAll(on: boolean) {
    for (const row of telemetryStore.local_piddef) row.selected = on;
  }

  function indexOf(row: PidRow) {
    return telemetryStore.local_piddef.indexOf(row);
  }
</script>

<div class="pl">
  <header class="pl-head">
    <h2>PIDs</h2>
    <button type="button" class="tel-btn pri sm" onclick={onNew}>
      <Icon name="plus" size={16} />New PID
    </button>
  </header>

  <div class="pl-bar">
    <label class="selall">
      <input
        class="tel-check"
        type="checkbox"
        checked={allSelected}
        onchange={(e) => toggleSelectAll(e.currentTarget.checked)}
        aria-label="Select all PIDs"
        {@attach (el) => {
          el.indeterminate = selectedCount > 0 && !allSelected;
        }}
      />
      Select all
    </label>
    <span class="num counts">{loadedCount} loaded &middot; {supportedCount} supported &middot; {dirtyCount} pending</span>
  </div>

  <div class="pl-groups">
    {#each groups as group (group.key)}
      {@const open = isOpen(group.key)}
      <section class="grp" aria-labelledby="grp-{group.key}">
        <h3 id="grp-{group.key}">
          <button
            type="button"
            class="gh"
            aria-expanded={open}
            aria-controls="grpl-{group.key}"
            onclick={() => (toggled[group.key] = !open)}
          >
            <span class="chev" class:open><Icon name="chevron-down" size={16} /></span>
            <span class="gt">{group.title}</span>
            <span class="cnt num">{group.items.length}</span>
          </button>
        </h3>
        {#if open}
          <ul class="tel-plist" id="grpl-{group.key}">
            {#each group.items as row (row)}
              {@const index = indexOf(row)}
              {@const isCurrent = telemetryStore.activeIndex === index}
              <li class="prow tel-bar-mark" aria-current={isCurrent ? "true" : undefined}>
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
                  <span class="nm">
                    <i style="--sw: {pidColor(row)}"></i>
                    <b>{row.name}</b>
                    {#if row.isSupported && !row.isValid}
                      <span class="st k-crit" role="img" aria-label="Invalid" title="Invalid: the formula gives no usable value">
                        <Icon name="triangle-exclamation" size={14} />
                      </span>
                    {/if}
                    {#if row.pendingChanges === "Yes"}
                      <span class="st k-warn" role="img" aria-label="Pending sync" title="Pending sync">
                        <Icon name="clock" size={14} />
                      </span>
                    {/if}
                  </span>
                  <span class="sub num">{row.def.pid}{row.metricUnit ? ` · ${row.metricUnit}` : ""}</span>
                </button>
                <span class="vcol">
                  {#if hasValue(row)}
                    <span class="v num">{row.value}</span>
                    <small>{row.metricUnit}</small>
                  {:else}
                    <span class="v na">N/A</span>
                  {/if}
                </span>
                <button type="button" class="tel-btn icon sm pedit" aria-label="Edit {row.name}" onclick={() => onEdit(index)}>
                  <Icon name="edit" size={18} />
                </button>
              </li>
            {/each}
          </ul>
        {/if}
      </section>
    {:else}
      <p class="empty">No PIDs loaded yet.</p>
    {/each}
  </div>
</div>

<style>
  .pl {
    display: flex;
    flex-direction: column;
    gap: 10px;
    min-height: 0;
    flex: 1 1 auto;
  }
  .pl-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    padding: 6px 8px 0 10px;
  }
  .pl-head h2 {
    font-size: 1.15rem;
    margin: 0;
  }
  .pl-bar {
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
    display: inline-flex;
    align-items: center;
    gap: 8px;
    min-height: 44px;
    font-weight: 700;
    color: var(--text);
    cursor: pointer;
  }
  .pl-groups {
    display: flex;
    flex-direction: column;
    gap: 6px;
    min-height: 0;
    flex: 1 1 auto;
    overflow: auto;
    padding: 2px;
  }
  .gh {
    all: unset;
    box-sizing: border-box;
    display: flex;
    align-items: center;
    gap: 8px;
    width: 100%;
    min-height: 40px;
    padding: 0 8px;
    border-radius: var(--r-ctl);
    color: var(--ctl-sub);
    font-weight: 800;
    cursor: pointer;
  }
  .gh:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
  }
  @media (hover: hover) {
    .gh:hover {
      background: var(--hov);
    }
  }
  .gh .gt {
    flex: 1;
    min-width: 0;
  }
  .gh .cnt {
    font-size: 0.8rem;
    color: var(--muted);
  }
  .chev {
    display: inline-grid;
    transition: transform 0.2s var(--ease-out);
  }
  .chev:not(.open) {
    transform: rotate(-90deg);
  }
  @media (pointer: coarse) {
    .gh {
      min-height: 44px;
    }
  }
  .tel-plist {
    display: flex;
    flex-direction: column;
    gap: 6px;
    margin: 0 0 6px;
    padding: 0;
    list-style: none;
  }
  .prow {
    position: relative;
    display: flex;
    align-items: stretch;
    gap: 2px;
    min-height: 56px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  @media (hover: hover) {
    .prow:hover {
      background: var(--hov);
    }
  }
  .pcheck {
    display: grid;
    place-items: center;
    width: 44px;
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
    justify-content: center;
    gap: 2px;
    padding: 6px 4px;
    cursor: pointer;
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
    min-width: 0;
  }
  .nm b {
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
  .st {
    display: inline-grid;
    flex: none;
  }
  .st.k-crit {
    color: var(--crit);
  }
  .st.k-warn {
    color: var(--warn);
  }
  .prow[aria-current='true'] .nm b {
    color: var(--accent-text);
  }
  .sub {
    color: var(--muted);
    font-size: 0.8rem;
    line-height: 1.3;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .vcol {
    display: flex;
    flex-direction: column;
    align-items: flex-end;
    justify-content: center;
    min-width: 56px;
    padding-right: 4px;
    font-variant-numeric: tabular-nums;
  }
  .vcol .v {
    font-weight: 800;
    white-space: nowrap;
  }
  .vcol .v.na {
    color: var(--muted);
    font-weight: 700;
  }
  .vcol small {
    color: var(--muted);
    font-size: 0.72rem;
  }
  .pedit {
    align-self: center;
    width: 44px;
    background: transparent;
    margin-right: 2px;
  }
  /* Edit shows on hover, keyboard focus and for the active row; touch has no hover, so it stays visible there. */
  @media (hover: hover) and (pointer: fine) {
    .pedit {
      opacity: 0;
    }
    .prow:hover .pedit,
    .prow:focus-within .pedit,
    .prow[aria-current='true'] .pedit {
      opacity: 1;
    }
  }
  .empty {
    padding: 12px;
    color: var(--muted);
  }
</style>
