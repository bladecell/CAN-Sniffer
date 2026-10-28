<script lang="ts">
  import { telemetryStore, type PidRow } from "$lib/telemetryStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { safeHexColor } from "$lib/color";
  import { pendingKind, pidHex, type PendingKind } from "$lib/telemetryView";
  import Icon from "$lib/Icon.svelte";
  import SidebarHint from "./SidebarHint.svelte";
  import SidebarPanel from "./SidebarPanel.svelte";
  import { SIDEBAR_HINTS } from "./sidebarHints";

  interface Props {
    onEdit: (index: number) => void;
  }

  let { onEdit }: Props = $props();

  const pidColor = (row: PidRow) => safeHexColor("#" + Number(row.def.color).toString(16).padStart(6, "0"));

  const KIND_ORDER: PendingKind[] = ["added", "modified", "removed"];
  const GROUP: Record<PendingKind, { title: string; icon: string; cls: string; why: string }> = {
    added: { title: "Added", icon: "plus", cls: "t-ok", why: "Not on the device yet" },
    modified: { title: "Modified", icon: "edit", cls: "t-warn", why: "Differs from the device" },
    removed: { title: "Removed", icon: "trash", cls: "t-crit", why: "Will be removed from the device" },
  };

  function kindOf(row: PidRow): PendingKind {
    return pendingKind(row.loaded, vehicleStore.pidDefinitions.some((d) => Number(d.pid) === parseInt(row.pid, 16)));
  }

  const changes = $derived(
    telemetryStore.local_piddef
      .map((row) => ({ row, index: telemetryStore.local_piddef.indexOf(row), kind: kindOf(row) }))
      .filter((c) => c.row.pendingChanges === "Yes"),
  );
  const sections = $derived(
    KIND_ORDER.map((kind) => ({ kind, items: changes.filter((c) => c.kind === kind) })).filter((s) => s.items.length > 0),
  );
  let pendingCount = $derived(changes.length);

  let toggled = $state<Record<string, boolean>>({});
  const isOpen = (key: string) => toggled[key] ?? true;

  async function apply() {
    if (pendingCount === 0) return;
    await vehicleStore.updatePids(telemetryStore.local_piddef);
  }
</script>

<SidebarPanel>
{#snippet header()}
  <SidebarHint text={SIDEBAR_HINTS.changes} />
  <h2>Changes</h2>
  <p class="sb-counts num">{pendingCount === 0 ? "Matches the device" : `${pendingCount} not on the device`}</p>
{/snippet}

{#if sections.length === 0}
  <div class="sb-empty"><b>Nothing to apply</b>The device has these PIDs. Edit, add or remove a PID to see changes here.</div>
{:else}
  {#each sections as section (section.kind)}
    {@const g = GROUP[section.kind]}
    <section class="sb-group">
      <button
        type="button"
        class="sb-gh"
        aria-expanded={isOpen(section.kind)}
        onclick={() => (toggled[section.kind] = !isOpen(section.kind))}
      >
        <span class="chev"><Icon name="chevron-down" size={18} /></span>
        <span class="gi {g.cls}"><Icon name={g.icon} size={18} /></span>
        <span>{g.title}</span>
        <span class="cnt">{section.items.length}</span>
      </button>
      {#if isOpen(section.kind)}
        <ul class="sb-list">
          {#each section.items as change (change.row)}
            <li class="sb-row">
              <div class="sb-slide">
                <button type="button" class="sb-main" onclick={() => onEdit(change.index)} aria-label="Edit {change.row.name}, {g.title}">
                  <i class="sb-sw" style="--sw: {pidColor(change.row)}"></i>
                  <span class="sb-nm">
                    <span class="sb-nl"><b>{change.row.name}</b></span>
                    {#if pidHex(parseInt(change.row.pid, 16), change.row.mode)}
                      <span class="sb-hx">{pidHex(parseInt(change.row.pid, 16), change.row.mode)}</span>
                    {/if}
                    <span class="sb-why">{g.why}</span>
                  </span>
                </button>
                <!-- Revert needs the device API for restoring a PID; shown disabled until it lands. -->
                <button type="button" class="rvb" aria-disabled="true" title="Not available yet">
                  <Icon name="arrow-u-turn-up-left" size={18} />Revert
                </button>
              </div>
            </li>
          {/each}
        </ul>
      {/if}
    </section>
  {/each}
{/if}

{#snippet footer()}
  <button type="button" class="tel-btn pri" aria-disabled={pendingCount === 0} onclick={apply}>
    <Icon name="reload" size={18} />{pendingCount > 0 ? `Apply ${pendingCount}` : "Nothing to apply"}
  </button>
{/snippet}
</SidebarPanel>

<style>
  .sb-list {
    display: grid;
    gap: 8px;
    margin: 0;
    padding: 0;
    list-style: none;
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
    opacity: 0.55;
    cursor: not-allowed;
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
  .sb-gh .gi.t-muted {
    color: var(--muted);
  }
</style>
