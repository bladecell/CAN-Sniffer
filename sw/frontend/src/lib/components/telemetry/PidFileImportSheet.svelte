<script lang="ts">
  import { untrack } from "svelte";
  import { SvelteSet } from "svelte/reactivity";
  import Sheet from "$lib/components/Sheet.svelte";
  import { safeHexColor } from "$lib/color";
  import { getModeLabel } from "$lib/modeLabel";
  import { pidHex } from "$lib/telemetryView";
  import { chosenDefinitions, classifyPidImport, type CurrentPid, type PidImportStatus } from "$lib/pidImport";
  import type { PidDefinition } from "$lib/types";
  import { formatPid } from "$lib/telemetryView";
  import Icon from "$lib/Icon.svelte";
  import { iconLabel } from "./editor/pidIcons";
  import PidCard from "./PidCard.svelte";

  interface Props {
    open?: boolean;
    fileName: string;
    /** The PIDs read from the file, in file order. */
    defs: PidDefinition[];
    /** The PIDs already in the set (the PIDs tab rows). Nothing here is written. */
    current: CurrentPid[];
    onImport: (defs: PidDefinition[]) => void;
    onClose: () => void;
  }

  let { open = false, fileName, defs, current, onImport, onClose }: Props = $props();

  const uid = $props.id();
  const items = $derived(classifyPidImport(defs, current));

  /** Ticked PIDs. Nothing is ticked when the sheet opens. */
  const picked = new SvelteSet<number>();
  const pickedSet = $derived(new Set(picked));
  const chosenCount = $derived(pickedSet.size);

  /** Expanded details rows, by PID. */
  const expanded = new SvelteSet<number>();

  $effect.pre(() => {
    void open;
    void defs;
    untrack(() => {
      picked.clear();
      expanded.clear();
    });
  });

  /** Status tag: the same tag classes the PIDs tab uses (sb-tag, k-ok, k-warn). Identical is the muted default. */
  const STATUS: Record<PidImportStatus, { label: string; tone: string }> = {
    new: { label: "New", tone: "k-ok" },
    changed: { label: "Changed", tone: "k-warn" },
    identical: { label: "Identical", tone: "" },
  };

  const colorHex = (color: number) => safeHexColor("#" + Number(color).toString(16).padStart(6, "0"));
  const hexByte = (value: number) => `0x${value.toString(16).toUpperCase().padStart(2, "0")}`;
  const bound = (value: number) => (Number.isFinite(value) ? String(value) : "None");

  function toggle(pid: number) {
    if (picked.has(pid)) picked.delete(pid);
    else picked.add(pid);
  }
  function selectAll() {
    for (const def of defs) picked.add(def.pid);
  }
  function selectNone() {
    picked.clear();
  }
  function toggleDetails(pid: number) {
    if (expanded.has(pid)) expanded.delete(pid);
    else expanded.add(pid);
  }
  function importSelected() {
    if (chosenCount === 0) return;
    onImport(chosenDefinitions(items, pickedSet));
  }
</script>

<Sheet {open} title="Import {fileName}" {onClose} resizable>
  <div class="imp">
    <div class="sb-selrow">
      <span class="num" aria-live="polite">{chosenCount} of {defs.length} selected</span>
      <span class="sb-acts">
        {#if chosenCount < defs.length}
          <button type="button" class="sb-txt" onclick={selectAll}>Select all</button>
        {/if}
        {#if chosenCount > 0}
          <button type="button" class="sb-txt" onclick={selectNone}>Select none</button>
        {/if}
      </span>
    </div>
    <p class="tel-note">
      Nothing is written to the device. New: not on the PIDs tab yet. Changed: replaces the PID there. Identical: already there.
    </p>

    <ul class="sb-list imp-list" aria-label="PIDs in {fileName}">
      {#each items as item (item.def.pid)}
        {@const def = item.def}
        {@const on = picked.has(def.pid)}
        {@const isOpen = expanded.has(def.pid)}
        <PidCard
          selected={on}
          color={colorHex(def.color)}
          name={def.name}
          hex={pidHex(def.pid, def.mode)}
          open={isOpen}
          onToggle={() => toggle(def.pid)}
        >
          {#snippet tags()}
            <span class="sb-tag {STATUS[item.status].tone}"><i class="dot"></i>{STATUS[item.status].label}</span>
          {/snippet}
          {#snippet actions()}
            <button
              type="button"
              class="imp-more"
              aria-expanded={isOpen}
              aria-controls="{uid}-d{def.pid}"
              aria-label="Details of {def.name}"
              onclick={() => toggleDetails(def.pid)}
            >
              <span class="imp-chev" class:up={isOpen}><Icon name="chevron-down" size={18} /></span>
            </button>
          {/snippet}
          {#snippet after()}
            <dl id="{uid}-d{def.pid}" class="imp-dl" hidden={!isOpen}>
              <div><dt>Mode</dt><dd>{getModeLabel(def.mode)} <span class="tel-mono">{hexByte(def.mode)}</span></dd></div>
              <div><dt>CAN ID</dt><dd class="tel-mono">0x{def.id.toString(16).toUpperCase()}</dd></div>
              <div><dt>PID</dt><dd class="tel-mono">{formatPid(def.pid)}</dd></div>
              <div><dt>Length</dt><dd>{def.length} {def.length === 1 ? "byte" : "bytes"}</dd></div>
              <div><dt>Formula</dt><dd class="tel-mono">{def.formula || "None"}</dd></div>
              <div><dt>Unit</dt><dd>{def.unit || "None"}</dd></div>
              <div><dt>Min</dt><dd>{bound(def.minValue)}</dd></div>
              <div><dt>Max</dt><dd>{bound(def.maxValue)}</dd></div>
              <div><dt>Name</dt><dd>{def.name}</dd></div>
              <div><dt>Description</dt><dd>{def.description || "None"}</dd></div>
              <div><dt>Color</dt><dd class="imp-color"><i class="imp-sw" style="--sw: {colorHex(def.color)}"></i><span class="tel-mono">{colorHex(def.color)}</span></dd></div>
              <div>
                <dt>Icon</dt>
                <dd class="imp-icon">
                  {#if def.icon}<Icon name={def.icon} size={18} />{iconLabel(def.icon)}{:else}None{/if}
                </dd>
              </div>
              <div><dt>Update interval</dt><dd>{def.update_interval_ms === 0 ? "Off" : `${def.update_interval_ms} ms`}</dd></div>
              <div><dt>Priority</dt><dd>{def.priority}</dd></div>
            </dl>
          {/snippet}
        </PidCard>
      {/each}
    </ul>
  </div>

  {#snippet footer()}
    <button type="button" class="btn pri" aria-disabled={chosenCount === 0} onclick={importSelected}>Import ({chosenCount})</button>
    <button type="button" class="btn" onclick={() => onImport(defs)}>Import all</button>
  {/snippet}
</Sheet>

<style>
  .imp {
    display: grid;
    gap: 10px;
  }
  /* The details control sits in the card's row, centred like the swatch and the name. */
  .imp-more {
    flex: none;
    align-self: center;
    display: grid;
    place-items: center;
    width: 44px;
    height: 44px;
    margin: 0 0 0 4px;
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: none;
    color: var(--muted);
    cursor: pointer;
  }
  .imp-more:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
  }
  .imp-chev {
    display: inline-grid;
    transition: transform 0.15s;
  }
  .imp-chev.up {
    transform: rotate(180deg);
  }
  .imp-dl {
    display: grid;
    gap: 6px;
    margin: 0;
    padding: 0 12px 14px 40px;
    font-size: 0.9rem;
  }
  .imp-color,
  .imp-icon {
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .imp-color .imp-sw {
    width: 14px;
    height: 14px;
    border-radius: 4px;
    background: var(--sw);
  }
  .imp-dl[hidden] {
    display: none;
  }
  .imp-dl > div {
    display: grid;
    grid-template-columns: 7.5rem minmax(0, 1fr);
    gap: 8px;
  }
  .imp-dl dt {
    font-weight: 700;
    color: var(--muted);
  }
  .imp-dl dd {
    margin: 0;
    overflow-wrap: anywhere;
  }
</style>
