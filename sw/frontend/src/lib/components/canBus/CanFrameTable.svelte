<script lang="ts">
  import type { LogRow, IdView, TableFilter } from "$lib/canBus/canModel";
  import type { TableMode } from "$lib/canBus/canBusStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import { FILTER_FIELDS, opsFor, parseFilterValue, type FilterField, type FilterOp, type FilterRule } from "$lib/canBus/canFilter";
  import { relTime } from "$lib/canBus/canFormat";
  import { measureHeight, trackUnder } from "$lib/layers";
  import { formatId, idColorIndex, idKey } from "$lib/canBus/canIds";

  type Row = LogRow | IdView;

  interface Props {
    mode: TableMode;
    rows: LogRow[];
    byId: IdView[];
    startUs: number | null;
    selected: number | null;
    onselect: (key: number) => void;
    ondetails: (key: number) => void;
    onmode: (mode: TableMode) => void;
    onclear: () => void;
    filter: TableFilter;
    rules: FilterRule[];
    onfilter: (filter: TableFilter) => void;
    onrules: (rules: FilterRule[]) => void;
  }

  let { mode, rows, byId, startUs, selected, onselect, ondetails, onmode, onclear, filter, rules, onfilter, onrules }: Props = $props();

  // The bar sits beside the scroller (Gecko drops the blur inside one), so mirror its horizontal scroll and scrollbar width.
  function syncBar(el: HTMLElement) {
    const host = el.parentElement;
    const sync = () => {
      host?.style.setProperty("--sl", `${-el.scrollLeft}px`);
      host?.style.setProperty("--sbw", `${el.offsetWidth - el.clientWidth}px`);
    };
    el.addEventListener("scroll", sync, { passive: true });
    const ro = new ResizeObserver(sync);
    ro.observe(el);
    sync();
    return () => {
      el.removeEventListener("scroll", sync);
      ro.disconnect();
    };
  }

  const FILTERS: { id: TableFilter; label: string }[] = [
    { id: "all", label: "All" },
    { id: "changed", label: "Changed" },
    { id: "obd", label: "OBD" },
  ];

  // The popup edits a local copy (DataTable pattern); every change pushes the whole list to the store.
  let adding = $state(false);
  // svelte-ignore state_referenced_locally
  let seq = rules.reduce((m, r) => Math.max(m, r.key), 0);
  // svelte-ignore state_referenced_locally
  let draft = $state<FilterRule[]>(rules.map((r) => ({ ...r })));
  let wrapEl = $state<HTMLDivElement>();

  const modeOptions = [
    { value: false, label: "Include" },
    { value: true, label: "Exclude" },
  ];
  const fieldOptions = FILTER_FIELDS.map((f) => ({ value: f.id, label: f.label }));
  const hintFor = (field: FilterField) => (field === "dir" ? "RX or TX" : field === "dlc" ? "0-8" : "hex, e.g. 7E8");
  const invalid = (r: FilterRule) => r.value.trim() !== "" && parseFilterValue(r.field, r.value) === null;

  const commit = () => onrules(draft.map((r) => ({ ...r })));

  function addRule() {
    draft.push({ key: ++seq, field: "id", op: "=", value: "", exclude: false });
    commit();
  }

  function removeRule(key: number) {
    draft = draft.filter((r) => r.key !== key);
    commit();
  }

  function resetRules() {
    draft = [];
    commit();
  }

  function onField(r: FilterRule) {
    if (!opsFor(r.field).includes(r.op)) r.op = "=";
    commit();
  }

  const count = $derived(mode === "log" ? rows.length : byId.length);
</script>

<svelte:window onpointerdown={(e) => adding && !wrapEl?.contains(e.target as Node) && !(e.target as Element).closest(".select-menu-list") && (adding = false)} />

<section class="card ft" aria-label="Frames">
  <div class="ft-head">
    <div class="seg" role="radiogroup" aria-label="Table">
      <button role="radio" aria-checked={mode === "log"} onclick={() => onmode("log")}>Log</button>
      <button role="radio" aria-checked={mode === "byId"} onclick={() => onmode("byId")}>By ID</button>
    </div>
    <div class="seg" role="radiogroup" aria-label="Show">
      {#each FILTERS as f (f.id)}
        <button role="radio" aria-checked={filter === f.id} onclick={() => onfilter(f.id)}>{f.label}</button>
      {/each}
    </div>
    <div class="ft-tools">
      <div class="ft-add" bind:this={wrapEl}>
        <button class="btn sm" aria-expanded={adding} aria-haspopup="dialog" onclick={() => (adding = !adding)}>
          <Icon name="filter" size={16} />
          <span>Filter</span>
          {#if draft.length > 0}<span class="ft-badge">{draft.length}</span>{/if}
        </button>
        {#if adding}
          <div class="popup ft-pop" role="dialog" aria-label="Filters" tabindex="-1" onkeydown={(e) => e.key === "Escape" && (adding = false)}>
            <div class="ft-pop-h">Filter by</div>
            <div class="ft-rules">
              {#each draft as r, i (r.key)}
                <div class="ft-rule">
                  <span class="ft-lbl muted">{i === 0 ? "Where" : "And"}</span>
                  <div class="ft-sel"><SelectMenu ariaLabel="Include or exclude" options={modeOptions} bind:value={r.exclude} onChange={commit} /></div>
                  <div class="ft-sel"><SelectMenu ariaLabel="Row" options={fieldOptions} bind:value={r.field} onChange={() => onField(r)} /></div>
                  <div class="ft-sel ft-op"><SelectMenu ariaLabel="Operator" options={opsFor(r.field).map((o) => ({ value: o, label: o }))} bind:value={r.op} onChange={commit} /></div>
                  <input
                    class="in mono ft-val"
                    aria-label="Value"
                    placeholder={hintFor(r.field)}
                    bind:value={r.value}
                    oninput={commit}
                    aria-invalid={invalid(r) ? "true" : undefined}
                  />
                  <button class="btn sm icon" title="Remove rule" aria-label="Remove rule" onclick={() => removeRule(r.key)}><Icon name="trash" size={14} /></button>
                </div>
              {/each}
              {#if draft.length === 0}<p class="muted">No active filters.</p>{/if}
            </div>
            <div class="ft-foot">
              <button class="btn sm pri" onclick={addRule}>Add filter</button>
              {#if draft.length > 0}<button class="btn sm" onclick={resetRules}>Reset filters</button>{/if}
            </div>
          </div>
        {/if}
      </div>
      <span class="muted">{count} shown</span>
      <button class="btn sm" disabled={count === 0} onclick={onclear}>
        <Icon name="trash" size={16} />
        <span>Clear</span>
      </button>
    </div>
  </div>

  {#if count === 0}
    <p class="muted ft-empty">{draft.length || filter !== "all" ? "No frames match the filter." : "No frames. Press Start to log the bus."}</p>
  {:else}
    <div class="ft-body">
      <div class="ft-bar float-layer rest-top" aria-hidden="true" {@attach measureHeight("--hdr-h")}>
        <div class="ft-clip">
        <div class="ft-cols">
          <span>Time (s)</span>
          <span>Direction</span>
          <span>ID</span>
          <span class="hide-s">DLC</span>
          <span>Data</span>
          <span>Parsed</span>
        </div>
        </div>
      </div>
      <div class="ft-scroll" {@attach trackUnder} {@attach syncBar}>
      <table class="ft-tbl">
        <colgroup>
          <col class="w-t" />
          <col class="w-d" />
          <col class="w-i" />
          <col class="w-l hide-s" />
          <col class="w-b" />
          <col />
        </colgroup>
        <thead class="sr-only">
          <tr>
            <th scope="col">Time (s)</th>
            <th scope="col">Direction</th>
            <th scope="col">ID</th>
            <th scope="col" class="hide-s">DLC</th>
            <th scope="col">Data</th>
            <th scope="col">Parsed</th>
          </tr>
        </thead>
        <tbody>
          {#if mode === "log"}
            {#each rows as row (row)}
              {@render line(row)}
            {/each}
          {:else}
            {#each byId as row (row.key)}
              {@render line(row)}
            {/each}
          {/if}
        </tbody>
      </table>
      </div>
    </div>
  {/if}
</section>

{#snippet line(row: Row)}
  {@const key = idKey(row.id, row.ext)}
  <tr
    tabindex="0"
    class:sel={selected === key}
    onclick={() => onselect(key)}
    ondblclick={() => ondetails(key)}
    onkeydown={(e) => {
      if (e.key === "Enter") ondetails(key);
    }}
  >
    <td class="mono muted">{relTime(row.t, startUs)}</td>
    <td class="dir" class:tx={row.tx}>{row.tx ? "TX" : "RX"}</td>
    <td class="mono id" style:color="var(--c{idColorIndex(key)})">{formatId(row.id, row.ext)}</td>
    <td class="hide-s">{row.data.length}</td>
    <td class="mono data">
      {#if row.rtr}
        <span class="muted">remote request</span>
      {:else}
        {#each Array.from(row.data) as b, i (i)}
          <span class="byte" class:chg={(row.changed & (1 << i)) !== 0} class:z={b === 0}>{b.toString(16).toUpperCase().padStart(2, "0")}</span>
        {/each}
      {/if}
    </td>
    <td class="mono parsed" class:muted={!row.parsed}>{row.parsed || "-"}</td>
  </tr>
{/snippet}

<style>
  .ft {
    padding: 8px 12px;
    min-width: 0;
  }
  .ft-head {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
    padding: 4px 4px 8px;
  }
  .ft-tools {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  .ft-tools .btn {
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  .ft-add {
    position: relative;
  }
  .ft-pop {
    position: absolute;
    right: 0;
    top: calc(100% + 6px);
    z-index: 50;
    width: max-content;
    min-width: min(620px, calc(100vw - 2rem));
    max-width: calc(100vw - 2rem);
    padding: 1.25rem;
    font-size: 0.85rem;
  }
  .ft-pop-h {
    font-weight: 600;
    margin-bottom: 1rem;
  }
  .ft-rules {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
    margin-bottom: 1rem;
  }
  .ft-rule {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 0.5rem;
  }
  .ft-lbl {
    flex: 0 0 50px;
  }
  .ft-sel {
    flex: 1 1 0;
    min-width: 0;
  }
  .ft-op {
    flex: 0 0 90px;
  }
  .ft-val {
    flex: 1 1 0;
    min-width: 0;
    margin: 0;
  }
  .ft-foot {
    display: flex;
    gap: 0.75rem;
    padding-top: 1rem;
    border-top: 1px solid var(--hair);
  }
  .ft-badge {
    background: var(--ctl);
    padding: 0.1rem 0.4rem;
    border-radius: var(--r-xs);
    font-size: 0.65rem;
  }
  @media (max-width: 768px) {
    .ft-pop {
      position: fixed;
      top: 50%;
      left: 50%;
      right: auto;
      transform: translate(-50%, -50%);
      width: 90vw;
      min-width: 0;
      max-height: 85vh;
      overflow-y: auto;
      box-shadow:
        0 0 0 100vw var(--scrim),
        var(--shadow-float);
    }
    .ft-lbl {
      flex-basis: 100%;
    }
    .ft-sel,
    .ft-val {
      flex: 1 1 100%;
    }
    .ft-op {
      flex: 1 1 100%;
    }
  }
  .ft-empty {
    margin: 12px;
  }
  .ft-body {
    --w-t: 84px;
    --w-d: 92px;
    --w-i: 92px;
    --w-l: 56px;
    --w-b: 300px;
    --min: calc(var(--w-t) + var(--w-d) + var(--w-i) + var(--w-l) + var(--w-b) + 140px);
    position: relative;
  }
  .ft-scroll {
    overflow: auto;
    max-height: 60vh;
    border-radius: var(--r-ctl);
    padding-top: var(--hdr-h, 30px);
  }
  .ft-bar {
    position: absolute;
    top: 0;
    left: 0;
    right: var(--sbw, 0px);
    z-index: 2;
    border-radius: var(--r-ctl);
    box-shadow: none;
    pointer-events: none;
  }
  .ft-clip {
    overflow: hidden;
    border-radius: inherit;
  }
  .ft-cols {
    display: grid;
    grid-template-columns: var(--w-t) var(--w-d) var(--w-i) var(--w-l) var(--w-b) 1fr;
    min-width: var(--min);
    margin-left: var(--sl, 0px);
  }
  .ft-cols span {
    padding: 6px 12px;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    font-size: 0.75rem;
    font-weight: 800;
    color: var(--muted);
    white-space: nowrap;
  }
  .ft-tbl {
    width: 100%;
    min-width: var(--min);
    table-layout: fixed;
    border-collapse: separate;
    border-spacing: 0 4px;
    font-size: 0.92rem;
  }
  .w-t {
    width: var(--w-t);
  }
  .w-d {
    width: var(--w-d);
  }
  .w-i {
    width: var(--w-i);
  }
  .w-l {
    width: var(--w-l);
  }
  .w-b {
    width: var(--w-b);
  }
  .sr-only {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip-path: inset(50%);
  }
  .ft-tbl td {
    padding: 9px 12px;
    background: var(--row);
    white-space: nowrap;
    font-variant-numeric: tabular-nums;
  }
  .ft-tbl td:first-child {
    border-radius: var(--r-ctl) 0 0 var(--r-ctl);
    position: relative;
  }
  .ft-tbl td:last-child {
    border-radius: 0 var(--r-ctl) var(--r-ctl) 0;
  }
  .ft-tbl tbody tr {
    cursor: pointer;
  }
  .ft-tbl tbody tr:hover td {
    background: var(--hov);
  }
  .ft-tbl tr.sel td,
  .ft-tbl tr.sel:hover td {
    background: color-mix(in srgb, var(--accent) 24%, var(--hov));
  }
  .id {
    font-weight: 800;
    letter-spacing: 0.02em;
  }
  .byte {
    display: inline-block;
    min-width: 2.1ch;
    padding: 1px 3px;
    border-radius: 6px;
    text-align: center;
  }
  .byte + .byte {
    margin-left: 0.25em;
  }
  .byte.z {
    color: var(--muted);
  }
  .byte.chg {
    background: color-mix(in srgb, var(--accent) 30%, transparent);
    color: var(--accent-text);
    font-weight: 800;
  }
  .parsed:not(.muted) {
    color: var(--accent-text);
  }
  .dir {
    font-weight: 600;
    color: var(--ok);
  }
  .dir.tx {
    color: var(--accent-text);
  }
  @media (max-width: 859px) {
    .hide-s {
      display: none;
    }
    .ft-body {
      --min: calc(var(--w-t) + var(--w-d) + var(--w-i) + var(--w-b) + 140px);
    }
    .ft-cols {
      grid-template-columns: var(--w-t) var(--w-d) var(--w-i) var(--w-b) 1fr;
    }
  }
</style>
