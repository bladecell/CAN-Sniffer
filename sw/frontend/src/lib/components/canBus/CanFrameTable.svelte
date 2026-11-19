<script lang="ts">
  import type { LogRow, IdView, TableFilter } from "$lib/canBus/canModel";
  import type { TableMode } from "$lib/canBus/canBusStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import { placePopup } from "$lib/popoverAnchor";
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

  // Column visibility (DataTable "View" menu). The last shown column takes the spare width.
  // The data bytes are one column each (B0..B7); the "Data" menu entry hides or shows all of them.
  type Col = { key: string; label: string; w: number };
  const BYTE_COLS: Col[] = Array.from({ length: 8 }, (_, i) => ({ key: `b${i}`, label: `B${i}`, w: 46 }));
  const BYTE_KEYS = BYTE_COLS.map((c) => c.key);
  const COLS: Col[] = [
    { key: "t", label: "Time (s)", w: 84 },
    { key: "d", label: "Direction", w: 92 },
    { key: "i", label: "ID", w: 92 },
    { key: "l", label: "DLC", w: 56 },
    ...BYTE_COLS,
    { key: "p", label: "Parsed", w: 140 },
  ];
  let hidden = $state<Set<string>>(new Set());
  let viewing = $state(false);
  let viewEl = $state<HTMLDivElement>();
  const vis = $derived(COLS.filter((c) => !hidden.has(c.key)));
  const show = $derived(Object.fromEntries(COLS.map((c) => [c.key, !hidden.has(c.key)])) as Record<string, boolean>);
  const bytesShown = $derived(BYTE_KEYS.filter((k) => !hidden.has(k)).length);
  const cols = $derived(vis.map((c, i) => (i === vis.length - 1 ? "1fr" : `${c.w}px`)).join(" "));
  const minW = $derived(vis.reduce((n, c) => n + c.w, 0));

  function toggleCol(key: string) {
    const next = new Set(hidden);
    if (!next.delete(key)) next.add(key);
    hidden = next;
  }

  function toggleData() {
    hidden = bytesShown > 0 ? new Set([...hidden, ...BYTE_KEYS]) : new Set([...hidden].filter((k) => !BYTE_KEYS.includes(k)));
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

  // Scrolled away from the top: hold the rows so new frames do not push them around. Back at the top, it follows the bus again.
  let held = $state.raw<{ rows: LogRow[]; byId: IdView[] } | null>(null);
  const shownRows = $derived(held?.rows ?? rows);
  const shownById = $derived(held?.byId ?? byId);

  // A new view (table, filter, rules) must not show the old frozen rows.
  $effect(() => {
    mode;
    filter;
    rules;
    held = null;
  });

  function onScroll(e: Event) {
    const top = (e.currentTarget as HTMLElement).scrollTop;
    if (top > 4 && !held) held = { rows, byId };
    else if (top <= 4 && held) held = null;
  }

  const count = $derived(mode === "log" ? shownRows.length : shownById.length);
</script>

<svelte:window
  onpointerdown={(e) => {
    const t = e.target as Element;
    if (adding && !wrapEl?.contains(t) && !t.closest(".select-menu-list")) adding = false;
    if (viewing && !viewEl?.contains(t)) viewing = false;
  }}
/>

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
          <div class="popup ft-pop" use:placePopup={() => wrapEl} role="dialog" aria-label="Filters" tabindex="-1" onkeydown={(e) => e.key === "Escape" && (adding = false)}>
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
                  <button type="button" class="ft-ib" title="Remove rule" aria-label="Remove rule" onclick={() => removeRule(r.key)}><Icon name="trash" size={20} /></button>
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
      <div class="ft-add" bind:this={viewEl}>
        <button class="btn sm" aria-expanded={viewing} aria-haspopup="menu" onclick={() => (viewing = !viewing)}>
          <Icon name="eye" size={16} />
          <span>View</span>
          {#if hidden.size > 0}<span class="ft-badge">{hidden.size}</span>{/if}
        </button>
        {#if viewing}
          <div class="popup ft-view" use:placePopup={() => viewEl} role="menu" tabindex="-1" onkeydown={(e) => e.key === "Escape" && (viewing = false)}>
            {#each COLS as c (c.key)}
              {#if c.key === "b0"}
                <button type="button" role="menuitemcheckbox" aria-checked={bytesShown > 0} class="ft-col" onclick={toggleData}>
                  <span>Data</span>
                  {#if bytesShown > 0}<Icon name="check" size={14} />{/if}
                </button>
              {/if}
              <button type="button" role="menuitemcheckbox" aria-checked={!hidden.has(c.key)} class="ft-col" class:ft-sub={c.key.startsWith("b")} onclick={() => toggleCol(c.key)}>
                <span>{c.label}</span>
                {#if !hidden.has(c.key)}<Icon name="check" size={14} />{/if}
              </button>
            {/each}
          </div>
        {/if}
      </div>
      <span class="muted">{count} shown</span>
      <button class="btn sm" disabled={count === 0} onclick={() => {
          held = null;
          onclear();
        }}>
        <Icon name="trash" size={16} />
        <span>Clear</span>
      </button>
    </div>
  </div>

  {#if count === 0}
    <p class="muted ft-empty">{draft.length || filter !== "all" ? "No frames match the filter." : "No frames. Press Start to log the bus."}</p>
  {:else}
    <div class="ft-body" style:--cols={cols} style:--min="{minW}px">
      <div class="ft-bar float-layer rest-top" aria-hidden="true" {@attach measureHeight("--hdr-h")}>
        <div class="ft-clip">
        <div class="ft-cols">
          {#each vis as c (c.key)}<span>{c.label}</span>{/each}
        </div>
        </div>
      </div>
      <div class="ft-scroll" onscroll={onScroll} {@attach trackUnder} {@attach syncBar}>
      <table class="ft-tbl">
        <colgroup>
          {#each vis as c, i (c.key)}<col style:width={i === vis.length - 1 ? undefined : `${c.w}px`} />{/each}
        </colgroup>
        <thead class="sr-only">
          <tr>
            {#each vis as c (c.key)}<th scope="col">{c.label}</th>{/each}
          </tr>
        </thead>
        <tbody>
          {#if mode === "log"}
            {#each shownRows as row (row)}
              {@render line(row)}
            {/each}
          {:else}
            {#each shownById as row (row.key)}
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
    {#if show.t}<td class="mono muted">{relTime(row.t, startUs)}</td>{/if}
    {#if show.d}<td class="dir" class:tx={row.tx}>{row.tx ? "TX" : "RX"}</td>{/if}
    {#if show.i}<td class="mono id" style:color="var(--c{idColorIndex(key)})">{formatId(row.id, row.ext)}</td>{/if}
    {#if show.l}<td>{row.data.length}</td>{/if}
    {#each BYTE_COLS as c, i (c.key)}
      {#if show[c.key]}
        <td class="bcell">
          {#if i < row.data.length && !row.rtr}
            {@const b = row.data[i]}
            <div class="tile" class:z={b === 0} class:chg={(row.changed & (1 << i)) !== 0}>{b.toString(16).toUpperCase().padStart(2, "0")}</div>
          {/if}
        </td>
      {/if}
    {/each}
    {#if show.p}<td class="mono parsed" class:muted={!row.parsed && !row.rtr}>{row.rtr ? "remote request" : row.parsed || "-"}</td>{/if}
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
    position: fixed;
    z-index: 50;
    box-sizing: border-box;
    width: max-content;
    min-width: min(540px, calc(100vw - 16px));
    overflow-y: auto;
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
    gap: 6px;
    padding: 6px 6px 6px 10px;
    border-radius: var(--r-track);
    background: var(--row);
  }
  .ft-ib {
    display: grid;
    flex: none;
    place-items: center;
    box-sizing: border-box;
    width: 44px;
    height: 44px;
    min-width: 0;
    min-height: 0;
    margin: 0 0 0 auto;
    padding: 0;
    border: 0;
    border-radius: calc(var(--r-ctl) - 2px);
    background: none;
    color: var(--ctl-crit);
    cursor: pointer;
  }
  .ft-ib:hover {
    background: var(--ctl);
  }
  .ft-ib:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .ft-lbl {
    flex: 0 0 44px;
  }
  .ft-sel {
    flex: 0 0 auto;
    width: 98px;
  }
  .ft-sel:nth-of-type(2) {
    width: 118px;
  }
  .ft-op {
    width: 66px;
  }
  .ft-sel :global(.select-menu-trigger) {
    padding: 0 8px 0 10px;
    gap: 4px;
  }
  .ft-sel :global(.select-menu-trigger svg) {
    width: 16px;
    height: 16px;
  }
  .ft-val {
    flex: 1 1 90px;
    min-width: 0;
    margin: 0;
  }
  .ft-foot {
    display: flex;
    gap: 0.75rem;
    padding-top: 1rem;
    border-top: 1px solid var(--hair);
  }
  .ft-view {
    position: fixed;
    z-index: 50;
    box-sizing: border-box;
    overflow-y: auto;
    width: 220px;
    padding: 0.5rem;
    display: flex;
    flex-direction: column;
  }
  .ft-col {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin: 0;
    padding: 0.6rem 0.5rem;
    border: 0;
    border-radius: 4px;
    background: transparent;
    color: var(--text);
    font-size: 0.85rem;
    text-align: left;
    cursor: pointer;
  }
  .ft-col:hover {
    background: var(--hov);
  }
  .ft-badge {
    background: var(--ctl);
    padding: 0.1rem 0.4rem;
    border-radius: var(--r-xs);
    font-size: 0.65rem;
  }
  @media (max-width: 768px) {
    .ft-pop {
      padding: 1rem;
    }
    .ft-sel:nth-of-type(2) {
      flex: 1 1 120px;
      width: auto;
    }
  }
  .ft-empty {
    margin: 12px;
  }
  .ft-body {
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
    grid-template-columns: var(--cols);
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
  .ft-sub {
    padding-left: 1.5rem;
  }
  .ft-tbl td.bcell {
    padding: 4px 3px;
  }
  .tile {
    position: relative;
    display: grid;
    place-items: center;
    height: 28px;
    border-radius: 7px;
    background: var(--card2);
    font-family: var(--mono);
    font-weight: 700;
    font-size: 0.84rem;
    font-variant-numeric: tabular-nums;
  }
  .tile.z {
    color: var(--muted);
    font-weight: 500;
    opacity: 0.7;
  }
  .tile.chg {
    background: color-mix(in srgb, var(--accent) 26%, transparent);
    color: var(--text);
  }
  .tile.chg::after {
    content: "";
    position: absolute;
    top: 0;
    right: 0;
    width: 7px;
    height: 7px;
    border-top-right-radius: 7px;
    background: linear-gradient(225deg, var(--accent) 50%, transparent 50%);
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
</style>
